
#include "qemu/osdep.h"
#include "hw/irq.h"
#include "hw/ssi/ssi.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "migration/register.h"
#include "migration/qemu-file.h"
#include "ui/console.h"
#include "hw/registerfields.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "hw/arm/z_model.h"


#define TYPE_SSI_MAX11254 "ssi-max11254"
OBJECT_DECLARE_SIMPLE_TYPE(ssi_max11254_state, SSI_MAX11254)

struct ssi_max11254_state {
    SSIPeripheral parent_obj;

    qemu_irq int1[1];

    ptimer_state *ptimer;

    uint8_t id;

    int cur_xfer_pos;

    uint8_t cmd;
    bool is_register;
    bool is_timer_running;
    uint8_t rsp_buffer[16];

    uint8_t regs[0xFF];

    z_model_state *p_model;
};

#define COMMAND_BYTE        0x80
#define REGISTER_BYTE       0xC0
#define REGISTER_WRITE      0x00
#define REGISTER_READ       0x01

#define DATA0_ADDR          0x0E

#define COMMAND_PD          0x10
#define COMMAND_CAL         0x20
#define COMMAND_SEQ         0x30

#define RATE_9              0x09


REG8(MAX_SEQ, COMMAND_SEQ)
    FIELD(MAX_SEQ, MODE, 3, 2)
REG8(MAX_READ, DATA0_ADDR)

FIELD(MAX_CONV, RATE, 0, 4)
FIELD(MAX_CONV, MODE, 5, 2)

FIELD(MAX_REG, RW, 0, 1)
FIELD(MAX_REG, ADDR, 1, 5)

FIELD(MAX_CMD, B6, 6, 1)

static uint32_t _transfer(SSIPeripheral *dev, uint32_t value)
{
    ssi_max11254_state *s = SSI_MAX11254(dev);
    uint32_t ret = 0;

    const uint8_t value8 = value & 0xFFu;

    //info_report("MAX#%u byte: [value: 0x%02X] (pos=%u)", s->id, value8, s->cur_xfer_pos);

    if (s->cur_xfer_pos == 0) {

        s->cmd = value8;
        s->is_register = (value8 & R_MAX_CMD_B6_MASK) ? true:false;

        if (!s->is_register) {

            // conversion
            uint8_t mode = s->cmd & R_MAX_CONV_MODE_MASK;
            mode >>= R_MAX_CONV_MODE_SHIFT;
            info_report("MAX#%u SEQ: [value: 0x%02X] mode=%u", s->id, value8, mode);
            if (mode) {
                ptimer_transaction_begin(s->ptimer);
                ptimer_stop(s->ptimer);
                ptimer_set_freq(s->ptimer, 1000+s->id); // TODO Rate_9 = 1.3ms conversion time ?
                ptimer_set_count(s->ptimer, 13);
                ptimer_set_limit(s->ptimer, 13, 13);
                ptimer_run(s->ptimer, mode != 3);
                ptimer_transaction_commit(s->ptimer);
            }
        }

    } else if (s->cur_xfer_pos == 1) {

        if (s->is_register) {

            if (s->cmd & R_MAX_REG_RW_MASK) {
                // read
                ret = s->regs[s->cmd & R_MAX_REG_ADDR_MASK];
            } else {
                // write
                s->regs[s->cmd & R_MAX_REG_ADDR_MASK] = value8;
            }

        }

    }

    if (s->cur_xfer_pos >= 1) {
        ret = s->rsp_buffer[s->cur_xfer_pos-1];
    }

    s->cur_xfer_pos++;

    return ret;
}

static int _set_cs(SSIPeripheral *dev, bool select) {

    // called when the state of the CS line changes
    ssi_max11254_state *s = SSI_MAX11254(dev);

    //info_report("MAX#%u CS %d", s->id, select);

    if (select) { // select true = cs high
    }

    s->cmd = 0;
    s->is_register = true;
    memset(s->rsp_buffer, 0, sizeof(s->rsp_buffer));

    s->cur_xfer_pos = 0;

    return 0;
}

static void timer_hit(void *opaque)
{
    struct ssi_max11254_state *s = opaque;

    //info_report("MAX#%u DRDY", s->id);

    // toggle irq
    qemu_set_irq(s->int1[0], true);
    qemu_set_irq(s->int1[0], false);
}

static int max11254_post_load(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_ssi_max11254 = {
        .name = "ssi_max11254",
        .version_id = 1,
        .minimum_version_id = 1,
        .post_load = max11254_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_SSI_PERIPHERAL(parent_obj, ssi_max11254_state),
                VMSTATE_END_OF_LIST()
        },
};

static void max11254_realize(SSIPeripheral *d, Error **errp)
{
    ssi_max11254_state *s = SSI_MAX11254(d);

    d->cs = true; // cs line default state

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_DEFAULT);
}

static void ssi_max11254_reset(DeviceState *dev)
{

}

static void max11254_instance_init(Object *obj)
{
    ssi_max11254_state *s = SSI_MAX11254(obj);

    qdev_init_gpio_out_named(DEVICE(s), s->int1, "DRDY", 1);

    vmstate_register(NULL, 0, &vmstate_ssi_max11254, s);
}

static Property _properties[] = {
        DEFINE_PROP_UINT8("ID", ssi_max11254_state, id, 0),
        DEFINE_PROP_LINK("model", ssi_max11254_state, p_model,
                         TYPE_ZMODEL, z_model_state *),
        DEFINE_PROP_END_OF_LIST(),
};

static void max11254_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = max11254_realize;
    k->transfer = _transfer;
    k->cs_polarity = SSI_CS_LOW;
    k->set_cs = _set_cs;
    dc->vmsd = &vmstate_ssi_max11254;
    dc->reset = ssi_max11254_reset;
    /* Reason: init() method uses drive_get_next() */
    dc->user_creatable = false;

    device_class_set_props(dc, _properties);
}

static const TypeInfo max11254_info = {
        .name          = TYPE_SSI_MAX11254,
        .parent        = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(ssi_max11254_state),
        .instance_init = max11254_instance_init,
        .class_init    = max11254_class_init,
};

static void max11254_register_types(void)
{
    type_register_static(&max11254_info);
}

type_init(max11254_register_types)
