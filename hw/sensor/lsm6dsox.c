
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
#include "hw/arm/z_model.h"
#include "hw/qdev-properties.h"


#define TYPE_SSI_LSM "ssi-lsm6dsox"
OBJECT_DECLARE_SIMPLE_TYPE(ssi_dsox_state, SSI_LSM)

#define FIFO_SIZE      256
#define FIFO_MASK_SIZE 0xFF

struct ssi_dsox_state {
    SSIPeripheral parent_obj;

    qemu_irq int1[1];

    ptimer_state *ptimer;

    int cur_xfer_pos;

    uint8_t cmd;
    bool is_read;
    bool is_timer_running;
    uint8_t rsp_buffer[16];

    uint8_t regs[0xFF];

    uint8_t read_level;
    uint8_t fifo_level;
    uint8_t fifo_diff;
    z_model_acc fifo_buffer[FIFO_SIZE];

    bool uses_int1;

    z_model_state *p_model;
};


#define LSM6DSOX_CTRL1_XL                     0x10U

REG8(LSM_INT1_CTRL, 0x0D)
    FIELD(LSM_INT1_CTRL, FIFO, 3, 3)
REG8(LSM_WHO_AM_I, 0x0F)
REG8(LSM_CTRL1_XL, (0x10U & 0x7F))
    FIELD(LSM_CTRL1_XL, ODR, 4, 4)

REG8(LSM_FIFO1_STATUS, (0xBA & 0x7F))
REG8(LSM_FIFO2_STATUS, (0xBB & 0x7F))
REG8(LSM_FIFO_DATA_OUT_TAG, (0xF8 & 0x7F))
REG8(LSM_FIFO_DATA_OUT_XL, (0xF9 & 0x7F))


static uint32_t _transfer(SSIPeripheral *dev, uint32_t value)
{
    ssi_dsox_state *s = SSI_LSM(dev);
    uint32_t ret = 0;

    const uint8_t value8 = value & 0xFFu;

    //info_report("LSM byte: [value: 0x%02X]", value8);

    if (s->cur_xfer_pos == 0) {

        //info_report("LSM cmd: [value: 0x%02X]", value8);

        s->cmd = value8 & 0x7F;
        s->is_read = (value8 & 0x80) ? true:false;

    } else if (s->cur_xfer_pos == 1) {

        //info_report("LSM byte: [value: 0x%02X]", value8);

        if (s->is_read) {

            // prepare answer
            switch (s->cmd) {
                case A_LSM_WHO_AM_I:
                    s->rsp_buffer[0] = 0x6C;
                    break;
                case A_LSM_FIFO1_STATUS:
                    s->rsp_buffer[0] = s->fifo_diff & 0xFF;
                    break;
                case A_LSM_FIFO2_STATUS:
                    s->rsp_buffer[0] = ((s->fifo_diff & 0x0300) >> 8) & 0xFF;
                    break;
                case A_LSM_FIFO_DATA_OUT_XL:
                    //info_report("A_LSM_FIFO_DATA_OUT_XL %u %u", s->fifo_level, s->read_level);
                    memcpy(s->rsp_buffer, &s->fifo_buffer[s->read_level], sizeof(z_model_acc));
                    s->read_level += 1;
                    s->fifo_diff = s->fifo_level - s->read_level;
                    break;
                default:
                    ret = s->regs[s->cmd];
                    break;
            }
        } else {
            s->regs[s->cmd] = value8;

            if (s->cmd == A_LSM_CTRL1_XL && (value8 & R_LSM_CTRL1_XL_ODR_MASK) && !s->is_timer_running) {

                s->is_timer_running = true;

                ptimer_transaction_begin(s->ptimer);
                ptimer_stop(s->ptimer);
                ptimer_set_freq(s->ptimer, 1666);
                ptimer_set_count(s->ptimer, 1);
                ptimer_set_limit(s->ptimer, 1, 1);
                ptimer_run(s->ptimer, 0);
                ptimer_transaction_commit(s->ptimer);
            }

            if (s->cmd == A_LSM_INT1_CTRL) {
                s->uses_int1 = (value8 & R_LSM_INT1_CTRL_FIFO_MASK) ? true : false;
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
    ssi_dsox_state *s = SSI_LSM(dev);

    if (select) { // select true = cs high
    }

    s->cmd = 0;
    s->is_read = true;
    memset(s->rsp_buffer, 0, sizeof(s->rsp_buffer));

    s->cur_xfer_pos = 0;

    return 0;
}

static int lsm6dsox_post_load(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_ssi_lsm6dsox = {
        .name = "ssi_lsm6dsox",
        .version_id = 1,
        .minimum_version_id = 1,
        .post_load = lsm6dsox_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_SSI_PERIPHERAL(parent_obj, ssi_dsox_state),
                VMSTATE_END_OF_LIST()
        },
};

static void timer_hit(void *opaque)
{
    struct ssi_dsox_state *s = opaque;

    z_model__compute_acc(s->p_model, &s->fifo_buffer[s->fifo_level]);

    s->fifo_level += 1;
    s->fifo_diff = s->fifo_level - s->read_level;

    if (s->uses_int1 && s->fifo_level > 16) { // watermark
        qemu_set_irq(s->int1[0], true);
    } else {
        qemu_set_irq(s->int1[0], false);
    }

}

static void lsm6dsox_realize(SSIPeripheral *d, Error **errp)
{
    ssi_dsox_state *s = SSI_LSM(d);

    d->cs = true; // cs line default state

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_DEFAULT);
}

static void ssi_lsm6dsox_reset(DeviceState *dev)
{

}

static void lsm6dsox_instance_init(Object *obj)
{
    ssi_dsox_state *s = SSI_LSM(obj);

    qdev_init_gpio_out_named(DEVICE(s), s->int1, "INT1", 1);

    vmstate_register(NULL, 0, &vmstate_ssi_lsm6dsox, s);
}

static Property _properties[] = {
        DEFINE_PROP_LINK("model", ssi_dsox_state, p_model,
                         TYPE_ZMODEL, z_model_state *),
        DEFINE_PROP_END_OF_LIST(),
};

static void lsm6dsox_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SSIPeripheralClass *k = SSI_PERIPHERAL_CLASS(klass);

    k->realize = lsm6dsox_realize;
    k->transfer = _transfer;
    k->cs_polarity = SSI_CS_LOW;
    k->set_cs = _set_cs;
    dc->vmsd = &vmstate_ssi_lsm6dsox;
    dc->reset = ssi_lsm6dsox_reset;

    device_class_set_props(dc, _properties);
}

static const TypeInfo lsm6dsox_info = {
        .name          = TYPE_SSI_LSM,
        .parent        = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(ssi_dsox_state),
        .instance_init = lsm6dsox_instance_init,
        .class_init    = lsm6dsox_class_init,
};

static void lsm6dsox_register_types(void)
{
    type_register_static(&lsm6dsox_info);
}

type_init(lsm6dsox_register_types)
