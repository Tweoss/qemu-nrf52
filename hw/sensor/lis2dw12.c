
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
#include "hw/i2c/i2c.h"


#define TYPE_LSM6DW12 "lsm6dw12"
OBJECT_DECLARE_SIMPLE_TYPE(lis12_state, LSM6DW12)

#define FIFO_SIZE      256
#define FIFO_MASK_SIZE 0xFF

typedef struct __attribute((packed)) {
    uint8_t tag;
    uint8_t raw_data[6];
} sLSMfifo;

struct lis12_state {
    SSIPeripheral parent_obj;

    qemu_irq int1[1];

    ptimer_state *ptimer;

    uint8_t len;
    uint8_t pointer;
    bool pending_clear;

    uint8_t cmd;
    bool is_read;
    bool is_timer_running;
    uint8_t rsp_buffer[16];

    uint8_t regs[0xFF];

    uint8_t read_level;
    uint8_t fifo_level;
    uint8_t fifo_diff;
    sLSMfifo fifo_buffer[FIFO_SIZE];

    bool uses_int1;

    z_model_state *p_model;
};

REG8(LSM_INT1_CTRL, 0x0D)
    FIELD(LSM_INT1_CTRL, FIFO, 3, 3)

REG8(LSM_WHO_AM_I, 0x0F)

REG8(LSM_CTRL1, 0x20)
    FIELD(LSM_CTRL1, LP, 0, 2)
    FIELD(LSM_CTRL1, MODE, 2, 2)
    FIELD(LSM_CTRL1, ODR, 4, 2)

REG8(LSM_CTRL2, 0x21)
    FIELD(LSM_CTRL2, SOFT_RESET, 6, 1)

REG8(LSM_CTRL4, 0x23U)
    FIELD(LSM_CTRL4, INT1_DRDY, 0, 1)

REG8(LSM_STATUS, 0x27U)
    FIELD(LSM_STATUS, DRDY, 0, 1)

REG8(LIS2DW12_OUT_X_L, 0x28U)
REG8(LIS2DW12_OUT_Z_H, 0x2DU)

static void timer_hit(void *opaque)
{
    struct lis12_state *s = opaque;

    z_model_acc p_acc[1];
    z_model__compute_acc(s->p_model, p_acc);

    if (s->uses_int1 &&
        (s->regs[R_LSM_STATUS] & R_LSM_STATUS_DRDY_MASK)) {
        qemu_set_irq(s->int1[0], true);
    } else {
        qemu_set_irq(s->int1[0], false);
    }

}

static void _data_read_clear(struct lis12_state *s) {

    s->regs[R_LSM_STATUS] &= ~(R_LSM_STATUS_DRDY_MASK);

    qemu_set_irq(s->int1[0], false);

    s->pending_clear = false;
}

static void lis12_reset(struct lis12_state *s);

static void lis12_write(struct lis12_state *s, uint8_t data)
{

    s->regs[s->pointer+s->len-1u] = data; // fill data always

    // process events
    if (s->len == 1) { // handle registers

        switch (s->pointer) {

            case A_LSM_CTRL2:
                if (data & R_LSM_CTRL2_SOFT_RESET_MASK) {
                    lis12_reset(s);
                }
                break;

            case A_LSM_CTRL4:
                if (data & R_LSM_CTRL4_INT1_DRDY_MASK) {
                    s->uses_int1 = true;
                    info_report("INT enabled !");
                    ptimer_transaction_begin(s->ptimer);
                    ptimer_stop(s->ptimer);
                    ptimer_set_freq(s->ptimer, 25);
                    ptimer_set_count(s->ptimer, 1);
                    ptimer_set_limit(s->ptimer, 1, 1);
                    ptimer_run(s->ptimer, 0);
                    ptimer_transaction_commit(s->ptimer);
                } else {
                    s->uses_int1 = false;
                    ptimer_transaction_begin(s->ptimer);
                    ptimer_stop(s->ptimer);
                    ptimer_transaction_commit(s->ptimer);
                }
                break;

            default:
                break;
        }
    }
}

static int lis12_tx(I2CSlave *i2c, uint8_t data)
{
    struct lis12_state *s = LSM6DW12(i2c);

    info_report("lis12: lis12_tx 0x%02X (pos=%u)", data, s->len);

    if (s->len == 0) {
        /* first byte is the register pointer for a read or write
         * operation */
        s->pointer = data;
    } else {
        lis12_write(s, data);
    }
    s->len++;

    return 0;
}

static void lis12_read_events_process(struct lis12_state *s)
{

    info_report("lis12: Device read reg= %02X", s->pointer);

    switch (s->pointer) {

        case A_LIS2DW12_OUT_X_L ... A_LIS2DW12_OUT_Z_H:
            s->pending_clear = true;
            break;

        default:
            break;
    }
}

static uint8_t lis12_rx(I2CSlave *i2c)
{
    struct lis12_state *s = LSM6DW12(i2c);

    if (s->pointer + s->len < sizeof(s->regs)) {
        uint8_t ret = s->regs[s->pointer + s->len];
        lis12_read_events_process(s);
        s->len++;
        return ret;
    } else {
        return 0xff;
    }
}

static int lis12_event(I2CSlave *i2c, enum i2c_event event)
{
    struct lis12_state *s = LSM6DW12(i2c);

    s->len = 0;

    if (event == I2C_START_SEND) {
//        warn_report("-------------------");
//        warn_report("lis12 I2C_START_SEND");
        s->pointer = 0;
    } else if (event == I2C_START_RECV) {
    } else if (event == I2C_FINISH) {
        if (s->pending_clear) {
            _data_read_clear(s);
        }
    }

    return 0;
}

static void lis12_reset(struct lis12_state *s)
{
    s->pointer = 0;

    if (s->ptimer) {
        ptimer_transaction_begin(s->ptimer);
        ptimer_stop(s->ptimer);
        ptimer_transaction_commit(s->ptimer);
    }

    s->pending_clear = false;

    info_report("Device reset");
}

static Property _properties[] = {
        DEFINE_PROP_LINK("model", lis12_state, p_model,
                         TYPE_ZMODEL, z_model_state *),
        DEFINE_PROP_END_OF_LIST(),
};


static int lis12_post_load(void *opaque, int version_id)
{
    return 0;
}

static const VMStateDescription vmstate_ssi_lis12 = {
        .name = TYPE_LSM6DW12,
        .version_id = 1,
        .minimum_version_id = 1,
        .post_load = lis12_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_SSI_PERIPHERAL(parent_obj, lis12_state),
                VMSTATE_END_OF_LIST()
        },
};

static void lis12_instance_init(Object *obj)
{
    lis12_state *s = LSM6DW12(obj);

    lis12_reset(s);
}

static void lis12_realize(DeviceState *dev, Error **errp)
{
    lis12_state *s = LSM6DW12(dev);

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_DEFAULT);

    qdev_init_gpio_out_named(DEVICE(s), s->int1, "DRDY", 1);

    vmstate_register(NULL, 0, &vmstate_ssi_lis12, s);
}

static void lis12_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);

    dc->realize = lis12_realize;
    k->event = lis12_event;
    k->recv = lis12_rx;
    k->send = lis12_tx;
    dc->vmsd = &vmstate_ssi_lis12;
//    dc->reset = lis12_reset;

    device_class_set_props(dc, _properties);
}

static const TypeInfo lis12_info = {
        .name          = TYPE_LSM6DW12,
        .parent        = TYPE_SSI_PERIPHERAL,
        .instance_size = sizeof(lis12_state),
        .class_size    = sizeof(lis12_state),
        .instance_init = lis12_instance_init,
        .class_init    = lis12_class_init,
};

static void lis12_register_types(void)
{
    type_register_static(&lis12_info);
}

type_init(lis12_register_types)
