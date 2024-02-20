//
// Created by vince on 05/03/2023.
//

#include "qemu/osdep.h"
#include "hw/i2c/i2c.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qapi/visitor.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "hw/arm/z_model.h"
#include "hw/ptimer.h"
#include "hw/qdev-properties.h"
#include "qemu/error-report.h"

#ifndef UINT8_C
#define UINT8_C(X)      ((uint8_t)X)
#define INT8_C(X)       ((int8_t)X)
#endif

/**\name Soft reset command */
#define BMP3_SOFT_RESET                         UINT8_C(0xB6)

/**
 * The last error code from read/write interface is stored in the device structure as intf_rslt.
 */
#ifndef BMP3_INTF_RET_SUCCESS
#define BMP3_INTF_RET_SUCCESS                   INT8_C(0)
#endif

/**\name I2C addresses */
#define BMP3_ADDR_I2C_PRIM                      UINT8_C(0x76)
#define BMP3_ADDR_I2C_SEC                       UINT8_C(0x77)

/**\name BMP3 chip identifier */
#define BMP3_CHIP_ID                            UINT8_C(0x50)
#define BMP390_CHIP_ID                          UINT8_C(0x60)

/**\name Macros related to size */
#define BMP3_LEN_CALIB_DATA                     UINT8_C(21)
#define BMP3_LEN_P_AND_T_HEADER_DATA            UINT8_C(7)
#define BMP3_LEN_P_OR_T_HEADER_DATA             UINT8_C(4)
#define BMP3_LEN_P_T_DATA                       UINT8_C(6)
#define BMP3_LEN_GEN_SETT                       UINT8_C(7)
#define BMP3_LEN_P_DATA                         UINT8_C(3)
#define BMP3_LEN_T_DATA                         UINT8_C(3)
#define BMP3_LEN_SENSOR_TIME                    UINT8_C(3)
#define BMP3_FIFO_MAX_FRAMES                    UINT8_C(73)

typedef struct DeviceInfo {
    int model;
    const char *name;
} DeviceInfo;

static const DeviceInfo devices[] = {
    { BMP3_CHIP_ID, "bmp3" },
    { BMP390_CHIP_ID, "bmp390" },
};

struct BMP3State {
    /*< private >*/
    I2CSlave i2c;
    /*< public >*/

    uint8_t len;
    uint8_t pointer;
    bool pending_clear;

    ptimer_state *ptimer;

    bool uses_int1;
    bool int_level;
    qemu_irq int1[1];

    z_model_state *p_model;

    uint8_t regs[0xFF];
};

struct BMP3Class {
    I2CSlaveClass parent_class;
    DeviceInfo *dev;
};

#define TYPE_BMP3 "BMP3-generic"
OBJECT_DECLARE_TYPE(BMP3State, BMP3Class, BMP3)

/**\name BMP3 pressure settling time (micro secs)*/
#define BMP3_SETTLE_TIME_PRESS                  UINT16_C(392)

/**\name BMP3 temperature settling time (micro secs) */
#define BMP3_SETTLE_TIME_TEMP                   UINT16_C(313)

/**\name BMP3 adc conversion time (micro secs) */
#define BMP3_ADC_CONV_TIME                      UINT16_C(2000)

/**\name Register Address */
REG8( BMP3_REG_CHIP_ID,                        UINT8_C(0x00))
REG8( BMP3_REG_ERR,                            UINT8_C(0x02))
REG8( BMP3_REG_SENS_STATUS,                    UINT8_C(0x03))
REG8( BMP3_REG_DATA,                           UINT8_C(0x04))
REG8( BMP3_REG_EVENT,                          UINT8_C(0x10))
REG8( BMP3_REG_INT_STATUS,                     UINT8_C(0x11))
REG8( BMP3_REG_FIFO_LENGTH,                    UINT8_C(0x12))
REG8( BMP3_REG_FIFO_DATA,                      UINT8_C(0x14))
REG8( BMP3_REG_FIFO_WM,                        UINT8_C(0x15))
REG8( BMP3_REG_FIFO_CONFIG_1,                  UINT8_C(0x17))
REG8( BMP3_REG_FIFO_CONFIG_2,                  UINT8_C(0x18))
REG8( BMP3_REG_INT_CTRL,                       UINT8_C(0x19))
REG8( BMP3_REG_IF_CONF,                        UINT8_C(0x1A))
REG8( BMP3_REG_PWR_CTRL,                       UINT8_C(0x1B))
REG8( BMP3_REG_OSR,                            UINT8_C(0X1C))
REG8( BMP3_REG_ODR,                            UINT8_C(0x1D))
REG8( BMP3_REG_CONFIG,                         UINT8_C(0x1F))
REG8( BMP3_REG_CALIB_DATA,                     UINT8_C(0x31))
REG8( BMP3_REG_CMD,                            UINT8_C(0x7E))

/**\name Status macros */
#define BMP3_CMD_RDY                            UINT8_C(0x10)
#define BMP3_DRDY_PRESS                         UINT8_C(0x20)
#define BMP3_DRDY_TEMP                          UINT8_C(0x40)

#define BMP3_INT_LEVEL                          UINT8_C(0x02)
#define BMP3_INT_DRDY_EN                        UINT8_C(0x40)

#define BMP3_INT_PIN_ACTIVE_HIGH                UINT8_C(0x01)
#define BMP3_INT_PIN_ACTIVE_LOW                 UINT8_C(0x00)

/**\name Power mode macros */
#define BMP3_MODE_SLEEP_VAL                         UINT8_C(0x00)
#define BMP3_MODE_FORCED_VAL                        UINT8_C(0x01)
#define BMP3_MODE_NORMAL_VAL                        UINT8_C(0x03)

#define BMP3_OP_MODE_MSK                        UINT8_C(0x30)
#define BMP3_OP_MODE_POS                        UINT8_C(0x04)

#define BMP3_MODE_SLEEP                         (BMP3_MODE_SLEEP_VAL << BMP3_OP_MODE_POS)
#define BMP3_MODE_FORCED                        (BMP3_MODE_FORCED_VAL << BMP3_OP_MODE_POS)
#define BMP3_MODE_NORMAL                        (BMP3_MODE_NORMAL_VAL << BMP3_OP_MODE_POS)


static void timer_hit(void *opaque)
{
    BMP3State *s = BMP3(opaque);

    z_model_pressure p_press;
    z_model__compute_pressure(s->p_model, 0, &p_press);

    if (s->regs[R_BMP3_REG_PWR_CTRL] & BMP3_MODE_NORMAL) {

        s->regs[R_BMP3_REG_SENS_STATUS] |= BMP3_DRDY_PRESS;
        s->regs[R_BMP3_REG_SENS_STATUS] |= BMP3_DRDY_TEMP;
    }

    if (s->uses_int1 &&
        (s->regs[R_BMP3_REG_SENS_STATUS] & BMP3_DRDY_PRESS)) {

        qemu_set_irq(s->int1[0], s->int_level);
    } else {
        qemu_set_irq(s->int1[0], !s->int_level);
    }

}

static void _data_read_clear(BMP3State *s) {

    s->regs[R_BMP3_REG_SENS_STATUS] &= ~(BMP3_DRDY_PRESS);
    s->regs[R_BMP3_REG_SENS_STATUS] &= ~(BMP3_DRDY_TEMP);

    qemu_set_irq(s->int1[0], !s->int_level);

    s->pending_clear = false;
}

static void bmp3_reset(BMP3State *s);

static void bmp3_write(BMP3State *s, uint8_t data)
{

    s->regs[s->pointer+s->len-1u] = data; // fill data always

    if (s->len == 1) { // handle registers

        switch (s->pointer) {

            case A_BMP3_REG_CMD:
                if (data & BMP3_SOFT_RESET) {
                    bmp3_reset(s);
                }
                break;

            case A_BMP3_REG_PWR_CTRL:
                if (data & BMP3_MODE_NORMAL) {
                    info_report(TYPE_BMP3": Switched to NORMAL mode 0x%02X", data);
                } else {
                    info_report(TYPE_BMP3": Switched to SLEEP mode 0x%02X", data);
                }
                break;

            case A_BMP3_REG_INT_CTRL:
                // info_report(TYPE_BMP3": REG_INT_CTRL 0x%02X", data);
                if (data & BMP3_INT_LEVEL) {
                    s->int_level = BMP3_INT_PIN_ACTIVE_HIGH;
                } else {
                    s->int_level = BMP3_INT_PIN_ACTIVE_LOW;
                }
                qemu_set_irq(s->int1[0], !s->int_level);
                if (data & BMP3_INT_DRDY_EN) {
                    s->uses_int1 = true;
                    info_report(TYPE_BMP3": INT enabled !");
                    ptimer_transaction_begin(s->ptimer);
                    ptimer_stop(s->ptimer);
                    ptimer_set_freq(s->ptimer, 2);
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

static int bmp3_tx(I2CSlave *i2c, uint8_t data)
{
    BMP3State *s = BMP3(i2c);

    // info_report(TYPE_BMP3":tx 0x%02X (pos=%u)", data, s->len);

    if (s->len == 0) {
        /* first byte is the register pointer for a read or write
         * operation */
        s->pointer = data;
    } else {
        bmp3_write(s, data);
    }
    s->len++;

    return 0;
}

static void bmp3_reset(BMP3State *s)
{
    s->pointer = 0;

    s->regs[R_BMP3_REG_PWR_CTRL] = BMP3_MODE_SLEEP;

    if (s->ptimer) {
        ptimer_transaction_begin(s->ptimer);
        ptimer_stop(s->ptimer);
        ptimer_transaction_commit(s->ptimer);
    }

    s->pending_clear = false;

    info_report(TYPE_BMP3": Device reset");

    s->regs[R_BMP3_REG_CHIP_ID] = BMP390_CHIP_ID;

    s->regs[R_BMP3_REG_SENS_STATUS] = BMP3_CMD_RDY;
}


static void bmp3_read_events_process(BMP3State *s)
{

    // info_report(TYPE_BMP3": Device read reg= %02X", s->pointer);

    switch (s->pointer) {

        case A_BMP3_REG_DATA:
            s->pending_clear = true;
            if (s->uses_int1) {
                qemu_set_irq(s->int1[0], s->int_level);
            } else {
                qemu_set_irq(s->int1[0], !s->int_level);
            }
            break;

        default:
            break;
    }
}

static uint8_t bmp3_rx(I2CSlave *i2c)
{
    BMP3State *s = BMP3(i2c);

    // info_report(TYPE_BMP3":rx 0x%02X (pos=%u)", ret, s->len);

    if (s->pointer + s->len < sizeof(s->regs)) {
        uint8_t ret = s->regs[s->pointer + s->len];
        bmp3_read_events_process(s);
        s->len++;
        return ret;
    } else {
        return 0xff;
    }
}

static int bmp3_event(I2CSlave *i2c, enum i2c_event event)
{
    BMP3State *s = BMP3(i2c);

    s->len = 0;

    if (event == I2C_START_SEND) {
//        warn_report("-------------------");
//        warn_report("bmp3 I2C_START_SEND");
        s->pointer = 0;
    } else if (event == I2C_START_RECV) {
    } else if (event == I2C_FINISH) {
        if (s->pending_clear) {
            _data_read_clear(s);
        }
    }

    return 0;
}

/* Units are XXX
 */
static void bmp3_set_pressure(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint32_t press;
    BMP3State *s = BMP3(obj);

    if (!visit_type_uint32(v, name, &press, errp)) {
        return;
    }

    // 21 bit - 0.085pa
//    s->data[0] = 0;

    /* Temporary variables to store the sensor data */
    s->regs[R_BMP3_REG_DATA+0] = (press & 0xFF);
    s->regs[R_BMP3_REG_DATA+1] = ((press >> 8u) & 0xFF);
    s->regs[R_BMP3_REG_DATA+2] = ((press >> 16u) & 0xFF);
}

static void bmp3_get_pressure(Object *obj, Visitor *v, const char *name,
                                   void *opaque, Error **errp)
{
    uint32_t value;
    BMP3State *s = BMP3(obj);

    /* Temporary variables to store the sensor data */
    uint32_t data_xlsb;
    uint32_t data_lsb;
    uint32_t data_msb;

    /* Store the parsed register values for pressure data */
    data_xlsb = (uint32_t)s->regs[R_BMP3_REG_DATA+0];
    data_lsb = (uint32_t)s->regs[R_BMP3_REG_DATA+1] << 8;
    data_msb = (uint32_t)s->regs[R_BMP3_REG_DATA+2] << 16;

    value = data_msb | data_lsb | data_xlsb;

    visit_type_uint32(v, name, &value, errp);
}

static Property _properties[] = {
        DEFINE_PROP_LINK("model", BMP3State, p_model,
                         TYPE_ZMODEL, z_model_state *),
        DEFINE_PROP_END_OF_LIST(),
};

static const VMStateDescription vmstate_bmp3 = {
        .name = TYPE_BMP3,
        .version_id = 0,
        .minimum_version_id = 0,
        .fields = (VMStateField[]) {
//                VMSTATE_UINT8(status, BMP3State),
                VMSTATE_END_OF_LIST()
        }
};

static void bmp3_instance_init(Object *obj)
{
    BMP3State *s = BMP3(obj);

    bmp3_reset(s);
}

static void bmp3_realize(DeviceState *dev, Error **errp)
{
    BMP3State *s = BMP3(dev);

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_DEFAULT);

    qdev_init_gpio_out_named(DEVICE(s), s->int1, "DRDY", 1);

    vmstate_register(NULL, 0, &vmstate_bmp3, s);
}

static void bmp3_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *k = I2C_SLAVE_CLASS(klass);
    BMP3Class *sc = BMP3_CLASS(klass);

    dc->realize = bmp3_realize;
    k->event = bmp3_event;
    k->recv = bmp3_rx;
    k->send = bmp3_tx;
    dc->vmsd = &vmstate_bmp3;
    sc->dev = (DeviceInfo *) data;

    device_class_set_props(dc, _properties);

    object_class_property_add(klass, "pressure", "uint32",
                              bmp3_get_pressure,
                              bmp3_set_pressure, NULL, NULL);

}

static const TypeInfo bmp3_info = {
    .name          = TYPE_BMP3,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(BMP3State),
    .class_size    = sizeof(BMP3Class),
    .abstract      = true,
};

static void bmp3_register_types(void)
{
    int i;

    type_register_static(&bmp3_info);
    for (i = 0; i < ARRAY_SIZE(devices); ++i) {
        TypeInfo ti = {
            .name       = devices[i].name,
            .parent     = TYPE_BMP3,
            .class_init = bmp3_class_init,
            .instance_init = bmp3_instance_init,
            .class_data = (void *) &devices[i],
        };
        type_register(&ti);
    }
}

type_init(bmp3_register_types)
