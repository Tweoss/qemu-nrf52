//
// Created by vgol on 07/01/2024.
//

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "trace.h"
#include "hw/dma/nrf5x_radio.h"


#define _TYPE_NAME TYPE_NRF_RADIO

#define MODULE_OBJ_NAME NRF5RADIOState
#include "hw/dma/nrf5x_ppi.h"

enum {
    eRadioMode_disabled = 0u,
    eRadioMode_RxRu,
    eRadioMode_RxIdle,
    eRadioMode_Rx,
    eRadioMode_RxDisable,
    eRadioMode_TxRu = 9u,
    eRadioMode_TxIdle,
    eRadioMode_Tx,
    eRadioMode_TxDisable,
} eRadioMode;

static void _update_irq(NRF5RADIOState *s) {

    bool flag = false;

    flag |= s->regs[R_RADIO_EVENT_READY] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_READY_MASK);
    flag |= s->regs[R_RADIO_EVENT_ADDRESS] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_ADDRESS_MASK);
    flag |= s->regs[R_RADIO_EVENT_PAYLOAD] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_PAYLOAD_MASK);
    flag |= s->regs[R_RADIO_EVENT_END] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_END_MASK);
    flag |= s->regs[R_RADIO_EVENT_DISABLED] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_DISABLED_MASK);
    flag |= s->regs[R_RADIO_EVENT_DEVMATCH] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_DEVMATCH_MASK);
    flag |= s->regs[R_RADIO_EVENT_DEVMISS] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_DEVMISS_MASK);
    flag |= s->regs[R_RADIO_EVENT_RSSIEND] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_RSSIEND_MASK);
    flag |= s->regs[R_RADIO_EVENT_BCMATCH] &&
            (s->regs[R_RADIO_INTEN] & R_RADIO_INTEN_BCMATCH_MASK);

//    if (flag) {
//        info_report(_TYPE_NAME": update_irq mask=%u", s->regs[R_RTC_INTEN]);
//    }

    qemu_set_irq(s->irq, flag);
}

static void _shorts_process(NRF5RADIOState *s, uint32_t r_event);

static void trigger_event_r(NRF5RADIOState *s, uint32_t r_event) {

    info_report(_TYPE_NAME": event 0x%04X", r_event << 2u);

    s->regs[r_event] = 1;

    // check shorts
    _shorts_process(s, r_event);

    // check PPI
    PPI_EVENT_R(s, r_event);

    _update_irq(s);
}

static void tick_rearm(void *opaque, int64_t expire_time_us)
{
    NRF5RADIOState *s = NRF_RADIO(opaque);

    int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);
    timer_mod(&s->per_int, now + expire_time_us);
}

static void timer_expire(void *opaque)
{
    NRF5RADIOState *s = NRF_RADIO(opaque);

    switch (s->regs[R_RADIO_STATE]) {

        case eRadioMode_TxRu:
            s->regs[R_RADIO_STATE] = eRadioMode_TxIdle;
            trigger_event_r(s, R_RADIO_EVENT_READY);
            break;

        case eRadioMode_RxRu:
            s->regs[R_RADIO_STATE] = eRadioMode_RxIdle;
            trigger_event_r(s, R_RADIO_EVENT_READY);
            break;

        case eRadioMode_TxIdle:
            s->regs[R_RADIO_STATE] = eRadioMode_Tx;
            // start RTX
            tick_rearm(s, 100);
            break;

        case eRadioMode_RxIdle:
            s->regs[R_RADIO_STATE] = eRadioMode_Rx;
            // start RTX
            tick_rearm(s, 100);
            break;

        case eRadioMode_Tx:
            // start RTX
            trigger_event_r(s, R_RADIO_EVENT_ADDRESS);
            trigger_event_r(s, R_RADIO_EVENT_PAYLOAD);
            s->regs[R_RADIO_STATE] = eRadioMode_TxIdle;
            trigger_event_r(s, R_RADIO_EVENT_END);
            break;

        case eRadioMode_Rx:
            // start RTX
            trigger_event_r(s, R_RADIO_EVENT_ADDRESS);
            trigger_event_r(s, R_RADIO_EVENT_PAYLOAD);
            s->regs[R_RADIO_STATE] = eRadioMode_RxIdle;
            trigger_event_r(s, R_RADIO_EVENT_END);
            break;

        case eRadioMode_TxDisable:
        case eRadioMode_RxDisable:
            // start RTX
            s->regs[R_RADIO_STATE] = eRadioMode_disabled;
            trigger_event_r(s, R_RADIO_EVENT_DISABLED);
            break;

        default:
            break;
    }

}

static void _nrf_write(void *opaque,
                       hwaddr addr,
                       uint64_t value,
                       unsigned size) {

    NRF5RADIOState *s = NRF_RADIO(opaque);

    info_report(_TYPE_NAME": _nrf_write %03llX <- %llX", addr, value);

    switch (addr) {

        case A_RADIO_TASKS_TXEN:
            if (value) {
                s->regs[R_RADIO_STATE] = eRadioMode_TxRu;
                // start ramp up
                tick_rearm(s, 10);
            }
            break;

        case A_RADIO_TASKS_RXEN:
            if (value) {
                s->regs[R_RADIO_STATE] = eRadioMode_RxRu;
                // start ramp up
                tick_rearm(s, 10);
            }
            break;

        case A_RADIO_TASKS_START:
            if (value) {
                tick_rearm(s, 10);
            }
            break;

        case A_RADIO_TASKS_DISABLE:
            if (value) {
                if (s->regs[R_RADIO_STATE] == eRadioMode_TxIdle) {
                    s->regs[R_RADIO_STATE] = eRadioMode_TxDisable;
                }
                if (s->regs[R_RADIO_STATE] == eRadioMode_RxIdle) {
                    s->regs[R_RADIO_STATE] = eRadioMode_RxDisable;
                }
                tick_rearm(s, 10);
            }
            break;

        case A_RADIO_INTEN:
            s->regs[R_RADIO_INTEN] = value;
            break;
        case A_RADIO_INTENSET:
            s->regs[R_RADIO_INTEN] |= value;
            break;
        case A_RADIO_INTENCLR:
            s->regs[R_RADIO_INTEN] &= ~value;
            break;

        default:
            s->regs[addr >> 2] = value;
            break;
    }

    _update_irq(s);
}

static uint64_t _nrf_read(void *opaque,
                          hwaddr addr,
                          unsigned size) {

    uint64_t r;
    NRF5RADIOState *s = NRF_RADIO(opaque);

    info_report(_TYPE_NAME": _nrf_read %08llX", addr);

    switch (addr) {

        case A_RADIO_INTEN:
        case A_RADIO_INTENSET:
        case A_RADIO_INTENCLR:
            r = s->regs[R_RADIO_INTEN];
            break;

        default:
            r = s->regs[addr >> 2];
            break;
    }

    return r;

}

static void _shorts_process(NRF5RADIOState *s, uint32_t r_event) {

    switch (r_event) {

        case R_RADIO_EVENT_READY:
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_READY_START_MASK) {
                _nrf_write(s, A_RADIO_TASKS_START, 1, 0);
            }
            break;

        case R_RADIO_EVENT_ADDRESS:
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_ADDRESS_RSSISTART_MASK) {
                _nrf_write(s, A_RADIO_TASKS_RSSISTART, 1, 0);
            }
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_ADDRESS_BCSTART_MASK) {
                _nrf_write(s, A_RADIO_TASKS_BCSTART, 1, 0);
            }
            break;

        case R_RADIO_EVENT_PAYLOAD:
            break;

        case R_RADIO_EVENT_END:
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_END_DISABLE_MASK) {
                _nrf_write(s, A_RADIO_TASKS_DISABLE, 1, 0);
            }
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_END_START_MASK) {
                _nrf_write(s, A_RADIO_TASKS_START, 1, 0);
            }
            break;

        case R_RADIO_EVENT_DISABLED:
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_DISABLED_TXEN_MASK) {
                _nrf_write(s, A_RADIO_TASKS_TXEN, 1, 0);
            }
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_DISABLED_RXEN_MASK) {
                _nrf_write(s, A_RADIO_TASKS_RXEN, 1, 0);
            }
            if (s->regs[R_RADIO_SHORTS] & R_RADIO_SHORTS_DISABLED_RSSISTOP_MASK) {
                _nrf_write(s, A_RADIO_TASKS_RSSISTOP, 1, 0);
            }
            break;

        default:
            break;

    }

}

static void nrf5x_radio_reset(DeviceState *dev)
{
    NRF5RADIOState *s = NRF_RADIO(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->regs[R_RADIO_FREQUENCY] = 1 << 1u;
    s->regs[R_RADIO_CRCPOLY] = 1u;
    s->regs[R_RADIO_DATAWHITEIV] = 1 << 6u;
    s->regs[R_RADIO_POWER] = 1u;

}

static const MemoryRegionOps radio_ops = {
        .read =  _nrf_read,
        .write = _nrf_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf5x_radio_realize(DeviceState *dev, Error **errp)
{
    NRF5RADIOState *s = NRF_RADIO(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &radio_ops, s,
                          TYPE_NRF_RADIO, NRF_RADIO_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    timer_init_us(&s->per_int, QEMU_CLOCK_VIRTUAL, timer_expire, s);

    if (!s->downstream) {
        error_report("!! 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");

}

static void nrf5x_radio_init(Object *obj)
{
//    NRF5RADIOState *s = NRF_RADIO(obj);
//    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

}

static int nrf5x_radio_post_load(void *opaque, int version_id)
{
    NRF5RADIOState *s = NRF_RADIO(opaque);

    // empty
    (void)s;

    return 0;
}

static const VMStateDescription nrf5x_radio_vmstate = {
        .name = TYPE_NRF_RADIO,
        .post_load = nrf5x_radio_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_UINT32_ARRAY(regs, NRF5RADIOState, NRF_RADIO_PER_SIZE),
                VMSTATE_END_OF_LIST()
        }
};

static Property nrf5x_radio_properties[] = {
        DEFINE_PROP_LINK("downstream", NRF5RADIOState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf5x_radio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf5x_radio_reset;
    dc->realize = nrf5x_radio_realize;
    device_class_set_props(dc, nrf5x_radio_properties);
    dc->vmsd = &nrf5x_radio_vmstate;
}

static const TypeInfo nrf5x_radio_info = {
        .name = TYPE_NRF_RADIO,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF5RADIOState),
        .instance_init = nrf5x_radio_init,
        .class_init = nrf5x_radio_class_init
};

static void nrf5x_radio_register_types(void)
{
    type_register_static(&nrf5x_radio_info);
}

type_init(nrf5x_radio_register_types)

