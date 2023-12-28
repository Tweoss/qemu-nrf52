/*
 *
 *
 *
    NRF_RTC1->EVTENSET = RTC_EVTEN_COMPARE0_Msk;
    NRF_RTC1->INTENSET = RTC_INTENSET_COMPARE0_Msk;

    NVIC_ClearPendingIRQ(RTC1_IRQn);
    NVIC_EnableIRQ(RTC1_IRQn);

    NRF_RTC1->TASKS_START = 1;
    nrf_delay_us(MAX_RTC_TASKS_DELAY);

    m_rtc1_running = true;

 */

#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "trace.h"
#include "hw/timer/nrf_rtc.h"

#define TIMER_CLK_FREQ 32768uL

#define TIMER_BITWIDTH 24


static void update_irq(NRF5RtcState *s)
{
    bool flag = false;

    flag |= s->regs[R_RTC_EVENT_TICK] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_TICK_MASK);
    flag |= s->regs[R_RTC_EVENT_OVRFLW] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_OVRFLW_MASK);
    flag |= s->regs[R_RTC_EVENT_CMP0] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_CC0_MASK);
    flag |= s->regs[R_RTC_EVENT_CMP1] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_CC1_MASK);
    flag |= s->regs[R_RTC_EVENT_CMP2] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_CC2_MASK);
    flag |= s->regs[R_RTC_EVENT_CMP3] &&
            (s->regs[R_RTC_INTEN] & R_RTC_INTEN_CC3_MASK);

//    if (flag) {
//        info_report("nrf.rtc: update_irq mask=%u", s->regs[R_RTC_INTEN]);
//    }

    qemu_set_irq(s->irq, flag);
}

static void counter_compare(NRF5RtcState *s)
{
    size_t i;

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {

        if (s->regs[R_RTC_EVENT_CMP0+i]) {
            continue; /* already expired, ignore it for now */
        }

        if (s->regs[R_RTC_CC0+i] == 0) {
            continue; /* not enabled */
        }

        // next compare to expire
        if (s->regs[R_RTC_CC0+i] == s->regs[R_RTC_CTR]) {
            s->regs[R_RTC_EVENT_CMP0+i] = 1;
        }
    }
}

static void tick_rearm(void *opaque, int64_t now)
{
    NRF5RtcState *s = NRF_RTC(opaque);

    s->regs[R_RTC_PRESCALER] = s->regs[R_RTC_PRESCALER] & 0xFFFu; // max value

    uint64_t timer_per_ns = muldiv64((s->regs[R_RTC_PRESCALER] + 1), NANOSECONDS_PER_SECOND, TIMER_CLK_FREQ);

    timer_mod_ns(&s->tick, now + timer_per_ns);

    if (!s->running) {
        s->running = true;
        info_report("nrf.rtc: timer_per_ns %lu", (long unsigned)timer_per_ns);
    }
}

static void tick_expire(void *opaque)
{
    NRF5RtcState *s = NRF_RTC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

//    info_report("nrf.rtc: tick");

    if (++s->regs[R_RTC_CTR] >= BIT(TIMER_BITWIDTH)) {
        s->regs[R_RTC_EVENT_OVRFLW] = 1;
        s->regs[R_RTC_CTR] = 0;
    }

    s->regs[R_RTC_EVENT_TICK] = 1;

    counter_compare(s);

    update_irq(s);

    tick_rearm(s, now);
}

static uint64_t _nrf_read(void *opaque,
                      hwaddr addr,
                      unsigned size) {

    uint64_t r;
    NRF5RtcState *s = NRF_RTC(opaque);

//    info_report("nrf.rtc: read %08lX", (long unsigned)addr);

    switch (addr) {
        case A_RTC_EVTEN:
        case A_RTC_EVTENCLR:
        case A_RTC_EVTENSET:
            r = s->regs[R_RTC_EVTEN];
            break;

        case A_RTC_INTENCLR:
        case A_RTC_INTENSET:
            r = s->regs[R_RTC_INTEN];
            break;

        default:
            r = s->regs[addr / 4];
            break;
    }

    //info_report("nrf.rtc: _nrf_read %08lX = %lu", addr, r);

    return r;

}

static void _nrf_write(void *opaque,
                   hwaddr addr,
                   uint64_t value,
                   unsigned size) {

    NRF5RtcState *s = NRF_RTC(opaque);
    uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);

//    info_report("nrf.rtc: write %08lX %lu", (long unsigned)addr, (long unsigned)value);

    switch (addr) { // EDMA

        case A_RTC_TASKS_START:
            if (value) {
                tick_rearm(s, now);
            }
            break;
        case A_RTC_TASKS_STOP:
            if (value) {
                timer_del(&s->tick);
                s->running = false;
            }
            break;
        case A_RTC_TASKS_CLEAR:
            if (value) {
                s->regs[R_RTC_CTR] = 0;
                if (s->running) {
                    tick_rearm(s, now);
                }
            }
            break;
        case A_RTC_TRIGOVRFLW:
            s->regs[R_RTC_CTR] = 0xFFFFF0;
            break;

        case A_RTC_CTR:
            if (!s->running) {
                s->regs[R_RTC_CTR] = value & (BIT(TIMER_BITWIDTH) - 1);
                counter_compare(s);
            }
            break;

        case A_RTC_EVENT_CMP0:
        case A_RTC_EVENT_CMP1:
        case A_RTC_EVENT_CMP2:
        case A_RTC_EVENT_CMP3:
            if (!value) {
                if (s->running) {
                    counter_compare(s);
                }
            }
            s->regs[addr / 4] = value;
            break;

        case A_RTC_CC0:
        case A_RTC_CC1:
        case A_RTC_CC2:
        case A_RTC_CC3:
            s->regs[addr / 4] = value; // compare value
            break;

        case A_RTC_EVTEN:
            s->regs[R_RTC_EVTEN] = value;
            break;
        case A_RTC_EVTENSET:
            s->regs[R_RTC_EVTEN] |= value;
            break;
        case A_RTC_EVTENCLR:
            s->regs[R_RTC_EVTEN] &= ~value;
            break;

        case A_RTC_INTENSET:
            s->regs[R_RTC_INTEN] |= value;
            break;
        case A_RTC_INTENCLR:
            s->regs[R_RTC_INTEN] &= ~value;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    update_irq(s);
}

static void nrf_rtcreset(DeviceState *dev)
{
    NRF5RtcState *s = NRF_RTC(dev);

    timer_del(&s->tick);
    s->running = false;
}

static const MemoryRegionOps rtc_ops = {
        .read =  _nrf_read,
        .write = _nrf_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf_rtcrealize(DeviceState *dev, Error **errp)
{
    NRF5RtcState *s = NRF_RTC(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &rtc_ops, s,
                          TYPE_NRF_RTC, NRF_RTC_PER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    timer_init_ns(&s->tick, QEMU_CLOCK_VIRTUAL, tick_expire, s);
}

static void nrf_rtcinit(Object *obj)
{
//    NRF5RtcState *s = NRF_RTC(obj);
//    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

//    memory_region_init_io(&s->iomem, obj, &rtc_ops, s,
//                          TYPE_NRF_RTC, NRF_RTC_PER_SIZE);
//    sysbus_init_mmio(sbd, &s->iomem);
//    sysbus_init_irq(sbd, &s->irq);

//    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, timer_expire, s);
}

static int nrf_rtcpost_load(void *opaque, int version_id)
{
    NRF5RtcState *s = NRF_RTC(opaque);

    if (s->running) {
        tick_expire(s);
    }

    return 0;
}

static const VMStateDescription nrf_rtcvmstate = {
        .name = TYPE_NRF_RTC,
        .post_load = nrf_rtcpost_load,
        .fields = (VMStateField[]) {
                VMSTATE_TIMER(tick, NRF5RtcState),
                VMSTATE_BOOL(running, NRF5RtcState),
                VMSTATE_UINT32_ARRAY(regs, NRF5RtcState, NRF_RTC_PER_SIZE),
                VMSTATE_BOOL(enabled, NRF5RtcState),
                VMSTATE_END_OF_LIST()
        }
};

static Property nrf_rtc_properties[] = {
        DEFINE_PROP_UINT8("id", NRF5RtcState, id, 0),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf_rtcclass_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf_rtcreset;
    dc->realize = nrf_rtcrealize;
    dc->vmsd = &nrf_rtcvmstate;
    device_class_set_props(dc, nrf_rtc_properties);
}

static const TypeInfo nrf_rtcinfo = {
        .name = TYPE_NRF_RTC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF5RtcState),
        .instance_init = nrf_rtcinit,
        .class_init = nrf_rtcclass_init
};

static void nrf_rtcregister_types(void)
{
    type_register_static(&nrf_rtcinfo);
}

type_init(nrf_rtcregister_types)

