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

static uint32_t ns_to_ticks(NRF5RtcState *s, int64_t ns)
{
    uint32_t freq = TIMER_CLK_FREQ / (s->regs[R_RTC_PRESCALER] + 1);

    return muldiv64(ns, freq, NANOSECONDS_PER_SECOND);
}

static int64_t ticks_to_ns(NRF5RtcState *s, uint32_t ticks)
{
    uint32_t freq = TIMER_CLK_FREQ / (s->regs[R_RTC_PRESCALER] + 1);

    return muldiv64(ticks, NANOSECONDS_PER_SECOND, freq);
}

/* Returns number of ticks since last call */
static uint32_t update_counter(NRF5RtcState *s, int64_t now)
{
    uint32_t ticks = ns_to_ticks(s, now - s->update_counter_ns);

    s->counter = (s->counter + ticks) % BIT(TIMER_BITWIDTH);
    s->update_counter_ns = now;
    return ticks;
}

/* Assumes s->counter is up-to-date */
static void rearm_timer(NRF5RtcState *s, int64_t now)
{
    int64_t min_ns = INT64_MAX;
    size_t i;

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        int64_t delta_ns;

        if (s->regs[R_RTC_EVENT_CMP0+i]) {
            continue; /* already expired, ignore it for now */
        }

        if (s->regs[R_RTC_CC0+i] <= s->counter) {
            delta_ns = ticks_to_ns(s, BIT(TIMER_BITWIDTH) -
                                      s->counter + s->regs[R_RTC_CC0+i]);
        } else {
            delta_ns = ticks_to_ns(s, s->regs[R_RTC_CC0+i] - s->counter);
        }

        if (delta_ns < min_ns) {
            min_ns = delta_ns;
        }
    }

    if (min_ns != INT64_MAX) {
        timer_mod_ns(&s->timer, now + min_ns);
    }
}

static void update_irq(NRF5RtcState *s)
{
    bool flag = false;

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

static void timer_expire(void *opaque)
{
    NRF5RtcState *s = NRF_RTC(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t cc_remaining[NRF5_RTC_REG_COUNT];
    bool should_stop = false;
    uint32_t ticks;
    size_t i;

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        if (s->regs[R_RTC_CC0+i] > s->counter) {
            cc_remaining[i] = s->regs[R_RTC_CC0+i] - s->counter;
        } else {
            cc_remaining[i] = BIT(TIMER_BITWIDTH) -
                              s->counter + s->regs[R_RTC_CC0+i];
        }
    }

    ticks = update_counter(s, now);

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        if (cc_remaining[i] <= ticks) {
            s->regs[R_RTC_EVENT_CMP0+i] = 1;

//            if (true) {
//                s->timer_start_ns = now;
//                s->update_counter_ns = s->timer_start_ns;
//                s->counter = 0;
//            }
//            should_stop |= s->shorts & BIT(i + 8);
        }
    }

    update_irq(s);

    if (should_stop) {
        s->running = false;
        timer_del(&s->timer);
    } else {
        rearm_timer(s, now);
    }
}

static void counter_compare(NRF5RtcState *s)
{
    uint32_t counter = s->counter;
    size_t i;

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        if (counter == s->regs[R_RTC_CC0+i]) {
            s->regs[R_RTC_EVENT_CMP0+i] = 1;

            s->counter = 0; // ?
        }
    }
}

static uint64_t _nrf_read(void *opaque,
                      hwaddr addr,
                      unsigned size) {

    uint64_t r;
    NRF5RtcState *s = NRF_RTC(opaque);

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

        case A_RTC_CTR:
        {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            (void)update_counter(s, now);
            r = s->counter;
        }   break;

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

    //info_report("nrf.rtc: _nrf_write %08lX %lu", addr, value);

    switch (addr) { // EDMA

        case A_RTC_TASKS_START:
            if (value) {
                s->running = true;
                s->timer_start_ns = now - ticks_to_ns(s, s->counter);
                s->update_counter_ns = s->timer_start_ns;
                rearm_timer(s, now);
            }
            break;
        case A_RTC_TASKS_STOP:
            if (value) {
                s->running = false;
                timer_del(&s->timer);
            }
            break;
        case A_RTC_TASKS_CLEAR:
            if (value) {
                s->timer_start_ns = now;
                s->update_counter_ns = s->timer_start_ns;
                s->counter = 0;
                if (s->running) {
                    rearm_timer(s, now);
                }
            }
            break;

        case A_RTC_CTR:
            if (value) {
                s->counter = value & 0xFFF;
                counter_compare(s);
            }
            break;

        case A_RTC_EVENT_CMP0:
        case A_RTC_EVENT_CMP1:
        case A_RTC_EVENT_CMP2:
        case A_RTC_EVENT_CMP3:
            if (!value) {
                if (s->running) {
                    timer_expire(s); /* update counter and all state */
                }
            }
            s->regs[addr / 4] = value;
            break;

        case A_RTC_CC0:
        case A_RTC_CC1:
        case A_RTC_CC2:
        case A_RTC_CC3:
            if (s->running) {
                timer_expire(s); /* update counter */
            }
            s->regs[addr / 4] = value;
            if (s->running) {
                rearm_timer(s, now);
            }
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

    timer_del(&s->timer);
    s->timer_start_ns = 0x00;
    s->update_counter_ns = 0x00;
    s->counter = 0x00;
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

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, timer_expire, s);
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
        timer_expire(s);
    }

    return 0;
}

static const VMStateDescription nrf_rtcvmstate = {
        .name = TYPE_NRF_RTC,
        .post_load = nrf_rtcpost_load,
        .fields = (VMStateField[]) {
                VMSTATE_TIMER(timer, NRF5RtcState),
                VMSTATE_INT64(timer_start_ns, NRF5RtcState),
                VMSTATE_INT64(update_counter_ns, NRF5RtcState),
                VMSTATE_UINT32(counter, NRF5RtcState),
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

