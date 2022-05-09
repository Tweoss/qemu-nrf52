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

#define NRF_RTC_PER_SIZE   0x600
#define NRF5_RTC_REG_COUNT 4

#define NRF_TIMER_TIMER 0
#define NRF_TIMER_COUNTER 1

REG32(RTC_TASKS_START, 0x000)
REG32(RTC_TASKS_STOP, 0x004)
REG32(RTCTASKS_CLEAR, 0x008)

REG32(RTC_EVENT_TICK, 0x100)
REG32(RTC_EVENT_OVRFLW, 0x104)

REG32(RTC_EVENT_CMP0, 0x140)
REG32(RTC_EVENT_CMP1, 0x144)
REG32(RTC_EVENT_CMP2, 0x148)
REG32(RTC_EVENT_CMP3, 0x14C)

REG32(RTC_INTENSET, 0x304)
REG32(RTC_INTENCLR, 0x308)

REG32(RTC_EVTEN, 0x340)
REG32(RTC_EVTENSET, 0x344)
REG32(RTC_EVTENCLR, 0x348)

REG32(RTC_CTR, 0x504)
REG32(RTC_PRESCALER, 0x508)

REG32(RTC_CC0, 0x540)
REG32(RTC_CC1, 0x544)
REG32(RTC_CC2, 0x548)
REG32(RTC_CC3, 0x54C)

struct NRF5RtcState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint8_t id;
    QEMUTimer timer;
    int64_t timer_start_ns;
    int64_t update_counter_ns;
    uint32_t counter;

    bool enabled;
    bool running;

    uint8_t events_compare[NRF5_RTC_REG_COUNT];
    uint32_t cc[NRF5_RTC_REG_COUNT];
    uint32_t shorts;
    uint32_t inten;
    uint32_t mode;
    uint32_t bitmode;
    uint32_t prescaler;

    uint32_t regs[NRF_RTC_PER_SIZE];
};

#define TYPE_NRF_RTC "nrf5_soc.rtc"
OBJECT_DECLARE_SIMPLE_TYPE(NRF5RtcState, NRF_RTC)


#define TIMER_CLK_FREQ 16000000UL

static uint32_t const bitwidths[] = {16, 8, 24, 32};

static uint32_t ns_to_ticks(NRF5RtcState *s, int64_t ns)
{
    uint32_t freq = TIMER_CLK_FREQ >> s->prescaler;

    return muldiv64(ns, freq, NANOSECONDS_PER_SECOND);
}

static int64_t ticks_to_ns(NRF5RtcState *s, uint32_t ticks)
{
    uint32_t freq = TIMER_CLK_FREQ >> s->prescaler;

    return muldiv64(ticks, NANOSECONDS_PER_SECOND, freq);
}

/* Returns number of ticks since last call */
static uint32_t update_counter(NRF5RtcState *s, int64_t now)
{
    uint32_t ticks = ns_to_ticks(s, now - s->update_counter_ns);

    s->counter = (s->counter + ticks) % BIT(bitwidths[s->bitmode]);
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

        if (s->events_compare[i]) {
            continue; /* already expired, ignore it for now */
        }

        if (s->cc[i] <= s->counter) {
            delta_ns = ticks_to_ns(s, BIT(bitwidths[s->bitmode]) -
                                      s->counter + s->cc[i]);
        } else {
            delta_ns = ticks_to_ns(s, s->cc[i] - s->counter);
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
    size_t i;

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        flag |= s->events_compare[i] && extract32(s->inten, 16 + i, 1);
    }
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
        if (s->cc[i] > s->counter) {
            cc_remaining[i] = s->cc[i] - s->counter;
        } else {
            cc_remaining[i] = BIT(bitwidths[s->bitmode]) -
                              s->counter + s->cc[i];
        }
    }

    ticks = update_counter(s, now);

    for (i = 0; i < NRF5_RTC_REG_COUNT; i++) {
        if (cc_remaining[i] <= ticks) {
            s->events_compare[i] = 1;

            if (s->shorts & BIT(i)) {
                s->timer_start_ns = now;
                s->update_counter_ns = s->timer_start_ns;
                s->counter = 0;
            }

            should_stop |= s->shorts & BIT(i + 8);
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
        if (counter == s->cc[i]) {
            s->events_compare[i] = 1;

            if (s->shorts & BIT(i)) {
                s->counter = 0;
            }
        }
    }
}

static uint64_t _read(void *opaque,
                      hwaddr addr,
                      unsigned size) {

    uint64_t r;
    NRF5RtcState *s = NRF_RTC(opaque);

    info_report("nrf.rtc: _read %08lX", addr);

    switch (addr) {
        case A_RTC_EVTEN:
        case A_RTC_EVTENCLR:
        case A_RTC_EVTENSET:
            r = s->regs[R_RTC_EVTEN];
            break;
        default:
            r = s->regs[addr / 4];
            break;
    }

    return r;

}

static void _write(void *opaque,
                   hwaddr addr,
                   uint64_t value,
                   unsigned size) {

    NRF5RtcState *s = NRF_RTC(opaque);

    info_report("nrf.rtc: _write %08lX %lu", addr, value);

    switch (addr) { // EDMA

        case A_RTC_EVTEN:
            s->regs[R_RTC_EVTEN] = value;
            break;
        case A_RTC_EVTENSET:
            s->regs[R_RTC_EVTEN] |= value;
            break;
        case A_RTC_EVTENCLR:
            s->regs[R_RTC_EVTEN] &= ~value;
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

    memset(s->events_compare, 0x00, sizeof(s->events_compare));
    memset(s->cc, 0x00, sizeof(s->cc));

    s->shorts = 0x00;
    s->inten = 0x00;
    s->mode = 0x00;
    s->bitmode = 0x00;
    s->prescaler = 0x00;
}

static const MemoryRegionOps rtc_ops = {
        .read =  _read,
        .write = _write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl.min_access_size = 4,
        .impl.max_access_size = 4,
};

static void nrf_rtcrealize(DeviceState *dev, Error **errp)
{
    NRF5RtcState *s = NRF_RTC(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &rtc_ops, s,
                          TYPE_NRF_RTC, NRF_RTC_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, timer_expire, s);
}

static void nrf_rtcinit(Object *obj)
{
//    NRF5RtcState *s = NRF_RTC(obj);
//    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

}

static int nrf_rtcpost_load(void *opaque, int version_id)
{
    NRF5RtcState *s = NRF_RTC(opaque);

    if (s->running && s->mode == NRF_TIMER_TIMER) {
        timer_expire(s);
    }

    return 0;
}

static const VMStateDescription nrf_rtcvmstate = {
        .name = TYPE_NRF_RTC,
        .post_load = nrf_rtcpost_load,
        .fields = (VMStateField[]) {
                VMSTATE_UINT32_ARRAY(regs, NRF5RtcState, 0x600),
                VMSTATE_BOOL(enabled, NRF5RtcState),
                VMSTATE_END_OF_LIST()
        }
};

static Property nrf_rtcproperties[] = {
    DEFINE_PROP_UINT8("id", NRF5RtcState, id, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void nrf_rtcclass_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf_rtcreset;
    dc->realize = nrf_rtcrealize;
    device_class_set_props(dc, nrf_rtcproperties);
    dc->vmsd = &nrf_rtcvmstate;
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

