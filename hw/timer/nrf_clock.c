//
// Created by vgol on 13/03/2023.
//

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/timer/nrf_clock.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

#define _TYPE_NAME TYPE_NRF_CLOCK

#define MODULE_OBJ_NAME NRF52CLOCKState
#include "hw/dma/nrf5x_ppi.h"

static void _clocks_irq_compute(NRF52CLOCKState *s) {

    bool irq = false;

    irq |= (s->regs[R_CLOCK_EVENTS_HFCLKSTARTED]   &&
            (s->regs[R_CLOCK_INTEN] & R_CLOCK_INTEN_HFCLKSTARTED_MASK));
    irq |= (s->regs[R_CLOCK_EVENTS_LFCLKSTARTED]   &&
            (s->regs[R_CLOCK_INTEN] & R_CLOCK_INTEN_LFCLKSTARTED_MASK));

//    if (irq)
//        info_report(_TYPE_NAME": irq %d STOPPED %u", irq, s->regs[R_EDMA_EVENT_STOPPED]);

    qemu_set_irq(s->irq, irq);
}

static void timer_rearm(void *opaque, bool use_ctiv)
{
    NRF52CLOCKState *s = NRF_CLOCK(opaque);
    uint64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    uint64_t timer_per_us = 100; // 100 us

    if (use_ctiv) {
        timer_per_us = s->regs[R_CLOCK_CTIV] * 250000; // Calibration timer interval in multiple of 0.25 seconds
    }

    timer_mod(&s->cal_timer, now + timer_per_us);
}

static void cal_expire(void *opaque)
{
    NRF52CLOCKState *s = NRF_CLOCK(opaque);

//    info_report(_TYPE_NAME": tick");

    if (s->regs[R_CLOCK_CAL]) {
        s->regs[R_CLOCK_CAL] = 0;
        s->regs[R_CLOCK_EVENTS_DONE] = 1;
        PPI_EVENT_R(s, R_CLOCK_EVENTS_DONE);
    } else if (R_CLOCK_CTSTART) {
        s->regs[R_CLOCK_CTSTART] = 0;
        s->regs[R_CLOCK_EVENTS_CTTO] = 1;
        PPI_EVENT_R(s, R_CLOCK_EVENTS_CTTO);
    }

    _clocks_irq_compute(s);
}

static uint64_t nrf52_clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    NRF52CLOCKState *s = NRF_CLOCK(opaque);
    uint64_t r = 0;

    switch (addr) {

        case A_CLOCK_INTEN:
        case A_CLOCK_INTENCLR:
        case A_CLOCK_INTENSET:
            r = s->regs[R_CLOCK_INTEN];
            break;

        default:
            r = s->regs[addr / 4];
            break;
    }

//    info_report(_TYPE_NAME": _read %08llX = %llu", addr, r);

    _clocks_irq_compute(s);

    return r;
}

static void nrf52_clock_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    NRF52CLOCKState *s = NRF_CLOCK(opaque);

//    info_report(_TYPE_NAME": _write %08llX %llu", addr, value);

    switch (addr) {

        case A_CLOCK_TASKS_LFCLKSTART:
            s->regs[R_CLOCK_LFCLKRUN] = 1;
            s->regs[R_CLOCK_EVENTS_LFCLKSTARTED] = 1;
            s->regs[R_CLOCK_LFCLKSRCPY] = s->regs[R_CLOCK_LFCLKSRC];
            PPI_EVENT_R(s, R_CLOCK_EVENTS_LFCLKSTARTED);
            s->regs[R_CLOCK_LFCLKSTAT] |= (1 << R_CLOCK_LFCLKSTAT_STATE_SHIFT) | (1 << R_CLOCK_LFCLKSTAT_SRC_SHIFT); //xtal
            info_report(_TYPE_NAME": A_CLOCK_TASKS_LFCLKSTART");
            break;

        case A_CLOCK_TASKS_HFCLKSTART:
            s->regs[R_CLOCK_HFCLKRUN] = 1;
            s->regs[R_CLOCK_EVENTS_HFCLKSTARTED] = 1;
            PPI_EVENT_R(s, R_CLOCK_EVENTS_HFCLKSTARTED);
            s->regs[R_CLOCK_HFCLKSTAT] |= (1 << R_CLOCK_HFCLKSTAT_STATE_SHIFT) | (1 << R_CLOCK_HFCLKSTAT_SRC_SHIFT); // xtal
            info_report(_TYPE_NAME": A_CLOCK_TASKS_HFCLKSTART");
            break;

        case A_CLOCK_CAL:
        case A_CLOCK_CTSTART:
            s->regs[addr / 4] = value;
            timer_rearm(s, addr == A_CLOCK_CTSTART);
            break;

        case A_CLOCK_INTEN:
            s->regs[R_CLOCK_INTEN] = value;
            break;
        case A_CLOCK_INTENSET:
            s->regs[R_CLOCK_INTEN] |= value;
            break;
        case A_CLOCK_INTENCLR:
            s->regs[R_CLOCK_INTEN] &= ~value;
            break;

        case A_PWR_SYSTEMOFF:
            warn_report("nrf52.pwr: Going to SYSTEMOFF"); // TODO move to separate file
            exit(0);
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    _clocks_irq_compute(s);

}

static const MemoryRegionOps clock_ops = {
        .read =  nrf52_clock_read,
        .write = nrf52_clock_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl.min_access_size = 4,
        .impl.max_access_size = 4,
};

static void nrf52_clock_reset(DeviceState *dev)
{
//    NRF52CLOCKState *s = NRF_CLOCK(dev);
}

static void nrf_clock_realize(DeviceState *dev, Error **errp)
{
    NRF52CLOCKState *s = NRF_CLOCK(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &clock_ops, s,
                          TYPE_NRF_CLOCK, NRF_CLOCK_PER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);

    timer_init_us(&s->cal_timer, QEMU_CLOCK_VIRTUAL, cal_expire, s);

    // qdev_init_gpio_in_named(DEVICE(s), nrf52_clock_set, "clock", NRF52_CLOCK_PINS);

    if (s->downstream) {
        address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");
    }
}

static void nrf5_clock_init(Object *obj)
{
//    NRF52CLOCKState *s = NRF_CLOCK(obj);
}

static Property nrf_clock_properties[] = {
        DEFINE_PROP_LINK("downstream", NRF52CLOCKState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion*),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf52_clock_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf52_clock_reset;
    dc->realize = nrf_clock_realize;
    device_class_set_props(dc, nrf_clock_properties);
}

static const TypeInfo nrf52_clock_info = {
        .name = TYPE_NRF_CLOCK,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF52CLOCKState),
        .instance_init = nrf5_clock_init,
        .class_init = nrf52_clock_class_init
};

static void nrf52_clock_register_types(void)
{
    type_register_static(&nrf52_clock_info);
}

type_init(nrf52_clock_register_types)

