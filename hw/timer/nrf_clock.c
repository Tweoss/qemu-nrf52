//
// Created by vgol on 13/03/2023.
//

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/timer/nrf_clock.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

static void _clocks_irq_compute(NRF52CLOCKState *s) {

    bool irq = false;

    irq |= (s->regs[R_CLOCK_EVENTS_HFCLKSTARTED]   &&
            (s->regs[R_CLOCK_INTEN] & R_CLOCK_INTEN_HFCLKSTARTED_MASK));
    irq |= (s->regs[R_CLOCK_EVENTS_LFCLKSTARTED]   &&
            (s->regs[R_CLOCK_INTEN] & R_CLOCK_INTEN_LFCLKSTARTED_MASK));

//    if (irq)
//        info_report("nrf52832.edma: irq %d STOPPED %u", irq, s->regs[R_EDMA_EVENT_STOPPED]);

    qemu_set_irq(s->irq, irq);

    s->regs[R_CLOCK_EVENTS_HFCLKSTARTED] = 0; // clear events
    s->regs[R_CLOCK_EVENTS_LFCLKSTARTED] = 0;

}

static uint64_t nrf52_clock_read(void *opaque, hwaddr addr, unsigned int size)
{
    NRF52CLOCKState *s = NRF52_CLOCK(opaque);
    uint64_t r = 0;

    switch (addr) {

        default:
            r = s->regs[addr / 4];
            break;
    }

    // info_report("nrf52.clock: _read %08lX = %lu", addr, r);

    _clocks_irq_compute(s);

    return r;
}

static void nrf52_clock_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    NRF52CLOCKState *s = NRF52_CLOCK(opaque);

    // info_report("nrf52.clock: _write %08lX %lu", addr, value);

    switch (addr) {

        case A_CLOCK_TASKS_LFCLKSTART:
            s->regs[R_CLOCK_LFCLKRUN] = 1;
            s->regs[R_CLOCK_EVENTS_LFCLKSTARTED] = 1;
            s->regs[R_CLOCK_LFCLKSTAT] |= (1 << R_CLOCK_LFCLKSTAT_STATE_SHIFT) | (1 << R_CLOCK_LFCLKSTAT_SRC_SHIFT); //xtal
            info_report("nrf52.clock: A_CLOCK_TASKS_LFCLKSTART");
            break;

        case A_CLOCK_TASKS_HFCLKSTART:
            s->regs[R_CLOCK_HFCLKRUN] = 1;
            s->regs[R_CLOCK_EVENTS_HFCLKSTARTED] = 1;
            s->regs[R_CLOCK_HFCLKSTAT] |= (1 << R_CLOCK_HFCLKSTAT_STATE_SHIFT) | (1 << R_CLOCK_HFCLKSTAT_SRC_SHIFT); // xtal
            info_report("nrf52.clock: A_CLOCK_TASKS_HFCLKSTART");
            break;

        case A_CLOCK_INTENSET:
            s->regs[R_CLOCK_INTEN] |= value;
            break;
        case A_CLOCK_INTENCLR:
            s->regs[R_CLOCK_INTEN] &= ~value;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    _clocks_irq_compute(s);

}

static const MemoryRegionOps gpio_ops = {
        .read =  nrf52_clock_read,
        .write = nrf52_clock_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl.min_access_size = 4,
        .impl.max_access_size = 4,
};

static void nrf52_clock_reset(DeviceState *dev)
{
//    NRF52CLOCKState *s = NRF52_CLOCK(dev);
}

static const VMStateDescription vmstate_nrf51_gpio = {
        .name = TYPE_NRF52_CLOCK,
        .version_id = 1,
        .minimum_version_id = 1,
        .fields = (VMStateField[]) {
                VMSTATE_END_OF_LIST()
        }
};

static void nrf52_clock_init(Object *obj)
{
    NRF52CLOCKState *s = NRF52_CLOCK(obj);

    memory_region_init_io(&s->mmio, obj, &gpio_ops, s,
                          TYPE_NRF52_CLOCK, NRF52832_CLOCK_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
    sysbus_init_irq(SYS_BUS_DEVICE(obj), &s->irq);

    // qdev_init_gpio_in_named(DEVICE(s), nrf52_clock_set, "clock", NRF52_CLOCK_PINS);
}

static void nrf52_clock_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_nrf51_gpio;
    dc->reset = nrf52_clock_reset;
    dc->desc = "nRF52 CLOCK";
}

static const TypeInfo nrf52_clock_info = {
        .name = TYPE_NRF52_CLOCK,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF52CLOCKState),
        .instance_init = nrf52_clock_init,
        .class_init = nrf52_clock_class_init
};

static void nrf52_clock_register_types(void)
{
    type_register_static(&nrf52_clock_info);
}

type_init(nrf52_clock_register_types)

