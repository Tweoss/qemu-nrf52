//
// Created by vgol on 13/03/2023.
//

#ifndef QEMU_NRF52_NRF_CLOCK_H
#define QEMU_NRF52_NRF_CLOCK_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/registerfields.h"

#define TYPE_NRF_CLOCK  "nrf5_soc.clock"
OBJECT_DECLARE_SIMPLE_TYPE(NRF52CLOCKState, NRF_CLOCK)


#define NRF_CLOCK_PER_SIZE 0x1000

REG32(CLOCK_TASKS_HFCLKSTART, 0x000)
REG32(CLOCK_TASKS_HFCLKSTOP, 0x004)

REG32(CLOCK_TASKS_LFCLKSTART, 0x008)
REG32(CLOCK_TASKS_LFCLKSTOP, 0x00C)

REG32(CLOCK_EVENTS_HFCLKSTARTED, 0x100)
REG32(CLOCK_EVENTS_LFCLKSTARTED, 0x104)
REG32(CLOCK_EVENTS_DONE, 0x10C)
REG32(CLOCK_EVENTS_CTTO, 0x110)

REG32(CLOCK_INTEN, 0x300) // fake register
    FIELD(CLOCK_INTEN, HFCLKSTARTED, 0, 1)
    FIELD(CLOCK_INTEN, LFCLKSTARTED, 1, 1)
    FIELD(CLOCK_INTEN, DONE, 3, 1)
    FIELD(CLOCK_INTEN, CTTO, 4, 1)

REG32(CLOCK_INTENSET, 0x304)
REG32(CLOCK_INTENCLR, 0x308)

REG32(CLOCK_HFCLKRUN, 0x408)
REG32(CLOCK_HFCLKSTAT, 0x40C)
    FIELD(CLOCK_HFCLKSTAT, SRC, 0, 1)
    FIELD(CLOCK_HFCLKSTAT, STATE, 16, 1)

REG32(CLOCK_LFCLKRUN, 0x414)
REG32(CLOCK_LFCLKSTAT, 0x418)
    FIELD(CLOCK_LFCLKSTAT, SRC, 0, 2)
    FIELD(CLOCK_LFCLKSTAT, STATE, 16, 1)

REG32(PWR_SYSTEMOFF, 0x500)

REG32(CLOCK_LFCLKSRC, 0x518)

struct NRF52CLOCKState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;

    uint32_t regs[NRF_CLOCK_PER_SIZE];

    MemoryRegion *downstream;
    AddressSpace downstream_as;
};


#endif //QEMU_NRF52_NRF_CLOCK_H
