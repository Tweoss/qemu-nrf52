//
// Created by vgol on 05/03/2024.
//

#ifndef QEMU_NRF52_WRAPPER_NRF52832_SAADC_H
#define QEMU_NRF52_WRAPPER_NRF52832_SAADC_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/registerfields.h"
#include "exec/memory.h"

#define NRF52_SAADC_PER_SIZE   0x1000

#define TYPE_NRF52_SAADC "nrf52832_soc.saadc"
OBJECT_DECLARE_SIMPLE_TYPE(NRF52SAADCState, NRF52_SAADC)


REG32(SAADC_TASKS_STOP, 0x014)
REG32(SAADC_TASKS_SUSPEND, 0x01C)
REG32(SAADC_TASKS_RESUME, 0x020)

REG32(SAADC_INTEN, 0x300)
    FIELD(SAADC_INTEN, STOPPED, 1, 1)
    FIELD(SAADC_INTEN, ENDRX, 4, 1)
    FIELD(SAADC_INTEN, END, 6, 1)
    FIELD(SAADC_INTEN, ENDTX, 8, 1)
    FIELD(SAADC_INTEN, ERROR, 9, 1)
    FIELD(SAADC_INTEN, SUSPENDED, 18, 1)
    FIELD(SAADC_INTEN, STARTED, 19, 1)
    FIELD(SAADC_INTEN, TX_STARTED, 20, 1)
    FIELD(SAADC_INTEN, TX_STOPPED, 22, 1)
    FIELD(SAADC_INTEN, LASTRX, 23, 1)
    FIELD(SAADC_INTEN, LASTTX, 24, 1)
REG32(SAADC_INTENSET, 0x304)
REG32(SAADC_INTENCLR, 0x308)


struct NRF52SAADCState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    qemu_irq irq;

    uint8_t id;

    QEMUTimer tick;

    bool enabled;
    bool running;

    uint32_t regs[NRF52_SAADC_PER_SIZE];

    MemoryRegion *downstream;
    AddressSpace downstream_as;

};

#endif //QEMU_NRF52_WRAPPER_NRF52832_SAADC_H
