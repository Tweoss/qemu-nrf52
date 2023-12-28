//
// Created by vgol on 28/12/2023.
//

#ifndef QEMU_NRF52_WRAPPER_NRF5X_PPI_H
#define QEMU_NRF52_WRAPPER_NRF5X_PPI_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/registerfields.h"

#define NRF_PPI_PER_SIZE   0x1000

#define TYPE_NRF_PPI "nrf_soc.ppi"
OBJECT_DECLARE_SIMPLE_TYPE(NRF5PPIState, NRF_PPI)

struct NRF5PPIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

//    qemu_irq irq;

    uint8_t id;

    QEMUTimer tick;

    bool enabled;
    bool running;

    uint32_t regs[NRF_PPI_PER_SIZE];
};

#endif //QEMU_NRF52_WRAPPER_NRF5X_PPI_H
