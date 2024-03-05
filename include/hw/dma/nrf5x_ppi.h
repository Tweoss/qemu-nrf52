//
// Created by vgol on 28/12/2023.
//

#ifndef QEMU_NRF52_WRAPPER_NRF5X_PPI_H
#define QEMU_NRF52_WRAPPER_NRF5X_PPI_H

#include "hw/sysbus.h"
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/registerfields.h"
#include "exec/memory.h"

#define NRF_PPI_PER_SIZE   0x1000

#define NRFxxxxx_PPI_BASE  0x4001F000

#define TYPE_NRF_PPI "nrf_soc.ppi"
OBJECT_DECLARE_SIMPLE_TYPE(NRF5PPIState, NRF_PPI)

REG32(PPI_CHG0_EN, 0x000)
REG32(PPI_CHG0_DIS, 0x004)

REG32(PPI_CHG3_EN, 0x018)
REG32(PPI_CHG3_DIS, 0x01C)

REG32(PPI_CHEN, 0x500)
REG32(PPI_CHENSET, 0x504)
REG32(PPI_CHENCLR, 0x508)

#define PPI_CHANNEL_NB 32u

REG32(PPI_CH0_EEP, 0x510)
REG32(PPI_CH0_TEP, 0x514)

REG32(PPI_CH15_EEP, 0x588)
REG32(PPI_CH15_TEP, 0x58C)

REG32(PPI_EVENT_IN, 0x850)

struct NRF5PPIState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint8_t id;

    QEMUTimer tick;

    bool enabled;
    bool running;

    uint32_t regs[NRF_PPI_PER_SIZE];

    MemoryRegion *downstream;
    AddressSpace downstream_as;
};

#ifdef MODULE_OBJ_NAME
static inline void _trigger_ppi_event(MODULE_OBJ_NAME *s, uint64_t event_offset) {

    if (!s->downstream) {
        return;
    }

    hwaddr addr = NRFxxxxx_PPI_BASE | A_PPI_EVENT_IN;
    uint32_t per_add = event_offset | s->iomem.addr;

//    info_report("nrf.rtc: dispatching to PPI %d", s->downstream->alias!=NULL);

    MemTxResult res = address_space_rw(&s->downstream_as,
                                       addr,
                                       MEMTXATTRS_UNSPECIFIED,
                                       &per_add,
                                       sizeof(per_add),
                                       true);

    if (res) {
#ifdef _TYPE_NAME
        error_report(_TYPE_NAME": error %u addr 0x%llX", res, addr);
#else
        error_report("UNKNOWN: error %u addr 0x%llX", res, addr);
#endif
    }
}

#define PPI_EVENT_R(OBJECT,R_ADDR)    _trigger_ppi_event((OBJECT), (R_ADDR) << 2u);
#define PPI_EVENT_A(OBJECT,A_ADDR)    _trigger_ppi_event((OBJECT), (A_ADDR));

#endif

#endif //QEMU_NRF52_WRAPPER_NRF5X_PPI_H
