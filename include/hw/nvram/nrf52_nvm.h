/*
 * Nordic Semiconductor nRF51 non-volatile memory
 *
 * It provides an interface to erase regions in flash memory.
 * Furthermore it provides the user and factory information registers.
 *
 * QEMU interface:
 * + sysbus MMIO regions 0: NVMC peripheral registers
 * + sysbus MMIO regions 1: FICR peripheral registers
 * + sysbus MMIO regions 2: UICR peripheral registers
 * + flash-size property: flash size in bytes.
 *
 * Accuracy of the peripheral model:
 * + Code regions (MPU configuration) are disregarded.
 *
 * Copyright 2018 Steffen Görtz <contrib@steffen-goertz.de>
 *
 * This code is licensed under the GPL version 2 or later.  See
 * the COPYING file in the top-level directory.
 *
 */
#ifndef NRF52_NVM_H
#define NRF52_NVM_H

#include "hw/sysbus.h"
#include "qom/object.h"
#define TYPE_NRF52_NVM "nrf52_soc.nvm"
OBJECT_DECLARE_SIMPLE_TYPE(NRF52NVMState, NRF52_NVM)

#define NRF52_NVMC_SIZE         0x1000

#define NRF52_PAGE_SIZE         1024uL

#define NRF52_NVMC_READY        0x400
#define NRF52_NVMC_READY_READY  0x01
#define NRF52_NVMC_CONFIG       0x504
#define NRF52_NVMC_CONFIG_MASK  0x03
#define NRF52_NVMC_CONFIG_WEN   0x01
#define NRF52_NVMC_CONFIG_EEN   0x02
#define NRF52_NVMC_ERASEPCR1    0x508
#define NRF52_NVMC_ERASEPCR0    0x510
#define NRF52_NVMC_ERASEALL     0x50C
#define NRF52_NVMC_ERASEUICR    0x514
#define NRF52_NVMC_ERASE        0x01

#define NRF52_UICR_SIZE         0x210
#define NRF52_FICR_SIZE         0x460

#define NRF52_UICR_FIXTURE_SIZE (NRF52_UICR_SIZE>>2)

#define NRF52_FICR_FIXTURE_SIZE (NRF52_FICR_SIZE>>2)

struct NRF52NVMState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;
    MemoryRegion ficr;
    MemoryRegion uicr;
    MemoryRegion flash;

    uint32_t *ficr_content;
    uint32_t *uicr_content;

    uint32_t flash_size;
    uint32_t ficr_size;
    uint32_t uicr_size;

    uint8_t *storage;

    uint32_t config;

};


#endif
