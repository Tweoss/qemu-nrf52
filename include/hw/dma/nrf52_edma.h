//
// Created by vince on 08/05/2022.
//

#ifndef QEMU_NRF52_NRF52_EDMA_H
#define QEMU_NRF52_NRF52_EDMA_H

#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"

#define TYPE_NRF52832_EDMA "nrf52832.edma"
OBJECT_DECLARE_SIMPLE_TYPE(EDMAState, NRF52832_EDMA)


/* Bit 12 : Shortcut between LASTRX event and STOP task */
#define TWIM_SHORTS_LASTRX_STOP_Pos (12UL) /*!< Position of LASTRX_STOP field. */
/* Bit 10 : Shortcut between LASTRX event and STARTTX task */
#define TWIM_SHORTS_LASTRX_STARTTX_Pos (10UL) /*!< Position of LASTRX_STARTTX field. */
/* Bit 9 : Shortcut between LASTTX event and STOP task */
#define TWIM_SHORTS_LASTTX_STOP_Pos (9UL) /*!< Position of LASTTX_STOP field. */
/* Bit 8 : Shortcut between LASTTX event and SUSPEND task */
#define TWIM_SHORTS_LASTTX_SUSPEND_Pos (8UL) /*!< Position of LASTTX_SUSPEND field. */
/* Bit 7 : Shortcut between LASTTX event and STARTRX task */
#define TWIM_SHORTS_LASTTX_STARTRX_Pos (7UL) /*!< Position of LASTTX_STARTRX field. */


REG32(EDMA_TASKS_START_TWI_RX, 0x000)
REG32(EDMA_TASKS_START_TWI_TX, 0x008)
REG32(EDMA_TASKS_START_SPI, 0x010)

REG32(EDMA_TASKS_STOP, 0x014)
REG32(EDMA_TASKS_SUSPEND, 0x01C)
REG32(EDMA_TASKS_RESUME, 0x020)

REG32(EDMA_EVENT_STOPPED, 0x104)
REG32(EDMA_EVENT_ENDRX, 0x110) // not for EDMA TWIM, only SPIM
REG32(EDMA_EVENT_END, 0x118) // not for EDMA TWIM, only SPIM
REG32(EDMA_EVENT_ENDTX, 0x120) // not for EDMA TWIM, only SPIM
REG32(EDMA_EVENT_ERROR, 0x124)
REG32(EDMA_EVENT_SUSPENDED, 0x148)
REG32(EDMA_EVENT_TWI_RX_STARTED, 0x14C)
REG32(EDMA_EVENT_TWI_TX_STARTED, 0x150)
REG32(EDMA_EVENT_SPI_XFER_STARTED, 0x14C)
REG32(EDMA_EVENT_LAST_RX, 0x15C)
REG32(EDMA_EVENT_LAST_TX, 0x160)

REG32(EDMA_SHORTS, 0x200)
    FIELD(EDMA_SHORTS, TWIM_LASTTX_STARTRX, TWIM_SHORTS_LASTTX_STARTRX_Pos, 1)
    FIELD(EDMA_SHORTS, TWIM_LASTTX_SUSPEND, TWIM_SHORTS_LASTTX_SUSPEND_Pos, 1)
    FIELD(EDMA_SHORTS, TWIM_LASTTX_STOP, TWIM_SHORTS_LASTTX_STOP_Pos, 1)
    FIELD(EDMA_SHORTS, TWIM_LASTRX_STARTTX, TWIM_SHORTS_LASTRX_STARTTX_Pos, 1)
    FIELD(EDMA_SHORTS, TWIM_LASTRX_STOP, TWIM_SHORTS_LASTRX_STOP_Pos, 1)

REG32(EDMA_INTEN, 0x300)
    FIELD(EDMA_INTEN, STOPPED, 1, 1)
    FIELD(EDMA_INTEN, ENDRX, 4, 1)
    FIELD(EDMA_INTEN, END, 6, 1)
    FIELD(EDMA_INTEN, ENDTX, 8, 1)
    FIELD(EDMA_INTEN, ERROR, 9, 1)
    FIELD(EDMA_INTEN, SUSPENDED, 18, 1)
    FIELD(EDMA_INTEN, STARTED, 19, 1)
    FIELD(EDMA_INTEN, TX_STARTED, 20, 1)
    FIELD(EDMA_INTEN, TX_STOPPED, 22, 1)
    FIELD(EDMA_INTEN, LASTRX, 23, 1)
    FIELD(EDMA_INTEN, LASTTX, 24, 1)
REG32(EDMA_INTENSET, 0x304)
REG32(EDMA_INTENCLR, 0x308)
REG32(EDMA_ERRORSRC, 0x4C4)
    FIELD(EDMA_ERRORSRC, OVERRUN, 0, 1)
    FIELD(EDMA_ERRORSRC, ANACK, 1, 1)
    FIELD(EDMA_ERRORSRC, DNACK, 2, 1)

REG32(EDMA_ENABLE, 0x500)
REG32(EDMA_PSELRTS, 0x508)
REG32(EDMA_PSELTXD, 0x50C)
REG32(EDMA_PSELCTS, 0x510)
REG32(EDMA_PSELRXD, 0x514)
REG32(EDMA_RXD, 0x518)
REG32(EDMA_TXD, 0x51C)
REG32(EDMA_BAUDRATE, 0x524)

REG32(EDMA_RXD_PTR, 0x534)
REG32(EDMA_RXD_CNT, 0x538)
REG32(EDMA_RXD_AMOUNT, 0x53C)
REG32(EDMA_RXD_LIST, 0x540)
REG32(EDMA_TXD_PTR, 0x544)
REG32(EDMA_TXD_CNT, 0x548)
REG32(EDMA_TXD_AMOUNT, 0x54C)
REG32(EDMA_TXD_LIST, 0x550)

REG32(EDMA_CONFIG, 0x554)

REG32(EDMA_TWI_ADDRESS, 0x588)

REG32(EDMA_SPI_ORC, 0x5C0)

#define NRF52832_EDMA_PER_SIZE 0x600

typedef enum {
    eEDMAtransationNone = 0,
    eEDMAtransationSPI,
    eEDMAtransationTWI_RX,
    eEDMAtransationTWI_TX,
} eEDMAtransation;

#define NUM_SPI_SLAVES 4

struct EDMAState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    qemu_irq irq;

    QEMUTimer tick;

    qemu_irq cs_lines[NUM_SPI_SLAVES];

    SSIBus *bus;
    I2CBus *i2c_bus;

    uint8_t rx_dma[64*1024];
    uint8_t tx_dma[64*1024];

    uint32_t error;

    bool enabled;
    bool async_stop;
    eEDMAtransation transaction;

    uint32_t regs[NRF52832_EDMA_PER_SIZE];

    MemoryRegion *downstream;
    AddressSpace downstream_as;
};

#endif //QEMU_NRF52_NRF52_EDMA_H
