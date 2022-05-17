//
// Created by vince on 08/05/2022.
//

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "hw/dma/nrf52_edma.h"
#include "trace.h"
#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"

#define _TYPE_NAME "nrf52832_soc.edma"

static void nrf52832_edma_update_irq(EDMAState *s)
{
    bool irq = false;

    irq |= (s->regs[R_EDMA_EVENT_STOPPED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STOPPED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_ENDRX]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_ENDRX_MASK));
    irq |= (s->regs[R_EDMA_EVENT_END]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_END_MASK));
    irq |= (s->regs[R_EDMA_EVENT_ENDTX]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_ENDTX_MASK));
    irq |= (s->regs[R_EDMA_EVENT_SUSPENDED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_SUSPENDED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_TWI_RX_STARTED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STARTED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_TWI_RX_STARTED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STARTED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_SPI_XFER_STARTED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STARTED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_TWI_TX_STARTED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_TX_STARTED_MASK));
    irq |= (s->regs[R_EDMA_EVENT_STOPPED]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STOPPED_MASK));

    //info_report("nrf52832.edma: irq %d STOPPED %u", irq, s->regs[R_EDMA_EVENT_STOPPED]);

    qemu_set_irq(s->irq, irq);
}

static uint64_t _read(void *opaque,
                 hwaddr addr,
                 unsigned size) {

    uint64_t r;
    EDMAState *s = NRF52832_EDMA(opaque);

    //info_report("nrf52832.edma: _read %08lX", addr);

    switch (addr) {
        case A_EDMA_INTENSET:
        case A_EDMA_INTENCLR:
        case A_EDMA_INTEN:
            r = s->regs[R_EDMA_INTEN];
            break;
        default:
            r = s->regs[addr / 4];
            break;
    }

    return r;

}

static void timer_hit(void *opaque)
{
    struct EDMAState *s = opaque;

    //info_report("nrf52832.edma: timer_hit %d", s->transaction);

    switch (s->transaction) {
        case eEDMAtransationSPI:
            s->regs[R_EDMA_EVENT_ENDRX] = 1;
            s->regs[R_EDMA_EVENT_ENDTX] = 1;
            s->regs[R_EDMA_EVENT_END] = 1;
            s->regs[R_EDMA_EVENT_STOPPED] = 1;
            break;
        case eEDMAtransationTWI_RX:
            s->regs[R_EDMA_EVENT_LAST_RX] = 1;
            s->regs[R_EDMA_EVENT_ENDRX] = 1;
            s->regs[R_EDMA_EVENT_STOPPED] = 1;
            s->regs[R_EDMA_EVENT_END] = 1;
            s->regs[R_EDMA_RXD_AMOUNT] = s->regs[R_EDMA_RXD_CNT];
            s->regs[R_EDMA_RXD_CNT] = 0;
            break;
        case eEDMAtransationTWI_TX:
            s->regs[R_EDMA_EVENT_ENDTX] = 1;
            s->regs[R_EDMA_EVENT_STOPPED] = 1;
            s->regs[R_EDMA_EVENT_END] = 1;
            s->regs[R_EDMA_TXD_AMOUNT] = s->regs[R_EDMA_TXD_CNT];
            s->regs[R_EDMA_TXD_CNT] = 0;
            break;
        default:
            break;
    }

    nrf52832_edma_update_irq(s);
}

static void _write(void *opaque,
              hwaddr addr,
              uint64_t value,
              unsigned size) {

    EDMAState *s = NRF52832_EDMA(opaque);

    //info_report("nrf52832.edma: _write %08lX %lu", addr, value);

    switch (addr) { // EDMA

        case A_EDMA_ENABLE:
            if (value) {
                s->enabled = true;
                break;
            } else {
                s->enabled = false;
            }
            break;

            // TWI
        case A_EDMA_TASKS_START_TWI_TX:
            s->transaction = eEDMAtransationTWI_TX;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_TWI_TX_STARTED] = 1;

                if (i2c_start_send(s->i2c_bus, s->regs[R_EDMA_TWI_ADDRESS])) {
                    /* if non zero is returned, the address is not valid */
                    timer_hit((void*)s);
                } else {

                    MemTxResult result = address_space_rw(&s->downstream_as,
                                                          s->regs[R_EDMA_TXD_PTR],
                                                          MEMTXATTRS_UNSPECIFIED,
                                                          s->tx_dma,
                                                          s->regs[R_EDMA_TXD_CNT],
                                                          false);

                    int i;
                    for (i=0;i<s->regs[R_EDMA_TXD_CNT];i++) {
                        i2c_send(s->i2c_bus, s->tx_dma[i]);
                    }
                    i2c_end_transfer(s->i2c_bus);
                    (void)result;

                    ptimer_transaction_begin(s->ptimer);
                    ptimer_stop(s->ptimer);
                    ptimer_set_freq(s->ptimer, 100000);
                    ptimer_set_count(s->ptimer, 64 + (s->regs[R_EDMA_TXD_CNT] << 3));
                    ptimer_run(s->ptimer, 1);
                    ptimer_transaction_commit(s->ptimer);

                }
            }
            break;
        case A_EDMA_TASKS_START_TWI_RX:
            s->transaction = eEDMAtransationTWI_RX;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_TWI_RX_STARTED] = 1;

                if (i2c_start_recv(s->i2c_bus, s->regs[R_EDMA_TWI_ADDRESS])) {
                    /* if non zero is returned, the address is not valid */
                    timer_hit((void*)s);
                } else {

                    int i;
                    for (i=0;i<s->regs[R_EDMA_RXD_CNT];i++) {
                        s->rx_dma[i] = i2c_recv(s->i2c_bus);
                    }
                    i2c_end_transfer(s->i2c_bus);

                    MemTxResult result = address_space_rw(&s->downstream_as,
                                                          s->regs[R_EDMA_RXD_PTR],
                                                          MEMTXATTRS_UNSPECIFIED,
                                                          s->rx_dma,
                                                          s->regs[R_EDMA_RXD_CNT],
                                                          true);

                    ptimer_transaction_begin(s->ptimer);
                    ptimer_stop(s->ptimer);
                    ptimer_set_freq(s->ptimer, 100000);
                    ptimer_set_count(s->ptimer, s->regs[R_EDMA_RXD_CNT] << 3);
                    ptimer_run(s->ptimer, 1);
                    ptimer_transaction_commit(s->ptimer);
                    (void)result;
                }
            }
            break;

            // SPI
        case A_EDMA_TASKS_START_SPI:
            s->transaction = eEDMAtransationSPI;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_SPI_XFER_STARTED] = 1;

                qemu_set_irq(s->cs_lines[0], 0);

                MemTxResult result;
                if (s->regs[R_EDMA_TXD_CNT]) {
                    result = address_space_rw(&s->downstream_as,
                                                          s->regs[R_EDMA_TXD_PTR],
                                                          MEMTXATTRS_UNSPECIFIED,
                                                          s->tx_dma,
                                                          s->regs[R_EDMA_TXD_CNT],
                                                          false);
                    (void) result;
                }

                uint32_t length = s->regs[R_EDMA_TXD_CNT] > s->regs[R_EDMA_RXD_CNT] ?
                        s->regs[R_EDMA_TXD_CNT] : s->regs[R_EDMA_RXD_CNT];

//                info_report("nrf52832.edma: xfer size: %u %u %u",
//                            s->regs[R_EDMA_TXD_CNT],
//                            s->regs[R_EDMA_RXD_CNT],
//                            length);

//                printf("TX:");
//                for (int i=0;i<length;i++) {
//                    printf(" %02X" , s->tx_dma[i]);
//                }
//                printf("\n");

                for (int i=0;i<length;i++) {
                    uint8_t byte = ssi_transfer(s->bus, s->tx_dma[i]);
                    if (i < s->regs[R_EDMA_RXD_CNT]) {
                        s->rx_dma[i] = byte;
                    } else {
                        s->rx_dma[i] = 0xFF;
                    }
                }

//                printf("RX:");
//                for (int i=0;i<length;i++) {
//                    printf(" %02X" , s->rx_dma[i]);
//                }
//                printf("\n");

                if (s->regs[R_EDMA_RXD_CNT]) {

                    result = address_space_rw(&s->downstream_as,
                                              s->regs[R_EDMA_RXD_PTR],
                                              MEMTXATTRS_UNSPECIFIED,
                                              s->rx_dma,
                                              s->regs[R_EDMA_RXD_CNT],
                                              true);
                    (void)result;
                }

//                s->regs[R_EDMA_RXD_CNT] = length;
                s->regs[R_EDMA_TXD_CNT] = 0;

                qemu_set_irq(s->cs_lines[0], 1);

                ptimer_transaction_begin(s->ptimer);
                ptimer_stop(s->ptimer);
                ptimer_set_freq(s->ptimer, 1000000); // 1 MHz
                ptimer_set_count(s->ptimer, 8 + (length << 3));
                ptimer_run(s->ptimer, 1);
                ptimer_transaction_commit(s->ptimer);
            }
            break;

        case A_EDMA_TASKS_STOP:
            s->regs[R_EDMA_EVENT_STOPPED] = 1;
            s->regs[R_EDMA_EVENT_ENDRX] = 1;
            ptimer_transaction_begin(s->ptimer);
            ptimer_stop(s->ptimer);
            ptimer_transaction_commit(s->ptimer);
            break;

        case A_EDMA_INTEN:
            s->regs[R_EDMA_INTEN] = value;
            break;
        case A_EDMA_INTENSET:
            s->regs[R_EDMA_INTEN] |= value;
            break;
        case A_EDMA_INTENCLR:
            s->regs[R_EDMA_INTEN] &= ~value;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    nrf52832_edma_update_irq(s);
}

static void nrf52832_edma_reset(DeviceState *dev)
{
    EDMAState *s = NRF52832_EDMA(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->enabled = false;

    s->regs[R_EDMA_PSELRTS] = 0xFFFFFFFF;
    s->regs[R_EDMA_PSELTXD] = 0xFFFFFFFF;
    s->regs[R_EDMA_PSELCTS] = 0xFFFFFFFF;
    s->regs[R_EDMA_PSELRXD] = 0xFFFFFFFF;
    s->regs[R_EDMA_BAUDRATE] = 0x4000000;
}

static const MemoryRegionOps edma_ops = {
        .read =  _read,
        .write = _write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf52832_edma_realize(DeviceState *dev, Error **errp)
{
    EDMAState *s = NRF52832_EDMA(dev);

    s->bus = ssi_create_bus(dev, "ssi");

    s->i2c_bus = i2c_init_bus(dev, "i2c");

    s->ptimer = ptimer_init(timer_hit, s, PTIMER_POLICY_DEFAULT);

    if (!s->downstream) {
        error_report("!! 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, "nrf52832_edma-downstream");

    memory_region_init_io(&s->iomem, OBJECT(dev), &edma_ops, s,
                          _TYPE_NAME, NRF52832_EDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    qdev_init_gpio_out_named(dev, s->cs_lines, "cs_lines", NUM_SPI_SLAVES);
}

static void nrf52832_edma_init(Object *obj)
{
//    EDMAState *s = NRF52832_EDMA(obj);
//    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

}

static int nrf52832_edma_post_load(void *opaque, int version_id)
{
    EDMAState *s = NRF52832_EDMA(opaque);

    // empty
    (void)s;

    return 0;
}

static const VMStateDescription nrf52832_edma_vmstate = {
        .name = _TYPE_NAME,
        .post_load = nrf52832_edma_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_UINT32_ARRAY(regs, EDMAState, 0x600),
                VMSTATE_BOOL(enabled, EDMAState),
                VMSTATE_END_OF_LIST()
        }
};

static Property nrf52832_edma_properties[] = {
        DEFINE_PROP_LINK("downstream", EDMAState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf52832_edma_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf52832_edma_reset;
    dc->realize = nrf52832_edma_realize;
    device_class_set_props(dc, nrf52832_edma_properties);
    dc->vmsd = &nrf52832_edma_vmstate;
}

static const TypeInfo nrf52832_edma_info = {
        .name = TYPE_NRF52832_EDMA,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(EDMAState),
        .instance_init = nrf52832_edma_init,
        .class_init = nrf52832_edma_class_init
};

static void nrf52832_edma_register_types(void)
{
    type_register_static(&nrf52832_edma_info);
}

type_init(nrf52832_edma_register_types)
