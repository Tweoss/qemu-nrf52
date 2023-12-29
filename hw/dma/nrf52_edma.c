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

#define _TYPE_NAME TYPE_NRF52832_EDMA

#define MODULE_OBJ_NAME EDMAState
#include "hw/dma/nrf5x_ppi.h"

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

    irq |= (s->regs[R_EDMA_ERRORSRC] &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_ERROR_MASK));

//    if (irq)
//        info_report(_TYPE_NAME": irq %d STOPPED %u", irq, s->regs[R_EDMA_EVENT_STOPPED]);

    qemu_set_irq(s->irq, irq);

    // TODO Events can be generated by the peripheral even when the event register is set to '1'.

    s->error = 0;
}

static void _shorts_process(EDMAState *s);

static void xfer_rearm(void *opaque, int64_t timer_per_us)
{
    EDMAState *s = opaque;

    int64_t now = qemu_clock_get_us(QEMU_CLOCK_VIRTUAL);

    timer_mod(&s->tick, now + timer_per_us);

//    info_report(_TYPE_NAME": timer_per_us %lu", (long unsigned)timer_per_us);
}

static void trigger_event_r(struct EDMAState *s, uint32_t r_event) {

//    info_report(_TYPE_NAME": event 0x%04X", r_event << 2u);

    s->regs[r_event] = 1;

    // check shorts
    _shorts_process(s);

    // check PPI
    PPI_EVENT_R(s, r_event);

    nrf52832_edma_update_irq(s);
}

static void xfer_expire(void *opaque)
{
    struct EDMAState *s = opaque;

    //info_report(_TYPE_NAME": timer_hit %d", s->transaction);

    switch (s->transaction) {
        case eEDMAtransationSPI:
            {
                // clear transaction
                s->transaction = eEDMAtransationNone;

                MemTxResult result;
                if (s->regs[R_EDMA_TXD_CNT]) {
                    (void)address_space_rw(&s->downstream_as,
                                           s->regs[R_EDMA_TXD_PTR],
                                           MEMTXATTRS_UNSPECIFIED,
                                           s->tx_dma,
                                           s->regs[R_EDMA_TXD_CNT],
                                           false);
                }

                uint32_t length = s->regs[R_EDMA_TXD_CNT] > s->regs[R_EDMA_RXD_CNT] ?
                                  s->regs[R_EDMA_TXD_CNT] : s->regs[R_EDMA_RXD_CNT];

                for (int i=0;i<length;i++) {
                    uint32_t to_tx = 0xFF;
                    if (i < s->regs[R_EDMA_TXD_CNT]) {
                        to_tx = s->tx_dma[i];
                    }
                    if (s->regs[R_EDMA_TXD_CNT] && i == s->regs[R_EDMA_TXD_CNT] - 1) {
                        trigger_event_r(s, R_EDMA_EVENT_ENDTX);
                    }
                    uint8_t byte = ssi_transfer(s->bus, to_tx);
                    if (i < s->regs[R_EDMA_RXD_CNT]) {
                        s->rx_dma[i] = byte;
                    }
                    if (s->regs[R_EDMA_RXD_CNT] && i == s->regs[R_EDMA_RXD_CNT] - 1) {
                        trigger_event_r(s, R_EDMA_EVENT_ENDRX);
                    }
                }

                s->regs[R_EDMA_TXD_CNT] = 0;

                qemu_set_irq(s->cs_lines[0], 1); // deactivate slave

                if (s->regs[R_EDMA_RXD_CNT]) { // write data to RAM

                    result = address_space_rw(&s->downstream_as,
                                              s->regs[R_EDMA_RXD_PTR],
                                              MEMTXATTRS_UNSPECIFIED,
                                              s->rx_dma,
                                              s->regs[R_EDMA_RXD_CNT],
                                              true);
                    (void)result;
                }

                trigger_event_r(s, R_EDMA_EVENT_END);
                trigger_event_r(s, R_EDMA_EVENT_STOPPED);
            }
            break;
        case eEDMAtransationTWI_RX:
            {
                // clear transaction
                s->transaction = eEDMAtransationNone;

                int val = i2c_start_recv(s->i2c_bus, s->regs[R_EDMA_TWI_ADDRESS]);
                if (val) {
                    /* if non zero is returned, the address is not valid */
                    s->regs[R_EDMA_ERRORSRC] |= R_EDMA_ERRORSRC_ANACK_MASK;

                    trigger_event_r(s, R_EDMA_EVENT_ERROR);
                    trigger_event_r(s, R_EDMA_EVENT_STOPPED);
                    warn_report(_TYPE_NAME": slave not found (0x%02X)", s->regs[R_EDMA_TWI_ADDRESS]);
                } else {

                    int i;
                    for (i=0;i<s->regs[R_EDMA_RXD_CNT];i++) {
                        s->rx_dma[i] = i2c_recv(s->i2c_bus);
                        // The TWI master will generate a LASTTX event when it starts to transmit the last byte
                        if (s->regs[R_EDMA_RXD_CNT] == 1 ||
                            i == s->regs[R_EDMA_RXD_CNT] - 2) {
                            trigger_event_r(s, R_EDMA_EVENT_LAST_RX);
                        }
                    }
                    i2c_end_transfer(s->i2c_bus);

                    // transfer back to memory
                    (void)address_space_rw(&s->downstream_as,
                                           s->regs[R_EDMA_RXD_PTR],
                                           MEMTXATTRS_UNSPECIFIED,
                                           s->rx_dma,
                                           s->regs[R_EDMA_RXD_CNT],
                                           true);

                    s->regs[R_EDMA_RXD_AMOUNT] = s->regs[R_EDMA_RXD_CNT];
                    s->regs[R_EDMA_RXD_CNT] = 0;

                    if (s->async_stop) {
                        trigger_event_r(s, R_EDMA_EVENT_STOPPED);
                        s->async_stop = 0;
                    }
                }
            }
            if (0 == (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STOPPED_MASK)) {
                s->error = 1;
            }
            break;
        case eEDMAtransationTWI_TX:
            {
                // clear transaction
                s->transaction = eEDMAtransationNone;

                int val = i2c_start_send(s->i2c_bus, s->regs[R_EDMA_TWI_ADDRESS]);
                if (val) {
                    /* if non zero is returned, the address is not valid */
                    s->regs[R_EDMA_ERRORSRC] |= R_EDMA_ERRORSRC_ANACK_MASK;

                    trigger_event_r(s, R_EDMA_EVENT_ERROR);
                    trigger_event_r(s, R_EDMA_EVENT_STOPPED);
                    warn_report(_TYPE_NAME": slave not found (0x%02X)", s->regs[R_EDMA_TWI_ADDRESS]);
                } else {

                    // read data from address space
                    (void)address_space_rw(&s->downstream_as,
                                           s->regs[R_EDMA_TXD_PTR],
                                           MEMTXATTRS_UNSPECIFIED,
                                           s->tx_dma,
                                           s->regs[R_EDMA_TXD_CNT],
                                           false);

                    int i;
                    for (i=0;i<s->regs[R_EDMA_TXD_CNT];i++) {
                        i2c_send(s->i2c_bus, s->tx_dma[i]);
                        // The TWI master will generate a LASTTX event when it starts to transmit the last byte
                        if (s->regs[R_EDMA_TXD_CNT] == 1 ||
                            i == s->regs[R_EDMA_TXD_CNT] - 2) {
                            trigger_event_r(s, R_EDMA_EVENT_LAST_TX);
                        }
                    }
                    i2c_end_transfer(s->i2c_bus);

                    s->regs[R_EDMA_TXD_AMOUNT] = s->regs[R_EDMA_TXD_CNT];
                    s->regs[R_EDMA_TXD_CNT] = 0;

                    if (s->async_stop) {
                        trigger_event_r(s, R_EDMA_EVENT_STOPPED);
                        s->async_stop = 0;
                    }
                }

            }
            if (0 == (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STOPPED_MASK)) {
                s->error = 1;
            }
            break;
        default:
            if (0 == (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_STOPPED_MASK)) {
                s->error = 1;
            }
            break;
    }
}

static void _nrf_write(void *opaque,
              hwaddr addr,
              uint64_t value,
              unsigned size) {

    EDMAState *s = NRF52832_EDMA(opaque);

//    info_report(_TYPE_NAME": _nrf_write %03llX <- %08llX", addr, value);

    switch (addr) { // EDMA

        case A_EDMA_ENABLE:
            if (value) {
                s->enabled = true;
                break;
            } else {
                s->enabled = false;
            }
            break;

            // SPI
        case A_EDMA_TASKS_START_SPI:
            s->error = 0;
            s->transaction = eEDMAtransationSPI;
            if (value == 1) {
                trigger_event_r(s, R_EDMA_EVENT_SPI_XFER_STARTED);

                qemu_set_irq(s->cs_lines[0], 0); // activate slave

                uint32_t length = s->regs[R_EDMA_TXD_CNT] > s->regs[R_EDMA_RXD_CNT] ?
                                  s->regs[R_EDMA_TXD_CNT] : s->regs[R_EDMA_RXD_CNT];

                // 1MHz = 8 us per byte
                xfer_rearm(s, 64 + 8 * length);
            }
            break;

            // TWI
        case A_EDMA_TASKS_START_TWI_TX:
            s->error = 0;
            s->transaction = eEDMAtransationTWI_TX;
            if (value && s->i2c_bus != NULL) {

                trigger_event_r(s, R_EDMA_EVENT_TWI_TX_STARTED);
                // 100kHz = 100 us per byte
                xfer_rearm(s, 64 + 100 * s->regs[R_EDMA_TXD_CNT]);
            }
            break;
        case A_EDMA_TASKS_START_TWI_RX:
            s->error = 0;
            s->transaction = eEDMAtransationTWI_RX;
            if (value && s->i2c_bus != NULL) {
                trigger_event_r(s, R_EDMA_EVENT_TWI_RX_STARTED);

                // 100kHz = 100 us per byte
                xfer_rearm(s, 64 + 100 * s->regs[R_EDMA_RXD_CNT]);
            }
            break;

        case A_EDMA_TASKS_RESUME:
            // info_report("EDMA resume");
            break;

        case A_EDMA_TASKS_STOP:
            s->async_stop = true;
            break;

        case A_EDMA_INTEN:
            s->regs[R_EDMA_INTEN] = value;
            break;
        case A_EDMA_INTENSET:
            s->regs[R_EDMA_INTEN] |= value;
            break;
        case A_EDMA_INTENCLR:
            s->regs[R_EDMA_INTEN] &= ~value;
            s->regs[R_EDMA_TXD_CNT] = 0;
            s->regs[R_EDMA_RXD_CNT] = 0;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    nrf52832_edma_update_irq(s);
}

static uint64_t _nrf_read(void *opaque,
                          hwaddr addr,
                          unsigned size) {

    uint64_t r;
    EDMAState *s = NRF52832_EDMA(opaque);

    //info_report(_TYPE_NAME": _nrf_read %08lX", addr);

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

static void _shorts_process(EDMAState *s) {

    if (s->regs[R_EDMA_EVENT_LAST_TX]) {

        if (s->regs[R_EDMA_SHORTS] & R_EDMA_SHORTS_TWIM_LASTTX_STARTRX_MASK)
        {
            s->regs[R_EDMA_SHORTS] &= ~R_EDMA_SHORTS_TWIM_LASTTX_STARTRX_MASK;
            // info_report(_TYPE_NAME": _shorts_process -> STARTRX");
            _nrf_write(s, A_EDMA_TASKS_START_TWI_RX, 1, 0);
        }

        if (s->regs[R_EDMA_SHORTS] & R_EDMA_SHORTS_TWIM_LASTTX_SUSPEND_MASK)
        {
            s->regs[R_EDMA_SHORTS] &= ~R_EDMA_SHORTS_TWIM_LASTTX_SUSPEND_MASK;
            // info_report(_TYPE_NAME": _shorts_process -> SUSPEND");
            trigger_event_r(s, R_EDMA_EVENT_SUSPENDED);
        }

        if (s->regs[R_EDMA_SHORTS] & R_EDMA_SHORTS_TWIM_LASTTX_STOP_MASK)
        {
            s->regs[R_EDMA_SHORTS] &= ~R_EDMA_SHORTS_TWIM_LASTTX_STOP_MASK;
            // info_report(_TYPE_NAME": _shorts_process -> STOP");
            s->async_stop = true;
//            trigger_event_r(s, R_EDMA_EVENT_STOPPED);
        }

    }

    if (s->regs[R_EDMA_EVENT_LAST_RX]) {

        if (s->regs[R_EDMA_SHORTS] & R_EDMA_SHORTS_TWIM_LASTRX_STARTTX_MASK)
        {
            s->regs[R_EDMA_SHORTS] &= ~R_EDMA_SHORTS_TWIM_LASTRX_STARTTX_MASK;
            // info_report(_TYPE_NAME": _shorts_process -> STARTTX");
            _nrf_write(s, A_EDMA_TASKS_START_TWI_TX, 1, 0);
        }

        if (s->regs[R_EDMA_SHORTS] & R_EDMA_SHORTS_TWIM_LASTRX_STOP_MASK)
        {
            s->regs[R_EDMA_SHORTS] &= ~R_EDMA_SHORTS_TWIM_LASTRX_STOP_MASK;
            // info_report(_TYPE_NAME": _shorts_process -> STOP");
            s->async_stop = true;
//            trigger_event_r(s, R_EDMA_EVENT_STOPPED);
        }

    }
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
        .read =  _nrf_read,
        .write = _nrf_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf52832_edma_realize(DeviceState *dev, Error **errp)
{
    EDMAState *s = NRF52832_EDMA(dev);

    s->bus = ssi_create_bus(dev, "ssi");

    s->i2c_bus = i2c_init_bus(dev, "i2c");

    timer_init_us(&s->tick, QEMU_CLOCK_VIRTUAL, xfer_expire, s);

    if (!s->downstream) {
        error_report("!! 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");

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
