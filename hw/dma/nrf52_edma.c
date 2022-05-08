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

#define _TYPE_NAME "nrf52832_soc.edma"

static void nrf52832_edma_update_irq(EDMAState *s)
{
    bool irq = false;

    irq |= (s->regs[R_EDMA_EVENT_ENDTX]   &&
            (s->regs[R_EDMA_INTEN] & R_EDMA_INTEN_ENDTX_MASK));

    qemu_set_irq(s->irq, irq);
}

static uint64_t _read(void *opaque,
                 hwaddr addr,
                 unsigned size) {

    uint64_t r;
    EDMAState *s = NRF52832_EDMA(opaque);

    info_report("nrf52832.edma: _read %08lX", addr);

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

static void _write(void *opaque,
              hwaddr addr,
              uint64_t value,
              unsigned size) {

    EDMAState *s = NRF52832_EDMA(opaque);

    info_report("nrf52832.edma: _write %08lX %lu", addr, value);

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
            s->is_spi_selected = false;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_TWI_TX_STARTED] = 1;
                // TODO uart_transmit(NULL, G_IO_OUT, s); // easy DMA start
            }
            break;
        case A_EDMA_TASKS_START_TWI_RX:
            s->is_spi_selected = false;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_TWI_RX_STARTED] = 1;
                //s->rx_started = true;
            }
            break;
        case A_EDMA_TASKS_START_SPI:
            s->is_spi_selected = true;
            if (value == 1) {
                s->regs[R_EDMA_EVENT_SPI_XFER_STARTED] = 1;
                //s->rx_started = true;
            }
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

//    s->regs[R_UART_PSELRTS] = 0xFFFFFFFF;
//    s->regs[R_UART_PSELTXD] = 0xFFFFFFFF;
//    s->regs[R_UART_PSELCTS] = 0xFFFFFFFF;
//    s->regs[R_UART_PSELRXD] = 0xFFFFFFFF;
//    s->regs[R_UART_BAUDRATE] = 0x4000000;
}

static void nrf52832_edma_realize(DeviceState *dev, Error **errp)
{
    EDMAState *s = NRF52832_EDMA(dev);

    if (!s->downstream) {
        error_report("UARTE0 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, "nrf52832_edma-downstream");
}

static const MemoryRegionOps edma_ops = {
        .read =  _read,
        .write = _write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf52832_edma_init(Object *obj)
{
    EDMAState *s = NRF52832_EDMA(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &edma_ops, s,
                          _TYPE_NAME, NRF52832_EDMA_PER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
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
