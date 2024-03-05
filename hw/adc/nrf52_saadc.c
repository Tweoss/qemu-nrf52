//
// Created by vgol on 05/03/2024.
//


#include "qemu/osdep.h"
#include "migration/vmstate.h"
#include "hw/qdev-properties.h"
#include "qapi/error.h"
#include "qemu/module.h"
#include "qom/object.h"
#include "hw/qdev-properties.h"
#include "hw/irq.h"
#include "hw/ptimer.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/adc/nrf52_saadc.h"
#include "trace.h"

#define _TYPE_NAME TYPE_NRF52_SAADC

#define MODULE_OBJ_NAME NRF52SAADCState
#include "hw/dma/nrf5x_ppi.h"


static void _shorts_process(NRF52SAADCState *s);

static void nrf52832_saadc_update_irq(NRF52SAADCState *s)
{

}

static void trigger_event_r(NRF52SAADCState *s, uint32_t r_event) {

//    info_report(_TYPE_NAME": event 0x%04X", r_event << 2u);

    s->regs[r_event] = 1;

    // check shorts
    _shorts_process(s);

    // check PPI
    PPI_EVENT_R(s, r_event);

    nrf52832_saadc_update_irq(s);
}

static uint64_t nrf52_saadc_read(void *opaque, hwaddr addr, unsigned int size)
{
    NRF52SAADCState *s = NRF52_SAADC(opaque);
    uint64_t r = 0;

    switch (addr) {

        case A_SAADC_INTENCLR:
        case A_SAADC_INTENSET:
            r = s->regs[R_SAADC_INTEN];
            break;


        default:
            r = s->regs[addr / 4];
            break;
    }

    info_report("nrf52.saadc: _read %08lX = %lu", addr, r);

    nrf52832_saadc_update_irq(s);

    return r;
}

static void nrf52_saadc_write(void *opaque, hwaddr addr,
                               uint64_t value, unsigned int size)
{
    NRF52SAADCState *s = NRF52_SAADC(opaque);

    info_report("nrf52.saadc: _write %08lX %lu", addr, value);

    switch (addr) {

        case A_SAADC_INTENSET:
            s->regs[R_SAADC_INTEN] |= value;
            break;
        case A_SAADC_INTENCLR:
            s->regs[R_SAADC_INTEN] &= ~value;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    nrf52832_saadc_update_irq(s);
}

static void xfer_expire(void *opaque)
{
    NRF52SAADCState *s = NRF52_SAADC(opaque);



    nrf52832_saadc_update_irq(s);
}

static const MemoryRegionOps saadc_ops = {
        .read =  nrf52_saadc_read,
        .write = nrf52_saadc_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl.min_access_size = 4,
        .impl.max_access_size = 4,
};

static void nrf52_saadc_reset(DeviceState *dev)
{
//    NRF52SAADCState *s = NRF52_SAADC(dev);
}

static void nrf52_saadc_init(Object *obj)
{
//    NRF52SAADCState *s = NRF52_SAADC(dev);
}

static void nrf52_saadc_realize(DeviceState *dev, Error **errp)
{
    NRF52SAADCState *s = NRF52_SAADC(dev);

    timer_init_us(&s->tick, QEMU_CLOCK_VIRTUAL, xfer_expire, s);

    if (!s->downstream) {
        error_report("!! 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");

    memory_region_init_io(&s->iomem, OBJECT(dev), &saadc_ops, s,
                          TYPE_NRF52_SAADC, NRF52_SAADC_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);
}

static Property nrf52_saadc_properties[] = {
        DEFINE_PROP_LINK("downstream", NRF52SAADCState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf52_saadc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf52_saadc_reset;
    dc->realize = nrf52_saadc_realize;
    device_class_set_props(dc, nrf52_saadc_properties);
}

static const TypeInfo nrf52_saadc_info = {
        .name = TYPE_NRF52_SAADC,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF52SAADCState),
        .instance_init = nrf52_saadc_init,
        .class_init = nrf52_saadc_class_init
};

static void nrf52_saadc_register_types(void)
{
    type_register_static(&nrf52_saadc_info);
}

type_init(nrf52_saadc_register_types)
