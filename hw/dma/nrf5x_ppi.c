//
// Created by vgol on 28/12/2023.
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
#include "trace.h"
#include "hw/dma/nrf5x_ppi.h"


static void _update_irq(NRF5PPIState *s) {

}

static void _nrf_write(void *opaque,
                       hwaddr addr,
                       uint64_t value,
                       unsigned size) {

    NRF5PPIState *s = NRF_PPI(opaque);

    info_report(TYPE_NRF_PPI": _nrf_write %03llX <- %llX", addr, value);

    switch (addr) { // EDMA

        default:
            s->regs[addr >> 2] = value;
            break;
    }

    _update_irq(s);
}

static uint64_t _nrf_read(void *opaque,
                          hwaddr addr,
                          unsigned size) {

    uint64_t r;
    NRF5PPIState *s = NRF_PPI(opaque);

    info_report(TYPE_NRF_PPI": _nrf_read %08llX", addr);

    switch (addr) {

        default:
            r = s->regs[addr >> 2];
            break;
    }

    return r;

}

static void nrf5x_ppi_reset(DeviceState *dev)
{
    NRF5PPIState *s = NRF_PPI(dev);

    memset(s->regs, 0, sizeof(s->regs));

    s->enabled = false;

}

static const MemoryRegionOps edma_ops = {
        .read =  _nrf_read,
        .write = _nrf_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf5x_ppi_realize(DeviceState *dev, Error **errp)
{
    NRF5PPIState *s = NRF_PPI(dev);

//    if (!s->downstream) {
//        error_report("!! 'downstream' link not set");
//        return;
//    }
//
//    address_space_init(&s->downstream_as, s->downstream, "nrf5x_ppi-downstream");

    memory_region_init_io(&s->iomem, OBJECT(dev), &edma_ops, s,
                          TYPE_NRF_PPI, NRF_PPI_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
//    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

}

static void nrf5x_ppi_init(Object *obj)
{
//    NRF5PPIState *s = NRF_PPI(obj);
//    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

}

static int nrf5x_ppi_post_load(void *opaque, int version_id)
{
    NRF5PPIState *s = NRF_PPI(opaque);

    // empty
    (void)s;

    return 0;
}

static const VMStateDescription nrf5x_ppi_vmstate = {
        .name = TYPE_NRF_PPI,
        .post_load = nrf5x_ppi_post_load,
        .fields = (VMStateField[]) {
                VMSTATE_UINT32_ARRAY(regs, NRF5PPIState, NRF_PPI_PER_SIZE),
                VMSTATE_BOOL(enabled, NRF5PPIState),
                VMSTATE_END_OF_LIST()
        }
};

static Property nrf5x_ppi_properties[] = {
//        DEFINE_PROP_LINK("downstream", NRF5PPIState, downstream,
//                         TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf5x_ppi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf5x_ppi_reset;
    dc->realize = nrf5x_ppi_realize;
    device_class_set_props(dc, nrf5x_ppi_properties);
    dc->vmsd = &nrf5x_ppi_vmstate;
}

static const TypeInfo nrf5x_ppi_info = {
        .name = TYPE_NRF_PPI,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF5PPIState),
        .instance_init = nrf5x_ppi_init,
        .class_init = nrf5x_ppi_class_init
};

static void nrf5x_ppi_register_types(void)
{
    type_register_static(&nrf5x_ppi_info);
}

type_init(nrf5x_ppi_register_types)

