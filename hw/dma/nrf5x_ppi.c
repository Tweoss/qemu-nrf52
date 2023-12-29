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

#define _TYPE_NAME TYPE_NRF_PPI


static void _update_irq(NRF5PPIState *s) {

}

static void _process_event(NRF5PPIState *s, uint32_t value) {

    for (unsigned chan_nb=0; chan_nb < PPI_CHANNEL_NB; chan_nb++) {
        if (s->regs[R_PPI_CHEN] & (1u << chan_nb)) { // channel enabled ?
            if (value == s->regs[R_PPI_CH0_EEP + (chan_nb << 3u)]) { // event match ?

//                info_report(TYPE_NRF_PPI": event match -> 0x%08X", s->regs[R_PPI_CH0_TEP + (chan_nb << 3u)]);

                uint32_t per_add = 1;
                MemTxResult res = address_space_rw(&s->downstream_as,
                                                   s->regs[R_PPI_CH0_TEP + (chan_nb << 3u)],
                                                   MEMTXATTRS_UNSPECIFIED,
                                                   &per_add,
                                                   sizeof(per_add),
                                                   true);

                if (res) {
                    error_report(_TYPE_NAME": error %u", res);
                }

                return;
            }
        }
    }

}

static void _nrf_write(void *opaque,
                       hwaddr addr,
                       uint64_t value,
                       unsigned size) {

    NRF5PPIState *s = NRF_PPI(opaque);

    if (addr != A_PPI_EVENT_IN) {
//        info_report(_TYPE_NAME": _nrf_write %03llX <- %llX", addr, value);

        if (addr <= A_PPI_CHG3_DIS) {

//            uint8_t channel = (addr - A_PPI_CHG0_EN) >> 3u;
//            info_report(_TYPE_NAME": channel group: %u", channel);

        } else
        if (addr >= A_PPI_CH0_EEP && addr <= A_PPI_CH15_TEP) {

//            uint8_t channel = (addr - A_PPI_CH0_EEP) >> 3u;
//            info_report(_TYPE_NAME": channel: %u", channel);

        }
    }

    s->regs[addr >> 2] = value;

    switch (addr) {

        case A_PPI_CHEN:
            s->regs[R_PPI_CHEN] = value;
            break;
        case A_PPI_CHENSET:
            s->regs[R_PPI_CHEN] |= value;
            break;
        case A_PPI_CHENCLR:
            s->regs[R_PPI_CHEN] &= ~value;
            break;

        case A_PPI_EVENT_IN:
//            info_report(_TYPE_NAME": Received event !");
            _process_event(s, value);
            break;

        default:
            break;
    }

    _update_irq(s);
}

static uint64_t _nrf_read(void *opaque,
                          hwaddr addr,
                          unsigned size) {

    uint64_t r;
    NRF5PPIState *s = NRF_PPI(opaque);

//    info_report(_TYPE_NAME": _nrf_read %08llX", addr);

    switch (addr) {

        case A_PPI_CHEN:
        case A_PPI_CHENSET:
        case A_PPI_CHENCLR:
            r = s->regs[R_PPI_CHEN];
            break;

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

    memory_region_init_io(&s->iomem, OBJECT(dev), &edma_ops, s,
                          TYPE_NRF_PPI, NRF_PPI_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);

//    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    if (!s->downstream) {
        error_report("!! 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");

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
        DEFINE_PROP_LINK("downstream", NRF5PPIState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion *),
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

