#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "hw/gpio/nrf52_gpiote.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "trace.h"

#define _TYPE_NAME TYPE_NRF52_GPIOTE

#define MODULE_OBJ_NAME NRF52GPIOTEState
#include "hw/dma/nrf5x_ppi.h"


static uint64_t nrf52_gpiote_read(void *opaque, hwaddr addr, unsigned int size)
{
    NRF52GPIOTEState *s = NRF52_GPIOTE(opaque);
    uint64_t r = 0;

    switch (addr) {

        case A_GPIOTE_INTENCLR:
        case A_GPIOTE_INTENSET:
            r = s->regs[R_GPIOTE_INTEN];
            break;


        default:
            r = s->regs[addr / 4];
            break;
    }

    //info_report("nrf52.gpiote: _read %08lX = %lu", addr, r);

    qemu_set_irq(s->irq, false);

    return r;
}

static void nrf52_gpiote_write(void *opaque, hwaddr addr,
                             uint64_t value, unsigned int size)
{
    NRF52GPIOTEState *s = NRF52_GPIOTE(opaque);

    //info_report("nrf52.gpiote: _write %08lX %lu", addr, value);

    switch (addr) {

        case A_GPIOTE_INTENSET:
            s->regs[R_GPIOTE_INTEN] |= value;
            break;
        case A_GPIOTE_INTENCLR:
            s->regs[R_GPIOTE_INTEN] &= ~value;
            break;

        default:
            s->regs[addr / 4] = value;
            break;
    }

    qemu_set_irq(s->irq, false);

}

static const MemoryRegionOps gpio_ops = {
        .read =  nrf52_gpiote_read,
        .write = nrf52_gpiote_write,
        .endianness = DEVICE_LITTLE_ENDIAN,
        .impl.min_access_size = 4,
        .impl.max_access_size = 4,
};

static void nrf52_gpiote_set(void *opaque, int line, int value)
{
    NRF52GPIOTEState *s = NRF52_GPIOTE(opaque);

    //info_report("nrf52.gpiote: nrf52_gpiote_set %d %d", line, value);

    assert(line >= 0 && line < NRF52_GPIOTE_PINS);

    bool irq_level = false;

    for (int i=0; i< NRF52_GPIOTE_CFG_NB; i++) {

        if (s->regs[R_GPIOTE_INTEN] & (1 << i)) {
            uint32_t mode = (s->regs[R_GPIOTE_CONFIG_0+i] & R_GPIOTE_CONFIG_0_MODE_MASK)
                    >> R_GPIOTE_CONFIG_0_MODE_SHIFT;
            uint32_t pin = (s->regs[R_GPIOTE_CONFIG_0+i] & R_GPIOTE_CONFIG_0_PIN_MASK)
                    >> R_GPIOTE_CONFIG_0_PIN_SHIFT;
            uint32_t pol = (s->regs[R_GPIOTE_CONFIG_0+i] & R_GPIOTE_CONFIG_0_POL_MASK)
                    >> R_GPIOTE_CONFIG_0_POL_SHIFT;

            // info_report("nrf52.gpiote: mode: %u pin: %u pol %u", mode, pin, pol);

            if (pin == line && mode == 1) { // event mode
                if ((pol & 0b01) && value) { // LowToHigh
                    irq_level = true;
                }
                if ((pol & 0b10) && !value) { // HighToLow
                    irq_level = true;
                }
                // info_report("nrf52.gpiote INT mode: %u pin: %u pol %u (val %d)", mode, pin, pol, value);
                if (irq_level) {
                    s->regs[R_GPIOTE_EVENTS_IN_0+i] = 1;
                    PPI_EVENT_R(s, R_GPIOTE_EVENTS_IN_0+i);
                }
                qemu_set_irq(s->irq, irq_level);
            }
        }
    }

}

static void nrf52_gpiote_reset(DeviceState *dev)
{
//    NRF52GPIOTEState *s = NRF52_GPIOTE(dev);
}

static void nrf52_gpiote_init(Object *obj)
{
//    NRF52GPIOTEState *s = NRF52_GPIOTE(dev);
}

static void nrf52832_gpiote_realize(DeviceState *dev, Error **errp)
{
    NRF52GPIOTEState *s = NRF52_GPIOTE(dev);

    memory_region_init_io(&s->iomem, OBJECT(dev), &gpio_ops, s,
                          TYPE_NRF52_GPIOTE, NRF52832_GPIOTE_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->iomem);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    qdev_init_gpio_in_named(DEVICE(s), nrf52_gpiote_set, "gpiote", NRF52_GPIOTE_PINS);

    if (s->downstream) {
        address_space_init(&s->downstream_as, s->downstream, _TYPE_NAME"-downstream");
    }
}

static Property nrf52832_gpiote_properties[] = {
        DEFINE_PROP_LINK("downstream", NRF52GPIOTEState, downstream,
                         TYPE_MEMORY_REGION, MemoryRegion *),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf52_gpiote_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf52_gpiote_reset;
    dc->realize = nrf52832_gpiote_realize;
    device_class_set_props(dc, nrf52832_gpiote_properties);
}

static const TypeInfo nrf52_gpiote_info = {
        .name = TYPE_NRF52_GPIOTE,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF52GPIOTEState),
        .instance_init = nrf52_gpiote_init,
        .class_init = nrf52_gpiote_class_init
};

static void nrf52_gpiote_register_types(void)
{
    type_register_static(&nrf52_gpiote_info);
}

type_init(nrf52_gpiote_register_types)
