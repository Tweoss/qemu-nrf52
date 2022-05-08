
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

#include "hw/arm/nrf52832_soc.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"
#include "hw/ssi/ssi.h"

struct nrf52832DKMachineState {
    MachineState parent;

    NRF52832State nrf52832;
};

#define TYPE_NRF52832DK_MACHINE MACHINE_TYPE_NAME("nrf52832DK")

OBJECT_DECLARE_SIMPLE_TYPE(nrf52832DKMachineState, NRF52832DK_MACHINE)

static void nrf52832DK_init(MachineState *machine)
{
    nrf52832DKMachineState *s = NRF52832DK_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();

    object_initialize_child(OBJECT(machine), "nrf52832", &s->nrf52832,
                            TYPE_NRF52832_SOC);
    qdev_prop_set_chr(DEVICE(&s->nrf52832), "serial0", serial_hd(0));
    object_property_set_link(OBJECT(&s->nrf52832), "memory",
                             OBJECT(system_memory), &error_fatal);
    sysbus_realize(SYS_BUS_DEVICE(&s->nrf52832), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       s->nrf52832.flash_size);

    // SD card
    {
        void *bus;
        DeviceState *carddev;

        bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim0_twim0), "ssi");
        assert(bus);
        carddev = ssi_create_peripheral(bus, "ssi-sd");

        // Connect CS
        qemu_irq line = qdev_get_gpio_in_named(carddev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out_named(DEVICE(&s->nrf52832.spim0_twim0), "cs_lines", 0, line);
    }
}

static void nrf52832DK_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "nrf52832 Dev. Kit";
    mc->init = nrf52832DK_init;
    mc->max_cpus = 1;
}

static const TypeInfo nrf52832DK_info = {
        .name = TYPE_NRF52832DK_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(nrf52832DKMachineState),
        .class_init = nrf52832DK_machine_class_init,
};

static void nrf52832DK_machine_init(void)
{
    type_register_static(&nrf52832DK_info);
}

type_init(nrf52832DK_machine_init);

/**
 *
 * ../configure --target-list=arm-softmmu
 *
 * ./qemu-system-arm -M nrf52832DK -device loader,file=../../fw/PowerMeter.elf -nographic
 *
 */
