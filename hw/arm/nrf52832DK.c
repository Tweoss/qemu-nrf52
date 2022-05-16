
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
#include "hw/i2c/i2c.h"
#include "hw/arm/z_model.h"

#define MAX_DRDY1             NRF_GPIO_PIN_MAP(0, 3)
#define MAX_DRDY2             NRF_GPIO_PIN_MAP(0, 7)

#define MAX_SS1_PIN           NRF_GPIO_PIN_MAP(0, 31)
#define MAX_SS2_PIN           NRF_GPIO_PIN_MAP(0, 30)

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
        void *ssi_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim0_twim0), "ssi");
        assert(ssi_bus);
        DeviceState *carddev = ssi_create_peripheral(ssi_bus, "ssi-sd");

        // Connect CS
        qemu_irq line = qdev_get_gpio_in_named(carddev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out_named(DEVICE(&s->nrf52832.spim0_twim0), "cs_lines", 0, line);
    }

    DeviceState *model = qdev_new(TYPE_ZMODEL);

    // LSM6 (CS pin SPI-activated)
    {
        void *ssi_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim1_twim1), "ssi");
        assert(ssi_bus);
        DeviceState *slave = qdev_new("ssi-lsm6dsox");
        object_property_set_link(OBJECT(slave), "model",
                                 OBJECT(model), &error_fatal);
        ssi_realize_and_unref(slave, ssi_bus, &error_fatal);

        // connect SPI CS line interrupt
        qemu_irq cs_line = qdev_get_gpio_in_named(slave, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out_named(DEVICE(&s->nrf52832.spim1_twim1), "cs_lines", 0, cs_line);
        // connect INT1
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), NRF_GPIO_PIN_MAP(0, 30));
        qdev_connect_gpio_out_named(slave, "INT1", 0, int1_line);
    }

    // MAX11254 (CS pin user activated)
    {
        void *ssi_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim2), "ssi");
        assert(ssi_bus);
        DeviceState *dev = qdev_new("ssi-max11254");
        qdev_prop_set_uint8(dev, "ID", 0);
        object_property_set_link(OBJECT(dev), "model",
                                 OBJECT(model), &error_fatal);
        ssi_realize_and_unref(dev, ssi_bus, &error_fatal);

        // connect SPI CS line interrupt
        qemu_irq cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out(DEVICE(&s->nrf52832), MAX_SS1_PIN, cs_line);
        // connect DRDY
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), MAX_DRDY1);
        qdev_connect_gpio_out_named(dev, "DRDY", 0, int1_line);
    }

    // MAX11254 (CS pin user activated)
    {
        void *ssi_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim2), "ssi");
        assert(ssi_bus);
        DeviceState *dev = qdev_new("ssi-max11254");
        qdev_prop_set_uint8(dev, "ID", 1);
        object_property_set_link(OBJECT(dev), "model",
                                 OBJECT(model), &error_fatal);
        ssi_realize_and_unref(dev, ssi_bus, &error_fatal);

        // connect SPI CS line interrupt
        qemu_irq cs_line = qdev_get_gpio_in_named(dev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out(DEVICE(&s->nrf52832), MAX_SS2_PIN, cs_line);
        // connect DRDY
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), MAX_DRDY2);
        qdev_connect_gpio_out_named(dev, "DRDY", 0, int1_line);
    }

    /* add a TMP423 temperature sensor */
    {
        void *i2c_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim0_twim0), "i2c");
        assert(i2c_bus);
        i2c_slave_create_simple(i2c_bus, "tmp423", 0x48);
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
 * In order to use a .img SD card image, you can first format it on the host via mkfs.vfat -s 16 -F 32 ./sd0.img (https://www.linux.org/threads/qemu.10727/)
 *
 * ./qemu-img create -f raw ../sd0.img 4G
 *
 * ./qemu-img convert -f raw -O qcow2 sd0.img sd1.img
 *
 * ./qemu-system-arm -M nrf52832DK -device loader,file=../fw/PowerMeter.elf -nographic -drive file=../sd0.img,id=mycard,format=qcow2,if=none -device sd-card,spi=true,spec_version=3,drive=mycard -s -S
 *
 */
