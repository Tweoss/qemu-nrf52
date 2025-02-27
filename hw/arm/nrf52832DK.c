
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "hw/qdev-clock.h"
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

#define LIS12_DRDY               NRF_GPIO_PIN_MAP(0, 28)

#define BMP390_DRDY1             NRF_GPIO_PIN_MAP(0, 11)
#define BMP390_DRDY2             NRF_GPIO_PIN_MAP(0, 12)

struct nrf52832DKMachineState {
    MachineState parent;

    NRF52832State nrf52832;
};

/* Main SYSCLK frequency in Hz (64MHz) */
#define SYSCLK_FRQ 64000000ULL

#define TYPE_NRF52832DK_MACHINE MACHINE_TYPE_NAME("nrf52832DK")

OBJECT_DECLARE_SIMPLE_TYPE(nrf52832DKMachineState, NRF52832DK_MACHINE)

static void nrf52832DK_init(MachineState *machine)
{
    Clock *sysclk;
    nrf52832DKMachineState *s = NRF52832DK_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();

    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, SYSCLK_FRQ);

    object_initialize_child(OBJECT(machine), "nrf52832", &s->nrf52832,
                            TYPE_NRF52832_SOC);
    qdev_prop_set_chr(DEVICE(&s->nrf52832), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(&s->nrf52832), "serial1", serial_hd(1));
    object_property_set_link(OBJECT(&s->nrf52832), "memory",
                             OBJECT(system_memory), &error_fatal);
    qdev_connect_clock_in(DEVICE(&s->nrf52832), "sysclk", sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->nrf52832), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       s->nrf52832.flash_size);

    DeviceState *model = qdev_new(TYPE_ZMODEL);

#if 0
    // SD card
    {
        void *ssi_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim0_twim0), "ssi");
        assert(ssi_bus);
        DeviceState *carddev = ssi_create_peripheral(ssi_bus, "ssi-sd");

        // Connect CS
        qemu_irq line = qdev_get_gpio_in_named(carddev, SSI_GPIO_CS, 0);
        qdev_connect_gpio_out_named(DEVICE(&s->nrf52832.spim0_twim0), "cs_lines", 0, line);
    }

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
#endif

    /* add a first BMP390 pressure sensor */
    {
        void *i2c_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim1_twim1), "i2c");
        assert(i2c_bus);
        I2CSlave *dev = i2c_slave_new("bmp390", 0x77);
        // connect Z model
        object_property_set_link(OBJECT(&dev->qdev), "model",
                                 OBJECT(model), &error_fatal);
        // init it
        i2c_slave_realize_and_unref(dev, i2c_bus, &error_abort);
        // connect DRDY
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), BMP390_DRDY1);
        qdev_connect_gpio_out_named(&dev->qdev, "DRDY", 0, int1_line);
    }

    /* add a second BMP390 pressure sensor */
    {
        void *i2c_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim1_twim1), "i2c");
        assert(i2c_bus);
        I2CSlave *dev = i2c_slave_new("bmp390", 0x76);
        // connect Z model
        object_property_set_link(OBJECT(&dev->qdev), "model",
                                 OBJECT(model), &error_fatal);
        // init it
        i2c_slave_realize_and_unref(dev, i2c_bus, &error_abort);
        // connect DRDY
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), BMP390_DRDY2);
        qdev_connect_gpio_out_named(&dev->qdev, "DRDY", 0, int1_line);
    }

    /* add a LIS12 pressure sensor */
    {
        void *i2c_bus = qdev_get_child_bus(DEVICE(&s->nrf52832.spim0_twim0), "i2c");
        assert(i2c_bus);
        I2CSlave *dev = i2c_slave_new("lis2dw12", 0b0011000);
        // connect Z model
        object_property_set_link(OBJECT(&dev->qdev), "model",
                                 OBJECT(model), &error_fatal);
        // init it
        i2c_slave_realize_and_unref(dev, i2c_bus, &error_abort);
        // connect DRDY
        qemu_irq int1_line = qdev_get_gpio_in(DEVICE(&s->nrf52832), LIS12_DRDY);
        qdev_connect_gpio_out_named(&dev->qdev, "DRDY", 0, int1_line);
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
 * mkdir build && cd build/
 *
 * ../configure --target-list=arm-softmmu --disable-werror
 *
 * In order to use a .img SD card image, you can first format it on the host (https://www.linux.org/threads/qemu.10727/)
 * mkfs.vfat -S 512 -F 32 ../sd0.img
 *
 * ./build/qemu-img create -f raw ../sd0.img 4G
 *
 * ./build/qemu-img convert -f raw -O qcow2 sd0.img sd1.img
 *
 * ./qemu-system-arm -M nrf52832DK -device loader,file=../fw/PowerMeter.elf -nographic -drive file=../sd0.img,id=mycard,format=qcow2,if=none -device sd-card,spi=true,spec_version=3,drive=mycard -s -S
 *
 * ./qemu-system-arm -M nrf52832DK -device loader,file=/mnt/c/Nordic/Projects/SITF/Aeropod/cmake-build-debug/Aeropod.elf -nographic -s -S
 *
 * ///////////////////////////////////////////////////////////////////////////////////////////////////
 *
 *  ==>>  serial_hd(N)
 *
 *  -chardev serial,mux=on,path=//./COM25,id=s0 -serial chardev:s0 -serial mon:telnet::4444,server=on,wait=off
 *
 *  -serial chardev:s0 -chardev serial,path=//./COM20,id=s0
 *
 *  -chardev serial,path=//./COM25,id=s0 -serial chardev:s0
 *
 *  -chardev socket,id=s0,host=127.0.0.1,port=1235 -serial chardev:s0
 *
 * ///////////////////////////////////////////////////////////////////////////////////////////////////
 *
 * OS: Microsoft Windows 10 Home 64-bit
 *
 * https://stackoverflow.com/questions/53084815/compile-qemu-under-windows-10-64-bit-for-windows-10-64-bit
 *
 * - Download and install msys2 to C:\msys64: http://repo.msys2.org/distrib/x86_64/msys2-x86_64-20180531.exe
 * - Start C:\msys64\mingw64.exe
 * - Updates (then close window and restart mingw64.exe): pacman -Syu
 * - Updates: pacman -Su
 * - Install basic packets: pacman -S base-devel mingw-w64-x86_64-toolchain git python
 * - Install QEMU specific packets: pacman -S mingw-w64-x86_64-glib2 mingw-w64-x86_64-gtk3 mingw-w64-x86_64-SDL2
 * - Get QEMU sources:
 *      - git submodule update --init ui/keycodemapdb
 *      - git submodule update --init capstone
 *      - git submodule update --init dtc
 *      - Comment out (#) Capstone (line 508) in qemu\Makefile or meson_options.txt
 * - Build qemu:
 *      - ../configure --target-list=arm-softmmu --disable-werror --disable-stack-protector
 *      - make -j4 -k
 *
 * - Run qemu with the MSYS2 PATH variable set: PATH=/mingw64/bin:/usr/local/bin:/usr/bin:/bin:/c/Windows/System32:/c/Windows:/c/Windows/System32/Wbem:/c/Windows/System32/WindowsPowerShell/v1.0/:/usr/bin/site_perl:/usr/bin/vendor_perl:/usr/bin/core_perl
 */
