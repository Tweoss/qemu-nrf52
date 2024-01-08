
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/arm/boot.h"
#include "hw/qdev-clock.h"
#include "sysemu/sysemu.h"
#include "exec/address-spaces.h"

#include "hw/arm/nrf51422_soc.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"
#include "hw/ssi/ssi.h"
#include "hw/i2c/i2c.h"

struct nrf51422DKMachineState {
    MachineState parent;

    NRF51422State nrf51422;
};

#define TYPE_NRF51422DK_MACHINE MACHINE_TYPE_NAME("nrf51422DK")

OBJECT_DECLARE_SIMPLE_TYPE(nrf51422DKMachineState, NRF51422DK_MACHINE)

static void nrf51422DK_init(MachineState *machine)
{
    Clock *sysclk;
    nrf51422DKMachineState *s = NRF51422DK_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();

    /* This clock doesn't need migration because it is fixed-frequency */
    sysclk = clock_new(OBJECT(machine), "SYSCLK");
    clock_set_hz(sysclk, 16000000ULL); /* Main SYSCLK frequency in Hz */

    object_initialize_child(OBJECT(machine), "nrf51422", &s->nrf51422,
                            TYPE_NRF51422_SOC);
    qdev_prop_set_chr(DEVICE(&s->nrf51422), "serial0", serial_hd(0));
    qdev_prop_set_chr(DEVICE(&s->nrf51422), "serial1", serial_hd(1));
    object_property_set_link(OBJECT(&s->nrf51422), "memory",
                             OBJECT(system_memory), &error_fatal);
    qdev_connect_clock_in(DEVICE(&s->nrf51422), "sysclk", sysclk);
    sysbus_realize(SYS_BUS_DEVICE(&s->nrf51422), &error_fatal);

    armv7m_load_kernel(ARM_CPU(first_cpu), machine->kernel_filename,
                       s->nrf51422.flash_size);

}

static void nrf51422DK_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "nrf51422 Dev. Kit";
    mc->init = nrf51422DK_init;
    mc->max_cpus = 1;
}

static const TypeInfo nrf51422DK_info = {
        .name = TYPE_NRF51422DK_MACHINE,
        .parent = TYPE_MACHINE,
        .instance_size = sizeof(nrf51422DKMachineState),
        .class_init = nrf51422DK_machine_class_init,
};

static void nrf51422DK_machine_init(void)
{
    type_register_static(&nrf51422DK_info);
}

type_init(nrf51422DK_machine_init);

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
 * ./qemu-system-arm -M nrf51422DK -device loader,file=../fw/PowerMeter.elf -nographic -drive file=../sd0.img,id=mycard,format=qcow2,if=none -device sd-card,spi=true,spec_version=3,drive=mycard -s -S
 *
 * ./qemu-system-arm -M nrf51422DK -device loader,file=/mnt/c/Nordic/Projects/SITF/Aeropod/cmake-build-debug/Aeropod.elf -nographic -s -S
 *
 */
