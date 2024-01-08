
#include "qemu/osdep.h"
#include "qapi/error.h"
#include "hw/arm/boot.h"
#include "hw/sysbus.h"
#include "hw/qdev-clock.h"
#include "hw/misc/unimp.h"
#include "qemu/log.h"

#include "hw/arm/nrf51422_soc.h"
#include "exec/address-spaces.h"
#include "sysemu/blockdev.h"
#include "hw/ssi/ssi.h"
#include "sysemu/sysemu.h"
#include "hw/qdev-properties-system.h"

#define NRF51422_PERIPHERAL_SIZE 0x00001000

#define NRF51422_FLASH_BASE      0x00000000
#define NRF51422_FICR_BASE       0x10000000
#define NRF51422_UICR_BASE       0x10001000
#define NRF51422_SRAM_BASE       0x20000000

#define NRF51422_IOMEM_BASE      0x40000000
#define NRF51422_IOMEM_SIZE      0x20000000

#define NRF51422_CLOCK_BASE      0x40000000

#define NRF51422_RADIO_BASE      0x40001000

#define NRF51422_UART0_BASE      0x40002000

#define NRF51422_BOX0_BASE       0x40003000
#define NRF51422_BOX1_BASE       0x40004000

#define NRF51422_GPIOTE_BASE     0x40006000

#define NRF51422_SAADC_BASE      0x40007000

#define NRF51422_TIMER0_BASE     0x40008000
#define NRF51422_TIMER1_BASE     0x40009000
#define NRF51422_TIMER2_BASE     0x4000A000

static const uint32_t timer__addr[NRF51422_NUM_TIMERS] = {
    NRF51422_TIMER0_BASE,
    NRF51422_TIMER1_BASE,
    NRF51422_TIMER2_BASE
};

#define NRF51422_RTC0_BASE       0x4000B000

#define NRF51422_RNG_BASE        0x4000D000

#define NRF51422_WDT_BASE        0x40010000

#define NRF51422_RTC1_BASE       0x40011000

#define NRF51422_COMP_BASE       0x40013000

#define NRF51422_NVMC_BASE       0x4001E000

#define NRF51422_GPIO_BASE       0x50000000

#define NRF51422_PRIVATE_BASE    0xF0000000
#define NRF51422_PRIVATE_SIZE    0x10000000

#define NRF51_GPIOTE_CFG_NB 4

#define NRF51_GPIOTE_PINS 32

/* Trigger */
#define NRF51422_TRIGGER_TASK 0x01

/* Events */
#define NRF51422_EVENT_CLEAR  0x00
/*
 * The size and base is for the NRF51422 part. If other parts
 * are supported in the future, add a sub-class of NRF51SoC for
 * the specific variants
 */

#define NRF51422_PAGE_SIZE       NRF51_PAGE_SIZE

#define NRF51422_FLASH_PAGES    256u
#define NRF51422_SRAM_PAGES     32u
#define NRF51422_FLASH_SIZE     (NRF51422_FLASH_PAGES * NRF51422_PAGE_SIZE)
#define NRF51422_SRAM_SIZE      (NRF51422_SRAM_PAGES * NRF51422_PAGE_SIZE)

#define BASE_TO_IRQ(base)       ((base >> 12) & 0x3F)

static uint64_t _rtt_read(void *opaque,
                          hwaddr addr,
                          unsigned size) {

    NRF51422State *s = (NRF51422State *)opaque;

    static uint64_t ccycnt = 0;

    uint64_t val = 0;

    // info_report("RTT access1: offset 0x%x", (uint32_t)addr);

    switch (addr) {
        case 0x000: // buffer down length
            val = s->rtt_buffer_down_len;
            break;
        case 0x004: // buffer data FIFO
            val = s->rtt_buffer_down[ccycnt++];
            if (ccycnt >= sizeof(s->rtt_buffer_down)) {
                ccycnt = 0;
                s->rtt_buffer_down_len = 0;
            }
            break;
        default:
            break;
    }

    return val;
}

static void _rtt_write(void *opaque,
                       hwaddr addr,
                       uint64_t data,
                       unsigned size) {

    NRF51422State *s = (NRF51422State *)opaque;

    // info_report("RTT access: offset 0x%x", (uint32_t)addr);

    if (addr == 0) {
        uint8_t p_data[] = { (uint8_t)data };
        qemu_chr_fe_write(&s->rtt_chr, p_data, 1);
    } else if (addr == 4) {
        s->rtt_buffer_up[s->rtt_buffer_up_len++] = (uint8_t)data;
        if (s->rtt_buffer_up_len >= sizeof(s->rtt_buffer_up)) {
            s->rtt_buffer_up_len = 0;
        }
    } else if (addr == 8) {
        // info_report("Sending %u RTT bytes", s->rtt_buffer_up_len);
        qemu_chr_fe_write(&s->rtt_chr, s->rtt_buffer_up, s->rtt_buffer_up_len);
        s->rtt_buffer_up_len = 0;
    } else {
        info_report("PRIVATE access: offset 0x%x", (uint32_t)addr);
    }

    return;
}


static void _rtt_receive(void *opaque, const uint8_t *buf, int size)
{

    NRF51422State *s = (NRF51422State *)opaque;

    // info_report("Received %u bytes", size);

    if (size > sizeof(s->rtt_buffer_down)) {
        size = sizeof(s->rtt_buffer_down);
    }

    memcpy(s->rtt_buffer_down, buf, size);
    s->rtt_buffer_down_len = size;

    // qemu_chr_fe_accept_input(&s->chr);
}

static int _rtt_can_receive(void *opaque)
{
    NRF51422State *s = (NRF51422State *)opaque;

    return s->rtt_buffer_down_len == 0 ? sizeof(s->rtt_buffer_down) : 0;
}

static const MemoryRegionOps rtt_ops = {
        .read = _rtt_read,
        .write = _rtt_write,
        .endianness = DEVICE_NATIVE_ENDIAN,
        .valid.min_access_size = 1,
        .valid.max_access_size = 8,
};

static void nrf51422_soc_realize(DeviceState *dev_soc, Error **errp)
{
    NRF51422State *s = NRF51422_SOC(dev_soc);
    MemoryRegion *mr;
    uint8_t i = 0;
    hwaddr base_addr = 0;

    if (!s->board_memory) {
        error_setg(errp, "memory property was not set");
        return;
    }

    /*
     * HCLK on this SoC is fixed, so we set up sysclk ourselves and
     * the board shouldn't connect it.
     */
    if (clock_has_source(s->refclk)) {
        error_setg(errp, "refclk clock must not be wired up by the board code");
        return;
    }
    if (!clock_has_source(s->sysclk)) {
        error_setg(errp, "sysclk clock must be wired up by the board code");
        return;
    }

    clock_set_mul_div(s->refclk, 1, 1);
    clock_set_source(s->refclk, s->sysclk);

    qdev_connect_clock_in(DEVICE(&s->armv7m), "cpuclk", s->sysclk);
    qdev_connect_clock_in(DEVICE(&s->armv7m), "refclk", s->refclk);

    object_property_set_link(OBJECT(&s->armv7m), "memory", OBJECT(&s->container),
                             &error_abort);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->armv7m), errp)) {
        return;
    }

    memory_region_add_subregion_overlap(&s->container, 0, s->board_memory, -1);

    memory_region_init_ram(&s->sram, OBJECT(s), "sram", s->sram_size,
                           &error_fatal);

    memory_region_add_subregion(&s->container, NRF51422_SRAM_BASE, &s->sram);

    /* UART */
    object_property_set_link(OBJECT(&s->uart), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->uart), errp)) {
        return;
    }
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->uart), 0);
    memory_region_add_subregion_overlap(&s->container, NRF51422_UART0_BASE, mr, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->uart), 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        BASE_TO_IRQ(NRF51422_UART0_BASE)));

    /* TWIM0 / SPIM0 */
    create_unimplemented_device("nrf51422_soc.box0",
                                NRF51422_BOX0_BASE, NRF51422_PERIPHERAL_SIZE); // TODO

    /* TWIM1 / SPIM1 */
    create_unimplemented_device("nrf51422_soc.box1",
                                NRF51422_BOX1_BASE, NRF51422_PERIPHERAL_SIZE); // TODO

    /* RNG */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rng), errp)) {
        return;
    }
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->rng), 0);
    memory_region_add_subregion_overlap(&s->container, NRF51422_RNG_BASE, mr, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rng), 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        BASE_TO_IRQ(NRF51422_RNG_BASE)));

    /* UICR, FICR, NVMC, FLASH */
    if (!object_property_set_uint(OBJECT(&s->nvm), "flash-size",
                                  s->flash_size, errp)) {
        return;
    }

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->nvm), errp)) {
        return;
    }

    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 0);
    memory_region_add_subregion_overlap(&s->container, NRF51422_NVMC_BASE, mr, 0);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 1);
    memory_region_add_subregion_overlap(&s->container, NRF51422_FICR_BASE, mr, 0);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 2);
    memory_region_add_subregion_overlap(&s->container, NRF51422_UICR_BASE, mr, 0);
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->nvm), 3);
    memory_region_add_subregion_overlap(&s->container, NRF51422_FLASH_BASE, mr, 0);

    /* GPIO */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->gpio), 0);
    memory_region_add_subregion_overlap(&s->container, NRF51422_GPIO_BASE, mr, 0);
    /* Pass all GPIOs to the SOC layer so they are available to the board */
    qdev_pass_gpios(DEVICE(&s->gpio), dev_soc, NULL);

    /* GPIOTE */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpiote), errp)) {
        return;
    }
    mr = sysbus_mmio_get_region(SYS_BUS_DEVICE(&s->gpiote), 0);
    memory_region_add_subregion_overlap(&s->container, NRF51422_GPIOTE_BASE, mr, 0);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpiote), 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        BASE_TO_IRQ(NRF51422_GPIOTE_BASE)));
    /* Connect GPIO interrupts to the GPIOTE */
    for (int i = 0; i < NRF51_GPIOTE_PINS; i++) {
        qemu_irq gpiote_line = qdev_get_gpio_in_named(DEVICE(&s->gpiote), "gpiote", i);
        qdev_connect_gpio_out_named(DEVICE(&s->gpio), "gpiote", i, gpiote_line);
    }

    /* TIMER */
    for (i = 0; i < NRF51422_NUM_TIMERS; i++) {
        object_property_set_link(OBJECT(&s->timer[i]), "downstream", OBJECT(&s->container),
                                 &error_fatal);
        if (!object_property_set_uint(OBJECT(&s->timer[i]), "id", i, errp)) {
            return;
        }
        if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer[i]), errp)) {
            return;
        }

        base_addr = timer__addr[i];

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer[i]), 0, base_addr);
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->timer[i]), 0,
                           qdev_get_gpio_in(DEVICE(&s->armv7m),
                                            BASE_TO_IRQ(base_addr)));
    }

    /* RTC0 */
    object_property_set_link(OBJECT(&s->rtc0), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc0), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc0), 0, NRF51422_RTC0_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc0), 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        BASE_TO_IRQ(NRF51422_RTC0_BASE)));

    /* RTC1 */
    object_property_set_link(OBJECT(&s->rtc1), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->rtc1), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->rtc1), 0, NRF51422_RTC1_BASE);
    sysbus_connect_irq(SYS_BUS_DEVICE(&s->rtc1), 0,
                       qdev_get_gpio_in(DEVICE(&s->armv7m),
                                        BASE_TO_IRQ(NRF51422_RTC1_BASE)));

    /* PPI */
    object_property_set_link(OBJECT(&s->ppi), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->ppi), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->ppi), 0, NRFxxxxx_PPI_BASE);

    /* CLOCK */
    object_property_set_link(OBJECT(&s->clock), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->clock), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->clock), 0, NRF51422_CLOCK_BASE);

    /* Radio */
    object_property_set_link(OBJECT(&s->radio), "downstream", OBJECT(&s->container),
                             &error_fatal);
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->radio), errp)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->radio), 0, NRF51422_RADIO_BASE);

    // Debug output
    memory_region_init_io(&s->rtt, NULL, &rtt_ops, s, "nrf51422_soc.rtt", 0x100);
    memory_region_add_subregion(&s->armv7m.container,  NRF51422_PRIVATE_BASE, &s->rtt);

    /* Other Peripherals */

    create_unimplemented_device("nrf51422_soc.io",
                                NRF51422_IOMEM_BASE, NRF51422_IOMEM_SIZE);
    create_unimplemented_device("nrf51422_soc.private",
                                NRF51422_PRIVATE_BASE, NRF51422_PRIVATE_SIZE);

    /* STUB Peripherals */

    create_unimplemented_device("nrf51422_soc.saadc",
                                NRF51422_SAADC_BASE, NRF51422_PERIPHERAL_SIZE);
    create_unimplemented_device("nrf51422_soc.wdt",
                                NRF51422_WDT_BASE, NRF51422_PERIPHERAL_SIZE);

    create_unimplemented_device("nrf51422_soc.comp",
                                NRF51422_COMP_BASE, NRF51422_PERIPHERAL_SIZE);


}

static void nrf51422_soc_init(Object *obj)
{
    uint8_t i = 0;

    NRF51422State *s = NRF51422_SOC(obj);

    memory_region_init(&s->container, obj, "nrf51422-container", UINT64_MAX);

    object_initialize_child(OBJECT(s), "armv6m", &s->armv7m, TYPE_ARMV7M);
    qdev_prop_set_string(DEVICE(&s->armv7m), "cpu-type", ARM_CPU_TYPE_NAME("cortex-m0"));
    qdev_prop_set_uint32(DEVICE(&s->armv7m), "num-irq", 32);

    object_initialize_child(obj, "uart", &s->uart, TYPE_NRF51_UART);
    object_property_add_alias(obj, "serial1", OBJECT(&s->uart), "chardev");

    object_initialize_child(obj, "rng", &s->rng, TYPE_NRF51_RNG);

    object_initialize_child(obj, "nvm", &s->nvm, TYPE_NRF51_NVM);

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_NRF51_GPIO);

    object_initialize_child(obj, "gpiote", &s->gpiote, TYPE_NRF52_GPIOTE);

    object_initialize_child(obj, "clock", &s->clock, TYPE_NRF_CLOCK);

    for (i = 0; i < NRF51422_NUM_TIMERS; i++) {
        object_initialize_child(obj, "timer[*]", &s->timer[i],
                                TYPE_NRF51_TIMER);

    }

    object_initialize_child(obj, "rtc0", &s->rtc0, TYPE_NRF_RTC);
    object_initialize_child(obj, "rtc1", &s->rtc1, TYPE_NRF_RTC);

    object_initialize_child(obj, "ppi", &s->ppi, TYPE_NRF_PPI);

    object_initialize_child(obj, "radio", &s->radio, TYPE_NRF_RADIO);

    object_property_add_alias(obj, "serial0", OBJECT(s), "rtt_chardev");

    s->sysclk = qdev_init_clock_in(DEVICE(s), "sysclk", NULL, NULL, 0);
    s->refclk = qdev_init_clock_in(DEVICE(s), "refclk", NULL, NULL, 0);

    s->rtt_buffer_down_len = 0;
    s->rtt_buffer_up_len = 0;
}

static Property nrf51422_soc_properties[] = {
        DEFINE_PROP_LINK("memory", NRF51422State, board_memory, TYPE_MEMORY_REGION,
                         MemoryRegion *),
        DEFINE_PROP_UINT32("sram-size", NRF51422State, sram_size, NRF51422_SRAM_SIZE),
        DEFINE_PROP_UINT32("flash-size", NRF51422State, flash_size,
                           NRF51422_FLASH_SIZE),
        DEFINE_PROP_CHR("rtt_chardev", NRF51422State, rtt_chr),
        DEFINE_PROP_END_OF_LIST(),
};

static void nrf51422_device_reset(DeviceState *dev) {

    NRF51422State *s = NRF51422_SOC(dev);

    qemu_chr_fe_set_open(&s->rtt_chr, 1);

    qemu_chr_fe_set_handlers(&s->rtt_chr, _rtt_can_receive,
                             _rtt_receive, NULL, NULL,
                             s, NULL, true);

//    qemu_chr_fe_add_watch(&s->rtt_chr, G_IO_OUT | G_IO_HUP,
//                          uart_transmit, s);
}

static void nrf51422_soc_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf51422_device_reset;
    dc->realize = nrf51422_soc_realize;
    device_class_set_props(dc, nrf51422_soc_properties);
}

static const TypeInfo nrf51422_soc_info = {
        .name          = TYPE_NRF51422_SOC,
        .parent        = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(NRF51422State),
        .instance_init = nrf51422_soc_init,
        .class_init    = nrf51422_soc_class_init,
};

static void nrf51422_soc_types(void)
{
    type_register_static(&nrf51422_soc_info);
}
type_init(nrf51422_soc_types)
