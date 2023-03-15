//
// Created by vgol on 06/05/2022.
//

#ifndef QEMU_NRF52_NRF52832_SOC_H
#define QEMU_NRF52_NRF52832_SOC_H

#include "hw/sysbus.h"
#include "hw/arm/armv7m.h"
#include "hw/char/nrf51_uart.h"
#include "hw/misc/nrf51_rng.h"
#include "hw/gpio/nrf51_gpio.h"
#include "hw/gpio/nrf52_gpiote.h"
#include "hw/nvram/nrf51_nvm.h"
#include "hw/timer/nrf51_timer.h"
#include "hw/timer/nrf_clock.h"
#include "hw/clock.h"
#include "qom/object.h"
#include "hw/dma/nrf52_edma.h"
#include "hw/timer/nrf_rtc.h"

#define TYPE_NRF52832_SOC "nrf52832-soc"
OBJECT_DECLARE_SIMPLE_TYPE(NRF52832State, NRF52832_SOC)

#define NRF_GPIO_PIN_MAP(port, pin) (((port) << 5) | ((pin) & 0x1F))

#define NRF52832_NUM_TIMERS 3

struct NRF52832State {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    ARMv7MState armv7m;

    MemoryRegion sram;
    MemoryRegion flash;

    uint32_t sram_size;
    uint32_t flash_size;

    MemoryRegion *board_memory;

    MemoryRegion container;

    MemoryRegion dwt;

    NRF51UARTState  uart;
    NRF51RNGState   rng;
    NRF51NVMState   nvm;
    NRF51GPIOState  gpio;

    NRF52GPIOTEState  gpiote;

    NRF52CLOCKState clock;

    EDMAState spim0_twim0;
    EDMAState spim1_twim1;
    EDMAState spim2;

    NRF51TimerState timer[NRF52832_NUM_TIMERS];

    NRF5RtcState rtc0;
    NRF5RtcState rtc1;
    NRF5RtcState rtc2;

    Clock *sysclk;
    Clock *refclk;
};

#endif //QEMU_NRF52_NRF52832_SOC_H
