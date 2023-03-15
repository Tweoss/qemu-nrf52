/*
 * nRF51 SoC UART emulation
 *
 * See nRF51 Series Reference Manual, "29 Universal Asynchronous
 * Receiver/Transmitter" for hardware specifications:
 * http://infocenter.nordicsemi.com/pdf/nRF51_RM_v3.0.pdf
 *
 * Copyright (c) 2018 Julia Suvorova <jusual@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "exec/address-spaces.h"
#include "hw/char/nrf51_uart.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "trace.h"

static void nrf51_uart_update_irq(NRF51UARTState *s)
{
    bool irq = false;

    irq |= (s->reg[R_UART_RXDRDY] &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_RXDRDY_MASK));
    irq |= (s->reg[R_UART_ENDRX]   &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_ENDRX_MASK));
    irq |= (s->reg[R_UART_TXDRDY] &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_TXDRDY_MASK));
    irq |= (s->reg[R_UART_ENDTX]   &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_ENDTX_MASK));
    irq |= (s->reg[R_UART_ERROR]  &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_ERROR_MASK));
    irq |= (s->reg[R_UART_RXTO]   &&
            (s->reg[R_UART_INTEN] & R_UART_INTEN_RXTO_MASK));

    if (irq) {
        info_report("nrf52832.uart0: irq");
    }

    qemu_set_irq(s->irq, irq);
}

static uint64_t uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    NRF51UARTState *s = NRF51_UART(opaque);
    uint64_t r;

    if (!s->enabled) {
        return 0;
    }

//    info_report("nrf52832.uart0: uart_read %08lX", addr);

    switch (addr) {
    case A_UART_RXD:
        r = s->rx_fifo[s->rx_fifo_pos];
        if (s->reg[R_UART_STARTRX] && s->rx_fifo_len) {
            s->rx_fifo_pos = (s->rx_fifo_pos + 1) % UART_FIFO_LENGTH;
            s->rx_fifo_len--;
            if (s->rx_fifo_len) {
                s->reg[R_UART_RXDRDY] = 1;
                nrf51_uart_update_irq(s);
            }
            qemu_chr_fe_accept_input(&s->chr);
        }
        break;
    case A_UART_INTENSET:
    case A_UART_INTENCLR:
    case A_UART_INTEN:
        r = s->reg[R_UART_INTEN];
        break;
    default:
        r = s->reg[addr / 4];
        break;
    }

    trace_nrf51_uart_read(addr, r, size);

    return r;
}

static gboolean uart_transmit(void *do_not_use, GIOCondition cond, void *opaque)
{
    NRF51UARTState *s = NRF51_UART(opaque);

    s->watch_tag = 0;

    // info_report("nrf52832.uart0: bytes sending: %u", s->reg[R_UART_TXD_MAXCNT]);

    if (s->reg[R_UART_STARTTX] && s->reg[R_UART_TXD_MAXCNT]) { // EDMA way

        MemTxResult result = address_space_rw(&s->downstream_as,
                                              s->reg[R_UART_TXD_PTR],
                                              MEMTXATTRS_UNSPECIFIED,
                                              s->tx_dma,
                                              s->reg[R_UART_TXD_MAXCNT],
                                              false);
        if (!result) {
            (void)qemu_chr_fe_write(&s->chr, s->tx_dma, s->reg[R_UART_TXD_MAXCNT]);
        } else {
            error_report("nrf52832.uart0: address_space_rw error: %d", result);
        }
        goto buffer_drained;

    } else if (s->pending_tx_byte) {

        warn_report("nrf52832.uart0: pending_tx_byte");

        uint8_t c = s->reg[R_UART_TXD];
        int r = qemu_chr_fe_write(&s->chr, &c, 1);

        if (r <= 0) {
            s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                                 uart_transmit, s);
            if (!s->watch_tag) {
                /* The hardware has no transmit error reporting,
                 * so silently drop the byte
                 */
                goto buffer_drained;
            }
            return FALSE;
        }
    }

buffer_drained:
    s->reg[R_UART_TXD_AMOUNT] = s->reg[R_UART_TXD_MAXCNT];
    s->reg[R_UART_TXD_MAXCNT] = 0;
    s->reg[R_UART_ENDTX] = 1;
    s->reg[R_UART_TXDRDY] = 1;
    s->pending_tx_byte = false;
    return FALSE;
}

static void uart_cancel_transmit(NRF51UARTState *s)
{
    if (s->watch_tag) {
        g_source_remove(s->watch_tag);
        s->watch_tag = 0;
    }
}

static void uart_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
    NRF51UARTState *s = NRF51_UART(opaque);

    trace_nrf51_uart_write(addr, value, size);
//    info_report("nrf52832.uart0: uart_write %08lX %lu", addr, value);
    if (!s->enabled && (addr != A_UART_ENABLE)) {
        return;
    }

    switch (addr) {
    case A_UART_TXD:
        if (!s->pending_tx_byte && s->reg[R_UART_TXSTARTED]) {
            s->reg[R_UART_TXD] = value;
            s->pending_tx_byte = true;
            uart_transmit(NULL, G_IO_OUT, s);
        }
        break;
    case A_UART_INTEN:
        s->reg[R_UART_INTEN] = value;
        break;
    case A_UART_INTENSET:
        s->reg[R_UART_INTEN] |= value;
        break;
    case A_UART_INTENCLR:
        s->reg[R_UART_INTEN] &= ~value;
        break;
//    case A_UART_TXDRDY ... A_UART_RXTO:
//        s->reg[addr / 4] = value;
//        break;
    case A_UART_ERRORSRC:
        s->reg[addr / 4] &= ~value;
        break;
    case A_UART_RXD:
        break;
    case A_UART_RXDRDY:
        if (value == 0) {
            s->reg[R_UART_RXDRDY] = 0;
        }
        break;
    case A_UART_STARTTX:
        s->reg[R_UART_STARTTX] = value;
        if (value == 1) {
            s->reg[R_UART_TXSTARTED] = 1;
            uart_transmit(NULL, G_IO_OUT, s); // easy DMA start
        }
        break;
    case A_UART_STARTRX:
        s->reg[R_UART_STARTRX] = value;
        if (value == 1) {
            // info_report("nrf52832.uart0: STARTRX %u", s->reg[R_UART_RXD_MAXCNT]);
            s->reg[R_UART_ENDRX] = 0;
            s->reg[R_UART_RXD_AMOUNT] = 0;
            s->reg[R_UART_RXTO] = 0;
        }
        break;
    case A_UART_ENABLE:
        //info_report("nrf52832.uart0: A_UART_ENABLE %lu", value);
        if (value) {
            s->enabled = true;
            break;
        }
        s->enabled = false;
        value = 1;
        /* fall through */
    case A_UART_SUSPEND:
        if (value == 1) {
            warn_report("nrf52832.uart0: A_UART_SUSPEND not implemented");
        }
        break;
    case A_UART_STOPTX:
        if (value == 1) {
            s->reg[R_UART_TXSTARTED] = 0;
            s->reg[R_UART_TXSTOPPED] = 1;
        }
        break;
        /* fall through */
    case A_UART_STOPRX:
        if (value == 1) {
            s->reg[R_UART_STARTRX] = 0;
        }
        break;
    default:
        s->reg[addr / 4] = value;
        break;
    }
    nrf51_uart_update_irq(s);
}

static const MemoryRegionOps uart_ops = {
    .read =  uart_read,
    .write = uart_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void nrf51_uart_reset(DeviceState *dev)
{
    NRF51UARTState *s = NRF51_UART(dev);

    s->pending_tx_byte = 0;

    uart_cancel_transmit(s);

    memset(s->reg, 0, sizeof(s->reg));

    s->reg[R_UART_PSELRTS] = 0xFFFFFFFF;
    s->reg[R_UART_PSELTXD] = 0xFFFFFFFF;
    s->reg[R_UART_PSELCTS] = 0xFFFFFFFF;
    s->reg[R_UART_PSELRXD] = 0xFFFFFFFF;
    s->reg[R_UART_BAUDRATE] = 0x4000000;

    s->rx_fifo_len = 0;
    s->rx_fifo_pos = 0;

    s->enabled = false;

    qemu_chr_fe_set_open(&s->chr, 1);
}

static void uart_receive(void *opaque, const uint8_t *buf, int size)
{

    NRF51UARTState *s = NRF51_UART(opaque);
    int i;

    info_report("nrf52832.uart0: uart_receive %d (started= %d)", size, s->reg[R_UART_STARTRX]);

    if (size == 0) {
        return;
    }

    for (i = 0; i < size && s->rx_fifo_len < UART_FIFO_LENGTH; i++) {
        uint32_t pos = (s->rx_fifo_pos + s->rx_fifo_len) % UART_FIFO_LENGTH;
        s->rx_fifo[pos] = buf[i];
        s->rx_fifo_len++;
        s->reg[R_UART_RXDRDY] = 1;
    }

    // copy to DMA buffer
    if (s->reg[R_UART_STARTRX]) {

        s->reg[R_UART_RXD_AMOUNT] += size;
        s->reg[R_UART_RXDRDY] = 1;
        if (s->reg[R_UART_RXD_AMOUNT] >= s->reg[R_UART_RXD_MAXCNT]) {
            // warn_report("nrf52832.uart0: uart_trimming %d", s->reg[R_UART_RXD_AMOUNT] - s->reg[R_UART_RXD_MAXCNT]);
            s->reg[R_UART_ENDRX] = 1;
            s->reg[R_UART_RXD_AMOUNT] = s->reg[R_UART_RXD_MAXCNT];
            s->reg[R_UART_STARTRX] = 0;
        }

        (void) address_space_rw(&s->downstream_as,
                                s->reg[R_UART_RXD_PTR],
                                MEMTXATTRS_UNSPECIFIED,
                                (void*)buf,
                                s->reg[R_UART_RXD_MAXCNT],
                                true);

        s->rx_fifo_len = 0;
        s->rx_fifo_pos = 0;
        //s->rx_started = false;

        nrf51_uart_update_irq(s);
    }

    qemu_chr_fe_accept_input(&s->chr);
}

static int uart_can_receive(void *opaque)
{
    NRF51UARTState *s = NRF51_UART(opaque);

    if (!s->enabled) {
        return 0;
    }

    if (s->reg[R_UART_STARTRX] && s->reg[R_UART_RXD_MAXCNT]) {
        return s->reg[R_UART_RXD_MAXCNT];
    }

    return 512; // random to empty the bytes
}

static void uart_event(void *opaque, QEMUChrEvent event)
{
    NRF51UARTState *s = NRF51_UART(opaque);

    // info_report("UARTE0 event %d", event);

    if (event == CHR_EVENT_BREAK) {

        error_report("UARTE0 CHR_EVENT_BREAK");

        s->reg[R_UART_ERRORSRC] |= 3;
        s->reg[R_UART_ERROR] = 1;
        nrf51_uart_update_irq(s);
    }
}

static void nrf51_uart_realize(DeviceState *dev, Error **errp)
{
    NRF51UARTState *s = NRF51_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, uart_can_receive, uart_receive,
                             uart_event, NULL, s, NULL, false);

    if (!s->downstream) {
        error_report("UARTE0 'downstream' link not set");
        return;
    }

    address_space_init(&s->downstream_as, s->downstream, "nrf51_uart-downstream");
}

static void nrf51_uart_init(Object *obj)
{
    NRF51UARTState *s = NRF51_UART(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          "nrf51_soc.uart", UART_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static int nrf51_uart_post_load(void *opaque, int version_id)
{
    NRF51UARTState *s = NRF51_UART(opaque);

    if (s->pending_tx_byte) {
        s->watch_tag = qemu_chr_fe_add_watch(&s->chr, G_IO_OUT | G_IO_HUP,
                                             uart_transmit, s);
    }

    return 0;
}

static const VMStateDescription nrf51_uart_vmstate = {
    .name = "nrf51_soc.uart",
    .post_load = nrf51_uart_post_load,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(reg, NRF51UARTState, NRF52832_UART_PER_SIZE),
        VMSTATE_UINT8_ARRAY(rx_fifo, NRF51UARTState, UART_FIFO_LENGTH),
        VMSTATE_UINT32(rx_fifo_pos, NRF51UARTState),
        VMSTATE_UINT32(rx_fifo_len, NRF51UARTState),
        VMSTATE_BOOL(pending_tx_byte, NRF51UARTState),
        VMSTATE_BOOL(enabled, NRF51UARTState),
        VMSTATE_END_OF_LIST()
    }
};

static Property nrf51_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", NRF51UARTState, chr),
    DEFINE_PROP_LINK("downstream", NRF51UARTState, downstream,
                     TYPE_MEMORY_REGION, MemoryRegion *),
    DEFINE_PROP_END_OF_LIST(),
};

static void nrf51_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = nrf51_uart_reset;
    dc->realize = nrf51_uart_realize;
    device_class_set_props(dc, nrf51_uart_properties);
    dc->vmsd = &nrf51_uart_vmstate;
}

static const TypeInfo nrf51_uart_info = {
    .name = TYPE_NRF51_UART,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NRF51UARTState),
    .instance_init = nrf51_uart_init,
    .class_init = nrf51_uart_class_init
};

static void nrf51_uart_register_types(void)
{
    type_register_static(&nrf51_uart_info);
}

type_init(nrf51_uart_register_types)
