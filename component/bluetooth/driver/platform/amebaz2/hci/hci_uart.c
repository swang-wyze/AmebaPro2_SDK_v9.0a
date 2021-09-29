/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#include <string.h>
#include "osif.h"
#include "hci_uart.h"
#include "hci_dbg.h"
#include "serial_api.h"
#include "serial_ex_api.h"

#define HCI_UART_TX_DMA          1

#define HCI_UART_TX_PIN          (PIN_UART3_TX)
#define HCI_UART_RX_PIN          (PIN_UART3_RX)
#define HCI_UART_TX_FIFO_SIZE    (16)
#define HCI_UART_RX_FIFO_SIZE    (32)
#define HCI_UART_RX_BUF_SIZE     (0x2000)   /* RX buffer size 8K */
#define HCI_UART_RX_ENABLE_SIZE  (512)      /* Only 512 left to read */
#define HCI_UART_RX_DISABLE_SIZE (128)      /* Only 128 left to write */

#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
#define HCI_UART_TX_BUF_SIZE     (512)      /* TX buffer size 512 */
#endif

static struct amebaz2_uart_t
{
    /* Serial UART */
    serial_t     serial_obj;

    /* UART RX RingBuf */
    uint8_t*     ring_buffer;
    uint32_t     ring_buffer_size;
    uint32_t     write_ptr;
    uint32_t     read_ptr;
    uint8_t      rx_disabled;

    /* UART RX Indication */
    HCI_RECV_IND rx_ind;

    /* UART TX */
    uint8_t      tx_run;
#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
    uint8_t*     tx_buffer;
    void*        tx_done_sem;
#endif

    /* UART Bridge */
    uint8_t      bridge_flag;
} *amebaz2_uart = NULL;

#if 0
void amebaz2_uart_set_out(uint8_t flag)
{
    if (!amebaz2_uart)
        HCI_ERR("amebaz2_uart_set_out: amebaz2_uart is NULL!");

    amebaz2_uart->bridge_flag = flag;
}

bool amebaz2_uart_tx_bridge(uint8_t rc)
{
    serial_putc(&hci_serial_obj, rc);
    return true;
}

bool amebaz2_uart_rx_bridge(uint8_t rc)
{
    extern void bt_uart_tx(uint8_t rc);
    bt_uart_tx(rc);
    return true;
}
#endif

static uint8_t amebaz2_uart_set_bdrate(uint32_t baudrate)
{
    serial_baud(&amebaz2_uart->serial_obj, baudrate);
    HCI_INFO("Set baudrate to %d success!", baudrate);
    return HCI_SUCCESS;
}

static uint8_t amebaz2_uart_set_rx_ind(HCI_RECV_IND rx_ind)
{
    amebaz2_uart->rx_ind = rx_ind;
    return HCI_SUCCESS;
}

static uint8_t amebaz2_uart_set_tx_run(uint8_t run)
{
    amebaz2_uart->tx_run = run;
    return HCI_SUCCESS;
}

static inline uint16_t amebaz2_uart_rx_to_read_space(void)
{
    return (amebaz2_uart->write_ptr + amebaz2_uart->ring_buffer_size - amebaz2_uart->read_ptr) % amebaz2_uart->ring_buffer_size;
}

static inline uint16_t amebaz2_uart_rx_to_write_space(void)
{
    return (amebaz2_uart->read_ptr + amebaz2_uart->ring_buffer_size - amebaz2_uart->write_ptr - 1) % amebaz2_uart->ring_buffer_size;
}

static void amebaz2_uart_irq(uint32_t id, SerialIrq event)
{
    uint8_t ch;
    uint16_t write_len = amebaz2_uart_rx_to_write_space();
    uint16_t max_count = (write_len > HCI_UART_RX_FIFO_SIZE) ? HCI_UART_RX_FIFO_SIZE : write_len;

    if (event == RxIrq) {
        while (serial_readable(&amebaz2_uart->serial_obj) && max_count-- > 0)
        {
            ch = serial_getc(&amebaz2_uart->serial_obj);
#if 0
            if (rltk_wlan_is_mp()) {
                if (amebaz2_uart->bridge_flag) {
                    hci_uart_rx_bridge(ch);
                    return;
                }
            }
#endif
            amebaz2_uart->ring_buffer[amebaz2_uart->write_ptr++] = ch;
            amebaz2_uart->write_ptr %= amebaz2_uart->ring_buffer_size;
        }

        if (!amebaz2_uart->rx_disabled && amebaz2_uart_rx_to_write_space() < HCI_UART_RX_DISABLE_SIZE)
        {
            serial_rts_control(&amebaz2_uart->serial_obj, 0);
            amebaz2_uart->rx_disabled = 1;
            HCI_INFO("amebaz2_uart rx disable!");
        }

        if (amebaz2_uart->rx_ind)
            amebaz2_uart->rx_ind();
    }
}

static uint16_t amebaz2_uart_send(uint8_t *buf, uint16_t len)
{
    if (!amebaz2_uart->tx_run)
        return 0;

#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
    int ret;
    if (((uint32_t)buf >> 28 == 0x6) && ((uint32_t)buf % 32 != 0)) { //if p_buf is in psram && address is not 32B aligned
        if (len > HCI_UART_TX_BUF_SIZE)
            return 0;

        memcpy(amebaz2_uart->tx_buffer, buf, len);
        ret = serial_send_stream_dma(&amebaz2_uart->serial_obj, (char *)amebaz2_uart->tx_buffer, len);
    } else {
        ret = serial_send_stream_dma(&amebaz2_uart->serial_obj, (char *)buf, len);
    }

    if (ret != 0) {
        HCI_ERR("serial_send_stream_dma fail!");
        return 0;
    }

    if (amebaz2_uart->tx_done_sem) {
        if (osif_sem_take(amebaz2_uart->tx_done_sem, 0xFFFFFFFF) == false) {
            HCI_ERR("amebaz2_uart->tx_done_sem take fail!");
            return 0;
        }
    }
#else
    serial_send_blocked(&amebaz2_uart->serial_obj, (char *)buf, len, UART_WAIT_FOREVER);
#endif

    return len;
}

#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
static void amebaz2_uart_send_done(uint32_t id)
{
    if (amebaz2_uart->tx_done_sem)
        osif_sem_give(amebaz2_uart->tx_done_sem);
}
#endif

static uint16_t amebaz2_uart_read(uint8_t *buf, uint16_t len)
{
    uint16_t read_len = amebaz2_uart_rx_to_read_space();
    read_len = (read_len > len) ? len : read_len;

    if (0 == read_len)
        return 0;

    if (read_len > amebaz2_uart->ring_buffer_size - amebaz2_uart->read_ptr)
        read_len = amebaz2_uart->ring_buffer_size - amebaz2_uart->read_ptr;

    memcpy(buf, &amebaz2_uart->ring_buffer[amebaz2_uart->read_ptr], read_len);
    amebaz2_uart->read_ptr += read_len;
    amebaz2_uart->read_ptr %= amebaz2_uart->ring_buffer_size;

    if (amebaz2_uart->rx_disabled && amebaz2_uart_rx_to_read_space() < HCI_UART_RX_ENABLE_SIZE)
    {
        serial_rts_control(&amebaz2_uart->serial_obj, 1);
        amebaz2_uart->rx_disabled = 0;
        HCI_INFO("amebaz2_uart rx enable!");
    }

    return read_len;
}

static void amebaz2_uart_set_irq(uint8_t trx_flag, uint8_t en)
{
    switch (trx_flag) {
        case UART_RX:
            serial_irq_set(&amebaz2_uart->serial_obj, RxIrq, en);
            break;
        case UART_TX:
            serial_irq_set(&amebaz2_uart->serial_obj, TxIrq, en);
            break;
        case UART_TRX:
            serial_irq_set(&amebaz2_uart->serial_obj, TxIrq, en);
            serial_irq_set(&amebaz2_uart->serial_obj, RxIrq, en);
            break;
        default:
            break;
    }
}

static uint8_t amebaz2_uart_open(void)
{
    /* Init amebaZ2_uart */
    if (!amebaz2_uart)
    {
        amebaz2_uart = osif_mem_alloc(0, sizeof(struct amebaz2_uart_t));
        if (!amebaz2_uart) {
            HCI_ERR("amebaz2_uart is NULL!");
            return HCI_FAIL;
        }
        memset(amebaz2_uart, 0, sizeof(struct amebaz2_uart_t));
    }
    if (!amebaz2_uart->ring_buffer)
    {
        amebaz2_uart->ring_buffer = osif_mem_aligned_alloc(0, HCI_UART_RX_BUF_SIZE, 4);
        if (!amebaz2_uart->ring_buffer) {
            HCI_ERR("amebaz2_uart->ring_buffer is NULL!");
            return HCI_FAIL;
        }
        memset(amebaz2_uart->ring_buffer, 0, sizeof(HCI_UART_RX_BUF_SIZE));
    }
    amebaz2_uart->ring_buffer_size = HCI_UART_RX_BUF_SIZE;
    amebaz2_uart->read_ptr = 0;
    amebaz2_uart->write_ptr = 0;
    amebaz2_uart->rx_disabled = 0;
    amebaz2_uart->rx_ind = 0;
    amebaz2_uart->tx_run = 1;

#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
    if (!amebaz2_uart->tx_buffer)
    {
        amebaz2_uart->tx_buffer = osif_mem_aligned_alloc(0, HCI_UART_TX_BUF_SIZE, 32);
        if (!amebaz2_uart->tx_buffer) {
            HCI_ERR("amebaz2_uart->tx_buffer is NULL!");
            return HCI_FAIL;
        }
        memset(amebaz2_uart->tx_buffer, 0, sizeof(HCI_UART_TX_BUF_SIZE));
    }

    if (osif_sem_create(&amebaz2_uart->tx_done_sem, 0, 1) == false) {
        HCI_ERR("amebaz2_uart->tx_done_sem create fail!");
        return HCI_FAIL;
    }
#endif

    serial_init(&amebaz2_uart->serial_obj, (PinName)HCI_UART_TX_PIN, (PinName)HCI_UART_RX_PIN);
    serial_baud(&amebaz2_uart->serial_obj, 115200);
    serial_format(&amebaz2_uart->serial_obj, 8, ParityNone, 1);
    amebaz2_uart->serial_obj.uart_adp.base_addr->fcr_b.rxfifo_trigger_level = FifoLvHalf;
    //serial_rx_fifo_level(&amebaz2_uart->serial_obj, FifoLvHalf);
    serial_set_flow_control(&amebaz2_uart->serial_obj, FlowControlRTSCTS, NC, NC);
#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
    serial_send_comp_handler(&amebaz2_uart->serial_obj, (void *)amebaz2_uart_send_done, (uint32_t)&amebaz2_uart->serial_obj);
#endif
    serial_clear_rx(&amebaz2_uart->serial_obj);
    serial_irq_handler(&amebaz2_uart->serial_obj, amebaz2_uart_irq, (uint32_t)&amebaz2_uart->serial_obj);
    serial_irq_set(&amebaz2_uart->serial_obj, RxIrq, 0);
    //serial_irq_set(&amebaz2_uart->serial_obj, RxIrq, 1);

    return HCI_SUCCESS;
}

static uint8_t amebaz2_uart_close(void)
{
    if (amebaz2_uart)
    {
        /* Disable Serial UART */
        serial_free(&amebaz2_uart->serial_obj);
        memset(&amebaz2_uart->serial_obj, 0, sizeof(serial_t));

#if defined(HCI_UART_TX_DMA) && HCI_UART_TX_DMA
        if (amebaz2_uart->tx_done_sem) {
            osif_sem_delete(amebaz2_uart->tx_done_sem);
            amebaz2_uart->tx_done_sem = NULL;
        }

        if (amebaz2_uart->tx_buffer)
            osif_mem_aligned_free(amebaz2_uart->tx_buffer);
#endif
        /* Deinit UART Ringbuf */
        if (amebaz2_uart->ring_buffer)
            osif_mem_aligned_free(amebaz2_uart->ring_buffer);

        osif_mem_free(amebaz2_uart);
        amebaz2_uart = NULL;

        return HCI_SUCCESS;
    } else {
        HCI_ERR("amebaz2_uart is NULL!");
        return HCI_FAIL;
    }
}

HCI_UART_OPS hci_uart_ops = {
    .open       = amebaz2_uart_open,
    .close      = amebaz2_uart_close,
    .send       = amebaz2_uart_send,
    .read       = amebaz2_uart_read,
    .set_rx_ind = amebaz2_uart_set_rx_ind,
    .set_tx_run = amebaz2_uart_set_tx_run,
    .set_bdrate = amebaz2_uart_set_bdrate,
    .set_irq    = amebaz2_uart_set_irq,
#if 0
    .set_out    = amebaz2_uart_set_out,
    .tx_bridge  = amebaz2_uart_tx_bridge,
    .rx_bridge  = amebaz2_uart_rx_bridge,
#endif
};
