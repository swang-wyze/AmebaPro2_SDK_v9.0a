/*
 *******************************************************************************
 * Copyright(c) 2021, Realtek Semiconductor Corporation. All rights reserved.
 *******************************************************************************
 */

#include <string.h>
#include "ameba_soc.h"
#include "hci_uart.h"
#include "hci_dbg.h"

#define HCI_UART_DEV             (UART3_DEV)
#define HCI_UART_IRQ             (UART3_BT_IRQ)
#define HCI_UART_IRQ_PRIO        (10)
#define HCI_UART_TX_FIFO_SIZE    (32)
#define HCI_UART_RX_FIFO_SIZE    (32)
#define HCI_UART_RX_BUF_SIZE     (0x2000)   /* RX buffer size 8K */
#define HCI_UART_RX_ENABLE_SIZE  (512)      /* Only 512 left to read */
#define HCI_UART_RX_DISABLE_SIZE (128)      /* Only 128 left to write */

static struct amebad2_uart_t
{
    /* UART */
    UART_InitTypeDef UART_InitStruct;

    /* UART RX RingBuf */
    uint8_t*         ring_buffer;
    uint32_t         ring_buffer_size;
    uint32_t         write_ptr;
    uint32_t         read_ptr;
    uint8_t          rx_disabled;

    /* UART RX Indication */
    HCI_RECV_IND     rx_ind;

    /* UART TX */
    uint8_t*         tx_buf;
    uint16_t         tx_len;
    void*            tx_done_sem;
} *amebad2_uart = NULL;

extern void UART_INT_Clear(UART_TypeDef *UARTx, uint32_t UART_IT);

static uint8_t amebad2_uart_set_bdrate(uint32_t baudrate)
{
    UART_SetBaud(HCI_UART_DEV, baudrate);
    HCI_INFO("Set baudrate to %d success!", baudrate);
    return HCI_SUCCESS;
}

static uint8_t amebad2_uart_set_rx_ind(HCI_RECV_IND rx_ind)
{
    amebad2_uart->rx_ind = rx_ind;
    return HCI_SUCCESS;
}

static inline uint16_t amebad2_uart_rx_to_read_space(void)
{
    return (amebad2_uart->write_ptr + amebad2_uart->ring_buffer_size - amebad2_uart->read_ptr) % amebad2_uart->ring_buffer_size;
}

static inline uint16_t amebad2_uart_rx_to_write_space(void)
{
    return (amebad2_uart->read_ptr + amebad2_uart->ring_buffer_size - amebad2_uart->write_ptr - 1) % amebad2_uart->ring_buffer_size;
}

static inline uint8_t amebad2_uart_irq_tx_ready(void)
{
    return (UART_LineStatusGet(HCI_UART_DEV) & RUART_BIT_ETBEI);
}

static inline uint8_t amebad2_uart_irq_rx_ready(void)
{
    return (UART_LineStatusGet(HCI_UART_DEV) & (RUART_BIT_ERBI | RUART_BIT_ETOI));
}

static inline uint8_t amebad2_uart_irq_is_pending(void)
{
    return (amebad2_uart_irq_tx_ready() | amebad2_uart_irq_rx_ready());
}

static inline void transmit_chars(void)
{
    uint16_t max_count = HCI_UART_TX_FIFO_SIZE;

    while (amebad2_uart->tx_len > 0 && max_count-- > 0)
    {
        UART_CharPut(HCI_UART_DEV, *(amebad2_uart->tx_buf));
        amebad2_uart->tx_buf++;
        amebad2_uart->tx_len--;
    }

    if (amebad2_uart->tx_len == 0)
    {
        UART_INTConfig(HCI_UART_DEV, RUART_BIT_ETBEI, DISABLE);
        if (amebad2_uart->tx_done_sem)
            osif_sem_give(amebad2_uart->tx_done_sem);
    }
}

static inline void receive_chars(void)
{
    uint8_t ch;
    uint16_t write_len = amebad2_uart_rx_to_write_space();
    uint16_t max_count = (write_len > HCI_UART_RX_FIFO_SIZE) ? HCI_UART_RX_FIFO_SIZE : write_len;

    while (UART_Readable(HCI_UART_DEV) && max_count-- > 0)
    {
        UART_CharGet(HCI_UART_DEV, &ch);
        amebad2_uart->ring_buffer[amebad2_uart->write_ptr++] = ch;
        amebad2_uart->write_ptr %= amebad2_uart->ring_buffer_size;
    }

    if (!amebad2_uart->rx_disabled && amebad2_uart_rx_to_write_space() < HCI_UART_RX_DISABLE_SIZE)
    {
        UART_INTConfig(HCI_UART_DEV, RUART_BIT_ERBI | RUART_BIT_ETOI, DISABLE);
        amebad2_uart->rx_disabled = 1;
        HCI_INFO("amebad2_uart rx disable!");
    }

    if (amebad2_uart->rx_ind)
        amebad2_uart->rx_ind();
}

static uint32_t amebad2_uart_irq(void *data)
{
    (void)data;
    uint32_t reg_lsr = UART_LineStatusGet(HCI_UART_DEV);

    if (reg_lsr & RUART_BIT_TX_EMPTY)
        transmit_chars();

    if (reg_lsr & RUART_BIT_RXFIFO_INT)
        receive_chars();

    if (reg_lsr & RUART_BIT_TIMEOUT_INT)
    {
        UART_INT_Clear(HCI_UART_DEV, RUART_BIT_TOICF);
        receive_chars();
    }

    if ((reg_lsr & RUART_BIT_RXFIFO_ERR))
    {
        UART_INTConfig(HCI_UART_DEV, RUART_BIT_ELSI, DISABLE);
        UART_INT_Clear(HCI_UART_DEV, RUART_BIT_RLSICF);
    }
    return 0;
}

static uint16_t amebad2_uart_send(uint8_t *buf, uint16_t len)
{
    if (!amebad2_uart) {
        HCI_ERR("amebad2_uart is NULL!");
        return 0;
    }

    /* UART_SendData() does not work */
    amebad2_uart->tx_buf = buf;
    amebad2_uart->tx_len = len;

    UART_INTConfig(HCI_UART_DEV, RUART_BIT_ETBEI, ENABLE);

    if (amebad2_uart->tx_done_sem)
    {
        if (osif_sem_take(amebad2_uart->tx_done_sem, 0xFFFFFFFF) == false) {
            HCI_ERR("amebad2_uart->tx_done_sem take fail!");
            return 0;
        }
    }

    /* Trigger TX Empty Interrrupt, so TX done here */
    return len;
}

static uint16_t amebad2_uart_read(uint8_t *buf, uint16_t len)
{
    uint16_t read_len = amebad2_uart_rx_to_read_space();
    read_len = (read_len > len) ? len : read_len;

    if (0 == read_len)
        return 0;

    if (read_len > amebad2_uart->ring_buffer_size - amebad2_uart->read_ptr)
        read_len = amebad2_uart->ring_buffer_size - amebad2_uart->read_ptr;

    memcpy(buf, &amebad2_uart->ring_buffer[amebad2_uart->read_ptr], read_len);
    amebad2_uart->read_ptr += read_len;
    amebad2_uart->read_ptr %= amebad2_uart->ring_buffer_size;

    if (amebad2_uart->rx_disabled && amebad2_uart_rx_to_read_space() < HCI_UART_RX_ENABLE_SIZE)
    {
        UART_INTConfig(HCI_UART_DEV, RUART_BIT_ERBI | RUART_BIT_ETOI, ENABLE);
        amebad2_uart->rx_disabled = 0;
        HCI_INFO("amebad2_uart rx enable!");
    }

    return read_len;
}

static uint8_t amebad2_uart_open(void)
{
    /* Init amebad2_uart */
    if (!amebad2_uart)
    {
        amebad2_uart = osif_mem_alloc(0, sizeof(struct amebad2_uart_t));
        if (!amebad2_uart) {
            HCI_ERR("amebad2_uart is NULL!");
            return HCI_FAIL;
        }
        memset(amebad2_uart, 0, sizeof(struct amebad2_uart_t));
    }
    if (!amebad2_uart->ring_buffer)
    {
        amebad2_uart->ring_buffer = osif_mem_aligned_alloc(0, HCI_UART_RX_BUF_SIZE, 4);
        if (!amebad2_uart->ring_buffer) {
            HCI_ERR("amebad2_uart->ring_buffer is NULL!");
            return HCI_FAIL;
        }
        memset(amebad2_uart->ring_buffer, 0, sizeof(HCI_UART_RX_BUF_SIZE));
    }
    amebad2_uart->ring_buffer_size = HCI_UART_RX_BUF_SIZE;
    amebad2_uart->read_ptr = 0;
    amebad2_uart->write_ptr = 0;
    amebad2_uart->rx_disabled = 0;

    if (osif_sem_create(&amebad2_uart->tx_done_sem, 0, 1) == false) {
        HCI_ERR("amebad2_uart->tx_done_sem create fail!");
        return HCI_FAIL;
    }

    /* Enable Clock */
    RCC_PeriphClockCmd(APBPeriph_UART3, APBPeriph_UART3_CLOCK, ENABLE);

    /* Enable UART
     * Use Flow Control (When rx FIFO reaches level, RTS will be pulled high)
     * Use Baudrate 115200 (Default)
     */
    UART_InitTypeDef *pUARTStruct = &amebad2_uart->UART_InitStruct;
    UART_StructInit(pUARTStruct);
    pUARTStruct->WordLen = RUART_WLS_8BITS;
    pUARTStruct->StopBit = RUART_STOP_BIT_1;
    pUARTStruct->Parity = RUART_PARITY_DISABLE;
    pUARTStruct->ParityType = RUART_EVEN_PARITY;
    pUARTStruct->StickParity = RUART_STICK_PARITY_DISABLE;
    pUARTStruct->RxFifoTrigLevel = UART_RX_FIFOTRIG_LEVEL_62BYTES;
    pUARTStruct->FlowControl = ENABLE;
    UART_Init(HCI_UART_DEV, pUARTStruct);
    UART_SetBaud(HCI_UART_DEV, 115200);

    /* Disable and Enable UART Interrupt */
    InterruptDis(HCI_UART_IRQ);
    InterruptUnRegister(HCI_UART_IRQ);
    InterruptRegister((IRQ_FUN)amebad2_uart_irq, HCI_UART_IRQ, NULL, HCI_UART_IRQ_PRIO);
    InterruptEn(HCI_UART_IRQ, HCI_UART_IRQ_PRIO);

    UART_INTConfig(HCI_UART_DEV, RUART_BIT_ETBEI, DISABLE);
    UART_INTConfig(HCI_UART_DEV, RUART_BIT_ERBI | RUART_BIT_ELSI | RUART_BIT_ETOI, ENABLE);

    /* Enable Uart High Rate Rx Path */
    UART_RxCmd(HCI_UART_DEV, ENABLE);

    return HCI_SUCCESS;
}

static uint8_t amebad2_uart_close(void)
{
    if (amebad2_uart)
    {
        /* Disable UART Interrupt and UART */
        InterruptDis(HCI_UART_IRQ);
        InterruptUnRegister(HCI_UART_IRQ);
        UART_DeInit(HCI_UART_DEV);

        if (amebad2_uart->tx_done_sem)
        {
            osif_sem_delete(amebad2_uart->tx_done_sem);
            amebad2_uart->tx_done_sem = NULL;
        }

        /* Deinit UART Ringbuf */
        if (amebad2_uart->ring_buffer)
            osif_mem_aligned_free(amebad2_uart->ring_buffer);

        osif_mem_free(amebad2_uart);
        amebad2_uart = NULL;

        return HCI_SUCCESS;
    } else {
        HCI_ERR("amebad2_uart is NULL!");
        return HCI_FAIL;
    }
}

HCI_UART_OPS hci_uart_ops = {
    .open       = amebad2_uart_open,
    .close      = amebad2_uart_close,
    .send       = amebad2_uart_send,
    .read       = amebad2_uart_read,
    .set_rx_ind = amebad2_uart_set_rx_ind,
    .set_bdrate = amebad2_uart_set_bdrate,
};
