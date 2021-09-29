#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "objects.h"
#include "serial_api.h"
#include "serial_ex_api.h"

#define BTSNOOP_TX  PA_3 
#define BTSNOOP_RX  PA_2 
#define BTSNOOP_UART_BAUDRATE 1500000

static uint8_t PKT_Head_Flag[6] = {0xA5, 0x5A, 0xC3, 0x3C, 0x5C, 0xC5};
static uint8_t PKT_Tail_Flag[2] = {0x3A, 0xA3};

typedef struct _btsnoop_uart_info {
    uint8_t rx_buffer[16];
	uint8_t rx_buffer_cnt;
    uint8_t rx_intr_flag;
} BTSNOOP_UART_INFO;

BTSNOOP_UART_INFO btsnoop_uart_info;
serial_t btsnoop_sobj;

static void (*btsnoop_irq_callback)(uint64_t);

static void btsnoop_uart_irq(uint32_t id, SerialIrq event)
{
    serial_t* serial_obj = (void *)id;
	int max_count = 16;
	uint8_t ch, *buf_ptr;
    uint64_t timestamp_temp = 0;

    if (event == RxIrq) {
        btsnoop_uart_info.rx_intr_flag = 2;
        do {
            ch = serial_getc(serial_obj);
            // printf("ch[%d] is %d\n", btsnoop_uart_info.rx_buffer_cnt, ch);
            btsnoop_uart_info.rx_buffer[btsnoop_uart_info.rx_buffer_cnt++] = ch;
        } while (serial_readable(serial_obj) && max_count-- > 0);

        buf_ptr = btsnoop_uart_info.rx_buffer;
        if (buf_ptr[0] == 't' && buf_ptr[1] == 'i' && buf_ptr[2] == 'm' && buf_ptr[3] == 'e' \
            && btsnoop_uart_info.rx_buffer_cnt >= 12) {
            memcpy(&timestamp_temp, buf_ptr+4, 8);
            if (btsnoop_irq_callback) {
                btsnoop_irq_callback(timestamp_temp);
            }
            // btsnoop_base_timestamp = timestamp_temp;
            // btsnoop_start_time = (uint64_t)k_uptime_get();
            btsnoop_uart_info.rx_buffer_cnt = 0;
            memset(buf_ptr, 0, 16);
        }
    }
}

void btsnoop_uart_init(void)
{
    printf("[BTSNOOP] btsnoop_uart_init: BTSNOOP UART OPEN\r\n");
    hal_pinmux_unregister(BTSNOOP_TX, 0x01 << 4);
    hal_pinmux_unregister(BTSNOOP_RX, 0x01 << 4);
    hal_gpio_pull_ctrl(BTSNOOP_TX, 0);
    hal_gpio_pull_ctrl(BTSNOOP_RX, 0);

    memset(&btsnoop_uart_info, 0 ,sizeof(btsnoop_uart_info));
    serial_init(&btsnoop_sobj, BTSNOOP_TX, BTSNOOP_RX);
    serial_baud(&btsnoop_sobj, BTSNOOP_UART_BAUDRATE);
    serial_format(&btsnoop_sobj, 8, ParityNone, 1);
}

void btsnoop_uart_deinit(void)
{
    serial_free(&btsnoop_sobj);
}

int btsnoop_uart_tx(uint8_t *data, uint16_t len)
{
    serial_send_blocked(&btsnoop_sobj, (char *)data, len, len);
    return 1;
}

int btsnoop_uart_rx(uint8_t *buf, uint16_t len)
{
    int ret;
    ret = serial_recv_blocked(&btsnoop_sobj, (char *)buf, len, 500+len);
    return ret;
}

void btsnoop_uart_setirq(void* cb)
{
    serial_clear_rx(&btsnoop_sobj);
    btsnoop_irq_callback = cb;
    serial_irq_handler(&btsnoop_sobj, btsnoop_uart_irq, (uint32_t)&btsnoop_sobj);
    serial_irq_set(&btsnoop_sobj, RxIrq, 1);
}