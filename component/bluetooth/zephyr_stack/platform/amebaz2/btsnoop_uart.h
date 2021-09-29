#ifndef  BTSNOOP_UART_H
#define  BTSNOOP_UART_H

#ifdef __cplusplus
extern "C" {
#endif

void btsnoop_uart_init(void);
void btsnoop_uart_deinit(void);
int btsnoop_uart_tx(uint8_t *data, uint16_t len);
int btsnoop_uart_rx(uint8_t *buf, uint16_t len);
void btsnoop_uart_setirq(void* cb);

#ifdef __cplusplus
}
#endif

#endif /* BTSNOOP_UART_H */