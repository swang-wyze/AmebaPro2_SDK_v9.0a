#ifndef  BTSNOOP_H
#define  BTSNOOP_H

#ifdef __cplusplus
extern "C" {
#endif

/* the direction is from the point of host */
#define H4_DIR_TX  0
#define H4_DIR_RX  1

/* convert a uint32_t data from host to TCP/IP network byte order (big-endian) */
uint32_t htonl(uint32_t hl);

/* convert a uint32_t data from TCP/IP network byte order (big-endian) to host */
uint32_t ntohl(uint32_t nl);

/* convert a uint64_t data from host to TCP/IP network byte order (big-endian) */
uint64_t htonll(uint64_t hl);

/* convert a uint64_t data from TCP/IP network byte order (big-endian) to host */
uint64_t ntohll(uint64_t nl);

/* initialize the function of btsnoop */
void btsnoop_init(void);

void btsnoop_deinit(void);

/* send the btsnoop packet through uart */
// int btsnoop_uart_tx(uint8_t *data, uint16_t len);

// int btsnoop_uart_rx(uint8_t *buf, uint16_t len);

/* create the btsnoop packet and send to host computer */
void btsnp_pkt_create_send(uint8_t dir, uint32_t h4_len, uint32_t flags, uint8_t h4_type, uint8_t *data);

#ifdef __cplusplus
}
#endif

#endif /* BTSNOOP_H */