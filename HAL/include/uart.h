#ifndef UART_H
#define UART_H

#include <inttypes.h>

void uart_init(uint32_t baudrate, uint32_t parity,
               uint32_t stop_bits, uint32_t word_length);

#endif /* UART_H */
