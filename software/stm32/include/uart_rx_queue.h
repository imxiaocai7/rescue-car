#ifndef RESCUE_UART_RX_QUEUE_H
#define RESCUE_UART_RX_QUEUE_H

#include <stdbool.h>
#include <stdatomic.h>
#include <stdint.h>

#define UART_RX_QUEUE_CAPACITY 256U

typedef struct {
    uint8_t byte;
    uint32_t received_at_ms;
} uart_rx_item_t;

/* Single IRQ producer, single main-loop consumer, lock-free publication. */
typedef struct {
    uart_rx_item_t items[UART_RX_QUEUE_CAPACITY];
    atomic_uint head;
    atomic_uint tail;
} uart_rx_queue_t;

void uart_rx_queue_init(uart_rx_queue_t *queue);
/* Reset only with producer IRQ disabled. */
void uart_rx_queue_reset(uart_rx_queue_t *queue);
bool uart_rx_queue_push(uart_rx_queue_t *queue, uint8_t byte, uint32_t received_at_ms);
bool uart_rx_queue_pop(uart_rx_queue_t *queue, uart_rx_item_t *item);

#endif
