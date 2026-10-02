#include "uart_rx_queue.h"

#include <assert.h>
#include <stddef.h>
#include <stdio.h>

int main(void) {
    uart_rx_queue_t queue;
    uart_rx_item_t item;
    uart_rx_queue_init(&queue);
    assert(!uart_rx_queue_pop(&queue, &item));
    for (unsigned index = 0U; index < UART_RX_QUEUE_CAPACITY - 1U; ++index) {
        assert(uart_rx_queue_push(&queue, (uint8_t)index, index + 100U));
    }
    assert(!uart_rx_queue_push(&queue, 0xFFU, 999U));
    for (unsigned index = 0U; index < UART_RX_QUEUE_CAPACITY - 1U; ++index) {
        assert(uart_rx_queue_pop(&queue, &item));
        assert(item.byte == (uint8_t)index && item.received_at_ms == index + 100U);
    }
    for (unsigned index = 0U; index < 4096U; ++index) {
        assert(uart_rx_queue_push(&queue, (uint8_t)index, 0xFFFFFFFFUL - index));
        assert(uart_rx_queue_pop(&queue, &item));
        assert(item.byte == (uint8_t)index && item.received_at_ms == 0xFFFFFFFFUL - index);
    }
    assert(uart_rx_queue_push(&queue, 1U, 0U));
    uart_rx_queue_reset(&queue);
    assert(!uart_rx_queue_pop(&queue, &item));
    assert(!uart_rx_queue_push(NULL, 0U, 0U));
    assert(!uart_rx_queue_pop(NULL, &item));
    assert(!uart_rx_queue_pop(&queue, NULL));
    uart_rx_queue_init(NULL);
    uart_rx_queue_reset(NULL);
    puts("UART RX queue tests passed");
    return 0;
}
