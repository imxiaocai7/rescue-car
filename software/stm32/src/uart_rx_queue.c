#include "uart_rx_queue.h"

#include <stddef.h>

_Static_assert(ATOMIC_INT_LOCK_FREE == 2, "UART indices must be lock-free in an ISR");

void uart_rx_queue_init(uart_rx_queue_t *queue) {
    if (queue != NULL) {
        atomic_init(&queue->head, 0U);
        atomic_init(&queue->tail, 0U);
    }
}

void uart_rx_queue_reset(uart_rx_queue_t *queue) {
    if (queue != NULL) {
        atomic_store_explicit(&queue->head, 0U, memory_order_relaxed);
        atomic_store_explicit(&queue->tail, 0U, memory_order_relaxed);
    }
}

bool uart_rx_queue_push(uart_rx_queue_t *queue, uint8_t byte, uint32_t received_at_ms) {
    if (queue == NULL) {
        return false;
    }
    const unsigned head = atomic_load_explicit(&queue->head, memory_order_relaxed);
    const unsigned next = (head + 1U) % UART_RX_QUEUE_CAPACITY;
    if (next == atomic_load_explicit(&queue->tail, memory_order_acquire)) {
        return false;
    }
    queue->items[head].byte = byte;
    queue->items[head].received_at_ms = received_at_ms;
    atomic_store_explicit(&queue->head, next, memory_order_release);
    return true;
}

bool uart_rx_queue_pop(uart_rx_queue_t *queue, uart_rx_item_t *item) {
    if (queue == NULL || item == NULL) {
        return false;
    }
    const unsigned tail = atomic_load_explicit(&queue->tail, memory_order_relaxed);
    if (tail == atomic_load_explicit(&queue->head, memory_order_acquire)) {
        return false;
    }
    *item = queue->items[tail];
    atomic_store_explicit(&queue->tail, (tail + 1U) % UART_RX_QUEUE_CAPACITY, memory_order_release);
    return true;
}
