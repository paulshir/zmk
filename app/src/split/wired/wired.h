/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/sys/ring_buffer.h>
#include <zephyr/device.h>

#include <zmk/split/transport/types.h>

struct event_envelope {
    uint8_t source;
    struct zmk_split_transport_peripheral_event event;
    uint32_t crc;
} __packed;

struct command_envelope {
    uint8_t source;
    struct zmk_split_transport_central_command cmd;
    uint32_t crc;
} __packed;

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_POLLING)

void zmk_split_wired_poll_out(struct ring_buf *tx_buf, const struct device *uart);

void zmk_split_wired_poll_in(struct ring_buf *rx_buf, const struct device *uart,
                             struct k_work *process_data_work, size_t envelope_size);

#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

void zmk_split_wired_fifo_read(const struct device *dev, struct ring_buf *buf,
                               struct k_work *process_work);
void zmk_split_wired_fifo_fill(const struct device *dev, struct ring_buf *tx_buf);

#endif