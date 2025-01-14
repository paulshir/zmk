/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include "wired.h"

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_POLLING)

void zmk_split_wired_poll_out(struct ring_buf *tx_buf, const struct device *uart) {
    uint8_t *buf;
    uint32_t claim_len;
    while ((claim_len = ring_buf_get_claim(tx_buf, &buf, MIN(32, tx_buf->size))) > 0) {
        LOG_HEXDUMP_DBG(buf, claim_len, "TX Bytes");
        for (int i = 0; i < claim_len; i++) {
            uart_poll_out(uart, buf[i]);
        }

        ring_buf_get_finish(tx_buf, claim_len);
    }
}

void zmk_split_wired_poll_in(struct ring_buf *rx_buf, const struct device *uart,
                             struct k_work *process_data_work, size_t envelope_size) {
    uint8_t *buf;
    uint32_t read = 0;
    uint32_t claim_len = ring_buf_put_claim(rx_buf, &buf, ring_buf_space_get(rx_buf));
    if (claim_len < 1) {
        LOG_WRN("No room available for reading in from the serial port");
        return;
    }

    while (read < claim_len) {
        if (uart_poll_in(uart, buf + read) < 0) {
            break;
        }

        read++;
    }

    ring_buf_put_finish(rx_buf, read);

    if (ring_buf_size_get(rx_buf) >= envelope_size) {
        k_work_submit(process_data_work);
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_POLLING)

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

void zmk_split_wired_fifo_read(const struct device *dev, struct ring_buf *buf,
                               struct k_work *process_work) {
    uint32_t last_read = 0, len = 0;
    do {
        uint8_t *buffer;
        len = ring_buf_put_claim(buf, &buffer, buf->size);
        if (len > 0) {
            last_read = uart_fifo_read(dev, buffer, len);

            ring_buf_put_finish(buf, last_read);
        } else {
            LOG_ERR("Dropping incoming RPC byte, insufficient room in the RX buffer. Bump "
                    "CONFIG_ZMK_STUDIO_RPC_RX_BUF_SIZE.");
            uint8_t dummy;
            last_read = uart_fifo_read(dev, &dummy, 1);
        }
    } while (last_read && last_read == len);

    k_work_submit(process_work);
}

void zmk_split_wired_fifo_fill(const struct device *dev, struct ring_buf *tx_buf) {
    uint32_t len;
    while ((len = ring_buf_size_get(tx_buf)) > 0) {
        uint8_t *buf;
        uint32_t claim_len = ring_buf_get_claim(tx_buf, &buf, tx_buf->size);

        if (claim_len <= 0) {
            break;
        }

        int sent = uart_fifo_fill(dev, buf, claim_len);

        LOG_DBG("Sent %d to the UART", sent);

        ring_buf_get_finish(tx_buf, MAX(sent, 0));

        if (sent <= 0) {
            break;
        }
    }

    if (ring_buf_size_get(tx_buf) == 0) {
        uart_irq_tx_disable(dev);
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)