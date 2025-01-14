/*
 * Copyright (c) 2024 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/settings/settings.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/ring_buffer.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zmk/stdlib.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/transport/peripheral.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "wired.h"

#define DT_DRV_COMPAT zmk_wired_split

// struct wired_bus_state {
//     struct k_work event_publish_work;
//     struct ring_buf *rx_buf;
//     struct ring_buf *tx_buf;
// };

// struct wired_bus {
//     const struct device *uart;
//     struct gpio_dt_spec recv_gpio;
//     bool poll;
//     struct wired_bus_state *state;
// };

// struct wired_peripheral {
//     uint8_t reg;
//     const struct wired_bus *bus;
// };

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#error "NOT YET IMPLEMENTED"

#elif DT_HAS_CHOSEN(zmk_split_uart)

RING_BUF_DECLARE(chosen_rx_buf,
                 sizeof(struct command_envelope) * CONFIG_ZMK_SPLIT_WIRED_CMD_BUFFER_ITEMS);
RING_BUF_DECLARE(chosen_tx_buf,
                 sizeof(struct event_envelope) * CONFIG_ZMK_SPLIT_WIRED_EVENT_BUFFER_ITEMS);

static const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zmk_split_uart));
static const uint8_t peripheral_id = 0;

K_SEM_DEFINE(tx_sem, 0, 1);

#else

// TODO: Error to link to docs
#error "Need to assign a 'zmk,wired-split-uart` node or create a wired split node with details"

#endif

static void publish_commands_work(struct k_work *work);

K_WORK_DEFINE(publish_commands, publish_commands_work);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

// static void fifo_fill(const struct device *dev) {
//     struct ring_buf *tx_buf = &chosen_tx_buf;
//     uint32_t len;
//     while ((len = ring_buf_size_get(tx_buf)) > 0) {
//         uint8_t *buf;
//         uint32_t claim_len = ring_buf_get_claim(tx_buf, &buf, tx_buf->size);

//         if (claim_len <= 0) {
//             break;
//         }

//         int sent = uart_fifo_fill(dev, buf, claim_len);

//         LOG_DBG("Sent %d to the UART", sent);

//         ring_buf_get_finish(tx_buf, MAX(sent, 0));

//         if (sent <= 0) {
//             break;
//         }
//     }

//     if (ring_buf_size_get(tx_buf) == 0) {
//         uart_irq_tx_disable(dev);
//     }
// }

static void serial_cb(const struct device *dev, void *user_data) {
    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            zmk_split_wired_fifo_read(dev, &chosen_rx_buf, &publish_commands);
        }

        if (uart_irq_tx_ready(dev)) {
            zmk_split_wired_fifo_fill(dev, &chosen_tx_buf);
        }
    }
}

#else

static void send_pending_tx_work_cb(struct k_work *work) {
    zmk_split_wired_poll_out(&chosen_tx_buf, uart);
}

static K_WORK_DEFINE(send_pending_tx, send_pending_tx_work_cb);

static void wired_peripheral_read_tick_cb(struct k_timer *timer) {
    zmk_split_wired_poll_in(&chosen_rx_buf, uart, &publish_commands,
                            sizeof(struct command_envelope));
}

static K_TIMER_DEFINE(wired_peripheral_read_timer, wired_peripheral_read_tick_cb, NULL);

#endif

static int zmk_split_wired_peripheral_init(void) {
    if (!device_is_ready(uart)) {
        return -ENODEV;
    }

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)
    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart, serial_cb, NULL);

    if (ret < 0) {
        if (ret == -ENOTSUP) {
            LOG_ERR("Interrupt-driven UART API support not enabled");
        } else if (ret == -ENOSYS) {
            LOG_ERR("UART device does not support interrupt-driven API");
        } else {
            LOG_ERR("Error setting UART callback: %d\n", ret);
        }
        return ret;
    }

    uart_irq_rx_enable(uart);
#else
    k_timer_start(&wired_peripheral_read_timer, K_TICKS(CONFIG_ZMK_SPLIT_WIRED_POLLING_RX_PERIOD),
                  K_TICKS(CONFIG_ZMK_SPLIT_WIRED_POLLING_RX_PERIOD));
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

    return 0;
}

SYS_INIT(zmk_split_wired_peripheral_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

#include <zmk/split/transport/types.h>

static int
split_peripheral_wired_report_event(const struct zmk_split_transport_peripheral_event *event) {
    size_t added = 0;

    struct event_envelope env = {
        .source = peripheral_id,
        .event = *event,
    };

    if (ring_buf_space_get(&chosen_tx_buf) < sizeof(env)) {
        LOG_ERR("Insufficient room for sending the peripheral event");
        return -ENOSPC;
    }

    env.crc = crc32_ieee((void *)&env, sizeof(env) - 4);

    while (added < sizeof(env)) {
        uint8_t *buf;
        size_t claim = ring_buf_put_claim(&chosen_tx_buf, &buf, sizeof(env) - added);

        if (claim == 0) {
            break;
        }

        memcpy(buf, (uint8_t *)&env + added, claim);

        added += claim;
        ring_buf_put_finish(&chosen_tx_buf, claim);

        LOG_DBG("Added %d to the ring buffer!", claim);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)
        uart_irq_tx_enable(uart);
#else
        k_work_submit(&send_pending_tx);
#endif
    }
    return 0;
}

static const struct zmk_split_transport_peripheral_api peripheral_api = {
    .report_event = split_peripheral_wired_report_event,
};

ZMK_SPLIT_TRANSPORT_PERIPHERAL_REGISTER(wired_peripheral, &peripheral_api);

static void publish_commands_work(struct k_work *work) {
    while (ring_buf_size_get(&chosen_rx_buf) >= sizeof(struct command_envelope)) {
        struct command_envelope env;
        size_t bytes_left = sizeof(struct command_envelope);

        while (bytes_left > 0) {
            size_t read = ring_buf_get(&chosen_rx_buf, (uint8_t *)&env + (sizeof(env) - bytes_left),
                                       bytes_left);
            bytes_left -= read;
        }

        // Exclude the trailing 4 bytes that contain the received CRC
        uint32_t crc = crc32_ieee((uint8_t *)&env, sizeof(env) - 4);
        if (crc != env.crc) {
            LOG_WRN("Data corruption in received peripheral event, ignoring");
            return;
        }

        zmk_split_transport_peripheral_command_handler(&wired_peripheral, env.cmd);
    }
}
