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
#include <zmk/split/transport/central.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>
#include <zmk/hid_indicators_types.h>
#include <zmk/physical_layouts.h>

#include "wired.h"

#define RX_BUFFER_SIZE (sizeof(struct event_envelope) * CONFIG_ZMK_SPLIT_WIRED_EVENT_BUFFER_ITEMS)
#define TX_BUFFER_SIZE (sizeof(struct command_envelope) * CONFIG_ZMK_SPLIT_WIRED_CMD_BUFFER_ITEMS)

struct wired_bus_state {
    struct k_work event_publish_work;

    struct ring_buf rx_buf;
    uint8_t rx_buffer[RX_BUFFER_SIZE];

    struct ring_buf tx_buf;
    uint8_t tx_buffer[TX_BUFFER_SIZE];
};

struct wired_bus {
    const struct device *uart;
    struct gpio_dt_spec recv_gpio;
    struct wired_bus_state *state;
};

struct wired_peripheral {
    uint8_t reg;
    const struct wired_bus *bus;
};

#if DT_HAS_CHOSEN(zmk_split_uart)

struct wired_bus_state bus_state = {};

static const struct wired_bus buses[] = {{
    .uart = DEVICE_DT_GET(DT_CHOSEN(zmk_split_uart)),
    .state = &bus_state,
}};

static const struct wired_peripheral peripherals[] = {{
    .bus = &buses[0],
    .reg = 0,
}};

#else

// TODO: Error to link to docs
#error "Need to assign a 'zmk,split-uart` property to an enabled UART"

#endif

static void publish_events_work(struct k_work *work);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)

static void recv_data(const struct device *dev, const struct wired_bus *bus) {
    uint32_t last_read = 0, len = 0;
    struct ring_buf *buf = &bus->state->rx_buf;
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

    k_work_submit(&bus->state->event_publish_work);
}

static void send_data(const struct device *dev, const struct wired_bus *bus) {
    struct ring_buf *tx_buf = &bus->state->tx_buf;
    uint32_t len;
    while ((len = ring_buf_size_get(tx_buf)) > 0) {
        uint8_t *buf;
        uint32_t claim_len = ring_buf_get_claim(tx_buf, &buf, tx_buf->size);

        if (claim_len == 0) {
            continue;
        }

        int sent = uart_fifo_fill(dev, buf, claim_len);

        ring_buf_get_finish(tx_buf, MAX(sent, 0));
    }

    uart_irq_tx_disable(dev);
}

static void serial_cb(const struct device *dev, void *user_data) {
    const struct wired_bus *bus = (const struct wired_bus *)user_data;

    while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
        if (uart_irq_rx_ready(dev)) {
            recv_data(dev, bus);
        }

        if (uart_irq_tx_ready(dev)) {
            send_data(dev, bus);
        }
    }
}

#else

static void send_pending_tx(const struct wired_bus *bus) {
    struct ring_buf *tx_buf = &bus->state->tx_buf;
    uint8_t *buf;
    uint32_t claim_len;
    while ((claim_len = ring_buf_get_claim(tx_buf, &buf, tx_buf->size)) > 0) {
        LOG_HEXDUMP_DBG(buf, claim_len, "TX Bytes");
        for (int i = 0; i < claim_len; i++) {
            uart_poll_out(bus->uart, buf[i]);
        }

        ring_buf_get_finish(tx_buf, claim_len);
    }
}

static void read_pending_rx(const struct wired_bus *bus) {
    struct ring_buf *ring_buf = &bus->state->rx_buf;
    uint8_t *buf;
    uint32_t read = 0;
    uint32_t claim_len = ring_buf_put_claim(ring_buf, &buf, ring_buf_space_get(ring_buf));
    if (claim_len < 1) {
        LOG_WRN("No room available for reading in from the serial port");
        return;
    }

    while (read < claim_len) {
        if (uart_poll_in(bus->uart, buf + read) < 0) {
            break;
        }

        read++;
    }

    if (read > 0) {
        LOG_DBG("Read %d bytes", read);
    }
    ring_buf_put_finish(ring_buf, read);

    if (ring_buf_size_get(ring_buf) >= sizeof(struct event_envelope)) {
        k_work_submit(&bus->state->event_publish_work);
    }
}

static void central_main(void) {
    while (true) {
        for (size_t i = 0; i < ARRAY_SIZE(buses); i++) {
            read_pending_rx(&buses[i]);
            send_pending_tx(&buses[i]);
        }

        k_sleep(K_TICKS(1));
    }
}

K_THREAD_DEFINE(central_thread, CONFIG_ZMK_SPLIT_WIRED_THREAD_STACK_SIZE, central_main, NULL, NULL,
                NULL, K_HIGHEST_APPLICATION_THREAD_PRIO, 0, 0);

#endif

static int zmk_split_wired_central_init(void) {
    LOG_DBG("");
    for (size_t i = 0; i < ARRAY_SIZE(buses); i++) {
        if (!device_is_ready(buses[i].uart)) {
            return -ENODEV;
        }

        k_work_init(&buses[i].state->event_publish_work, publish_events_work);
        ring_buf_init(&buses[i].state->rx_buf, RX_BUFFER_SIZE, buses[i].state->rx_buffer);
        ring_buf_init(&buses[i].state->tx_buf, TX_BUFFER_SIZE, buses[i].state->tx_buffer);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)
        int ret = uart_irq_callback_user_data_set(buses[i].uart, serial_cb, (void *)&buses[i]);

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

        uart_irq_rx_enable(buses[i].uart);
#endif // IS_ENABLED(CONFIG_ZMK_SPLIT_WIRED_UART_MODE_DEFAULT_INTERRUPT)
    }
    return 0;
}

SYS_INIT(zmk_split_wired_central_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

static int split_central_bt_send_command(uint8_t source,
                                         struct zmk_split_transport_central_command cmd) {

    if (source >= ARRAY_SIZE(peripherals)) {
        return -EINVAL;
    }

    const struct wired_peripheral *peripheral = &peripherals[source];

    uint8_t *buffer;
    size_t len = ring_buf_put_claim(&peripheral->bus->state->tx_buf, &buffer,
                                    sizeof(struct command_envelope));

    if (len < sizeof(struct command_envelope)) {
        LOG_WRN("No room to send command to the peripheral %d", source);
        return -ENOSPC;
    }

    struct command_envelope env = {.source = source, .cmd = cmd};

    env.crc = crc32_ieee((void *)&env, sizeof(env) - 4);
    LOG_DBG("calculated a CRC for %d", env.crc);

    memcpy(buffer, &env, sizeof(env));

    ring_buf_put_finish(&peripheral->bus->state->tx_buf, len);
    uart_irq_tx_enable(peripheral->bus->uart);

    return 0;
}

static int split_central_bt_get_available_source_ids(uint8_t *sources) {
    int count = 0;

    return count;
}

static const struct zmk_split_transport_central_api central_api = {
    .send_command = split_central_bt_send_command,
    .get_available_source_ids = split_central_bt_get_available_source_ids,
};

ZMK_SPLIT_TRANSPORT_CENTRAL_REGISTER(wired_central, &central_api);

static void publish_events_work(struct k_work *work) {
    struct wired_bus_state *state = CONTAINER_OF(work, struct wired_bus_state, event_publish_work);

    while (ring_buf_size_get(&state->rx_buf) >= sizeof(struct event_envelope)) {
        struct event_envelope env;
        size_t bytes_left = sizeof(struct event_envelope);

        while (bytes_left > 0) {
            size_t read = ring_buf_get(&state->rx_buf, (uint8_t *)&env + (sizeof(env) - bytes_left),
                                       bytes_left);
            bytes_left -= read;
        }

        LOG_HEXDUMP_DBG(&env, sizeof(env), "Env data");

        // Exclude the trailing 4 bytes that contain the received CRC
        uint32_t crc = crc32_ieee((uint8_t *)&env, sizeof(env) - 4);
        if (crc != env.crc) {
            LOG_WRN("Data corruption in received peripheral event, ignoring");
            return;
        }

        zmk_split_transport_central_peripheral_event_handler(&wired_central, env.source, env.event);
    }
}
