#pragma once

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