/*
 * Copyright (c) 2023 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

void zmk_position_state_change_handle(struct zmk_position_state_changed *ev);

#if ZMK_KEYMAP_HAS_SENSORS
void zmk_sensor_event_handle(struct zmk_sensor_event *ev);
#endif
