/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * io_gpio.h
 * Copyright (C) 2026 Vladimir Roncevic <elektron.ronca@gmail.com>
 *
 * scara-base is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * scara-base is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "scara_config.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes board status LED and auxiliary GPIOs.
 * @return true on success, false on error.
 */
bool io_gpio_init(void);

/**
 * @brief Sets on-board status LED state.
 * @param state true for LED on, false for LED off.
 */
void io_gpio_set_led(bool state);

/**
 * @brief Toggles on-board status LED state.
 */
void io_gpio_toggle_led(void);

/**
 * @brief Reads limit switch status.
 * @param endstop_pin Pin number of endstop.
 * @return true if triggered, false if open.
 */
bool io_gpio_read_endstop(uint32_t endstop_pin);

#ifdef __cplusplus
}
#endif
