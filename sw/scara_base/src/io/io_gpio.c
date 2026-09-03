/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * io_gpio.c
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
#include "io_gpio.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

static bool led_state = false;

bool io_gpio_init(void) {
  /* Initialize status LED */
  gpio_init(SCARA_STATUS_LED_PIN);
  gpio_set_dir(SCARA_STATUS_LED_PIN, GPIO_OUT);
  gpio_put(SCARA_STATUS_LED_PIN, 0);
  led_state = false;

  /* Initialize Endstops with internal pull-ups */
  const uint endstop_pins[] = {
      SCARA_ENDSTOP_X_PIN, SCARA_ENDSTOP_Y_PIN, SCARA_ENDSTOP_Z_PIN
  };

  for (size_t i = 0; i < 3; i++) {
    gpio_init(endstop_pins[i]);
    gpio_set_dir(endstop_pins[i], GPIO_IN);
    gpio_pull_up(endstop_pins[i]);
  }

  /* Initialize Tool GPIOs (Pump & Valve) */
  gpio_init(SCARA_TOOL_PUMP_PIN);
  gpio_set_dir(SCARA_TOOL_PUMP_PIN, GPIO_OUT);
  gpio_put(SCARA_TOOL_PUMP_PIN, 0);

  gpio_init(SCARA_TOOL_VALVE_PIN);
  gpio_set_dir(SCARA_TOOL_VALVE_PIN, GPIO_OUT);
  gpio_put(SCARA_TOOL_VALVE_PIN, 0);

  return true;
}

void io_gpio_set_led(bool state) {
  led_state = state;
  gpio_put(SCARA_STATUS_LED_PIN, state ? 1 : 0);
}

void io_gpio_toggle_led(void) {
  io_gpio_set_led(!led_state);
}

bool io_gpio_read_endstop(uint32_t endstop_pin) {
  /* Active LOW when switch hits GND */
  return gpio_get(endstop_pin) == 0;
}

void io_gpio_set_pump(bool state) {
  gpio_put(SCARA_TOOL_PUMP_PIN, state ? 1 : 0);
}

void io_gpio_set_valve(bool state) {
  gpio_put(SCARA_TOOL_VALVE_PIN, state ? 1 : 0);
}
