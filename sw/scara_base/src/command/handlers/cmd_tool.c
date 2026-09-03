/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * cmd_tool.c
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
#include "cmd_tool.h"
#include "io/io_gpio.h"
#include <stdio.h>

/* Tool response literals */
static const char RESP_ACK_PUMP_ON[] = "<RESP:ACK#PUMP_ON>\n";
static const char RESP_ACK_PUMP_OFF[] = "<RESP:ACK#PUMP_OFF>\n";
static const char RESP_ACK_VALVE_ON[] = "<RESP:ACK#VALVE_ON>\n";
static const char RESP_ACK_VALVE_OFF[] = "<RESP:ACK#VALVE_OFF>\n";

static void handle_pump(const scara_command_t *cmd) {
  io_gpio_set_pump(cmd->data.tool.enable);
  if (cmd->data.tool.enable) {
    printf("%s", RESP_ACK_PUMP_ON);
  } else {
    printf("%s", RESP_ACK_PUMP_OFF);
  }
}

static void handle_valve(const scara_command_t *cmd) {
  io_gpio_set_valve(cmd->data.tool.enable);
  if (cmd->data.tool.enable) {
    printf("%s", RESP_ACK_VALVE_ON);
  } else {
    printf("%s", RESP_ACK_VALVE_OFF);
  }
}

bool cmd_tool_handle(const scara_command_t *cmd) {
  if (!cmd) {
    return false;
  }

  switch (cmd->type) {
  case CMD_TYPE_PUMP:
    handle_pump(cmd);
    return true;

  case CMD_TYPE_VALVE:
    handle_valve(cmd);
    return true;

  default:
    return false;
  }
}
