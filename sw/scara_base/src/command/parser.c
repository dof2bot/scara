/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * parser.c
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
#include "parser.h"
#include "pico/stdlib.h"
#include <ctype.h>
#include <stdio.h>
#include <string.h>

static char rx_buf[PARSER_RX_BUFFER_SIZE];
static size_t rx_idx = 0;

void parser_init(void) {
  rx_idx = 0;
  memset(rx_buf, 0, sizeof(rx_buf));
}

static bool parse_packet_string(const char *pkt, scara_command_t *out_cmd) {
  if (!pkt || !out_cmd) {
    return false;
  }

  out_cmd->type = CMD_TYPE_NONE;

  /* Check command types */
  if (strcmp(pkt, "<CMD:ENABLE>") == 0) {
    out_cmd->type = CMD_TYPE_ENABLE;
    return true;
  }

  if (strcmp(pkt, "<CMD:DISABLE>") == 0) {
    out_cmd->type = CMD_TYPE_DISABLE;
    return true;
  }

  if (strcmp(pkt, "<CMD:ESTOP>") == 0) {
    out_cmd->type = CMD_TYPE_ESTOP;
    return true;
  }

  if (strcmp(pkt, "<CMD:STATUS>") == 0) {
    out_cmd->type = CMD_TYPE_STATUS;
    return true;
  }

  if (strcmp(pkt, "<CMD:HOME>") == 0) {
    out_cmd->type = CMD_TYPE_HOME;
    return true;
  }

  if (strcmp(pkt, "<CMD:GETPOS>") == 0) {
    out_cmd->type = CMD_TYPE_GETPOS;
    return true;
  }

  if (strcmp(pkt, "<CMD:GET_CONFIG>") == 0) {
    out_cmd->type = CMD_TYPE_GET_CONFIG;
    return true;
  }

  if (strcmp(pkt, "<CMD:SAVE_CONFIG>") == 0) {
    out_cmd->type = CMD_TYPE_SAVE_CONFIG;
    return true;
  }

  if (strcmp(pkt, "<CMD:RESET_CONFIG>") == 0) {
    out_cmd->type = CMD_TYPE_RESET_CONFIG;
    return true;
  }

  /* Check for SET_CONFIG: <CMD:SET_CONFIG#L1=%f#L2=%f#Z_MIN=%f#Z_MAX=%f#MIN_SPEED=%f#MAX_SPEED=%f> */
  float l1, l2, z_min, z_max, min_spd, max_spd;
  int cfg_match = sscanf(
      pkt,
      "<CMD:SET_CONFIG#L1=%f#L2=%f#Z_MIN=%f#Z_MAX=%f#MIN_SPEED=%f#MAX_SPEED=%f>",
      &l1, &l2, &z_min, &z_max, &min_spd, &max_spd
  );

  if (cfg_match == 6) {
    out_cmd->type = CMD_TYPE_SET_CONFIG;
    out_cmd->data.config.l1 = l1;
    out_cmd->data.config.l2 = l2;
    out_cmd->data.config.z_min = z_min;
    out_cmd->data.config.z_max = z_max;
    out_cmd->data.config.min_speed = min_spd;
    out_cmd->data.config.max_speed = max_spd;
    return true;
  }

  /* Check for SETPOS: <CMD:SETPOS#X#Y#Z#PHI#end> */
  float sx, sy, sz, sphi;
  int set_match =
      sscanf(pkt, "<CMD:SETPOS#%f#%f#%f#%f#end>", &sx, &sy, &sz, &sphi);

  if (set_match == 4) {
    out_cmd->type = CMD_TYPE_SETPOS;
    out_cmd->data.set_pose.x = sx;
    out_cmd->data.set_pose.y = sy;
    out_cmd->data.set_pose.z = sz;
    out_cmd->data.set_pose.phi = sphi;

    return true;
  }

  /* Check for Waypoint: 5 fields <pt#X#Y#Z#PHI#SPEED#end> */
  float x, y, z, phi, spd;
  int matched = sscanf(pkt, "<pt#%f#%f#%f#%f#%f#end>", &x, &y, &z, &phi, &spd);

  if (matched == 5) {
    out_cmd->type = CMD_TYPE_WAYPOINT;
    out_cmd->data.waypoint.x = x;
    out_cmd->data.waypoint.y = y;
    out_cmd->data.waypoint.z = z;
    out_cmd->data.waypoint.phi = phi;
    out_cmd->data.waypoint.speed = spd;

    return true;
  }

  /* Check for Waypoint: 4 fields <pt#X#Y#Z#SPEED#end> (phi defaults to 0) */
  matched = sscanf(pkt, "<pt#%f#%f#%f#%f#end>", &x, &y, &z, &spd);

  if (matched == 4) {
    out_cmd->type = CMD_TYPE_WAYPOINT;
    out_cmd->data.waypoint.x = x;
    out_cmd->data.waypoint.y = y;
    out_cmd->data.waypoint.z = z;
    out_cmd->data.waypoint.phi = 0.0f;
    out_cmd->data.waypoint.speed = spd;

    return true;
  }

  return false;
}

bool parser_poll(scara_command_t *out_cmd) {
  if (!out_cmd) {
    return false;
  }

  int ch = getchar_timeout_us(0);

  while (ch != PICO_ERROR_TIMEOUT) {
    char c = (char)ch;

    if (c == '<') {
      rx_idx = 0;
      rx_buf[rx_idx++] = c;
    } else if (c == '>') {
      if (rx_idx < PARSER_RX_BUFFER_SIZE - 1) {
        rx_buf[rx_idx++] = c;
        rx_buf[rx_idx] = '\0';

        bool ok = parse_packet_string(rx_buf, out_cmd);
        rx_idx = 0;

        if (ok) {
          return true;
        }

      } else {
        rx_idx = 0;
      }
    } else if (rx_idx > 0 && rx_idx < PARSER_RX_BUFFER_SIZE - 1) {
      rx_buf[rx_idx++] = c;
    }

    ch = getchar_timeout_us(0);
  }

  return false;
}
