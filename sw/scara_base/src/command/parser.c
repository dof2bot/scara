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

/* Command packet literal strings */
static const char CMD_STR_ENABLE[] = "<CMD:ENABLE>";
static const char CMD_STR_DISABLE[] = "<CMD:DISABLE>";
static const char CMD_STR_ESTOP[] = "<CMD:ESTOP>";
static const char CMD_STR_STATUS[] = "<CMD:STATUS>";
static const char CMD_STR_HOLD[] = "<CMD:HOLD>";
static const char CMD_STR_PAUSE[] = "<CMD:PAUSE>";
static const char CMD_STR_RESUME[] = "<CMD:RESUME>";
static const char CMD_STR_HOME[] = "<CMD:HOME>";
static const char CMD_STR_GETPOS[] = "<CMD:GETPOS>";
static const char CMD_STR_GET_CONFIG[] = "<CMD:GET_CONFIG>";
static const char CMD_STR_SAVE_CONFIG[] = "<CMD:SAVE_CONFIG>";
static const char CMD_STR_RESET_CONFIG[] = "<CMD:RESET_CONFIG>";
static const char CMD_STR_GET_ELBOW[] = "<CMD:GET_ELBOW>";
static const char CMD_STR_SET_ELBOW_LEFT[] = "<CMD:SET_ELBOW#LEFT>";
static const char CMD_STR_SET_ELBOW_RIGHT[] = "<CMD:SET_ELBOW#RIGHT>";
static const char CMD_STR_ELBOW_LEFT[] = "<CMD:ELBOW#LEFT>";
static const char CMD_STR_ELBOW_RIGHT[] = "<CMD:ELBOW#RIGHT>";
static const char CMD_STR_ELBOW_1[] = "<CMD:ELBOW#1>";
static const char CMD_STR_ELBOW_0[] = "<CMD:ELBOW#0>";

/* Format strings for sscanf */
static const char CMD_FMT_SET_CONFIG_LONG[] =
    "<CMD:SET_CONFIG#L1=%f#L2=%f#Z_MIN=%f#Z_MAX=%f#MIN_SPEED=%f#MAX_SPEED=%f>";
static const char CMD_FMT_SET_CONFIG_SHORT[] =
    "<CMD:SET_CONFIG#L1=%f#L2=%f#Z_MIN=%f#Z_MAX=%f#MIN_SPD=%f#MAX_SPD=%f>";
static const char CMD_FMT_SET_DYNAMICS_1[] =
    "<CMD:SET_DYNAMICS#ACCEL=%f#MAX_ACCEL=%f#DEF_SPD=%f>";
static const char CMD_FMT_SET_DYNAMICS_2[] =
    "<CMD:SET_DYNAMICS#ACCEL=%f#MAX_ACCEL=%f#DEF_SPEED=%f>";
static const char CMD_FMT_SET_HOMING[] =
    "<CMD:SET_HOMING#OFF_J1=%f#OFF_J2=%f#RATE=%u>";
static const char CMD_FMT_SET_LIMITS[] =
    "<CMD:SET_LIMITS#J1_MIN=%f#J1_MAX=%f#J2_MIN=%f#J2_MAX=%f>";
static const char CMD_FMT_SET_STEPS[] =
    "<CMD:SET_STEPS#GR_J1=%f#GR_J2=%f#GR_J4=%f#LEAD_Z=%f>";
static const char CMD_FMT_SETPOS[] =
    "<CMD:SETPOS#%f#%f#%f#%f#end>";
static const char CMD_FMT_JOG[] =
    "<CMD:JOG#%7[^#]#%f>";
static const char CMD_FMT_PUMP[] =
    "<CMD:PUMP#%d>";
static const char CMD_FMT_VALVE[] =
    "<CMD:VALVE#%d>";
static const char CMD_FMT_WAIT[] =
    "<CMD:WAIT#%u>";
static const char CMD_FMT_WAIT_ALT[] =
    "<wait#%u>";
static const char CMD_FMT_OVERRIDE[] =
    "<CMD:OVERRIDE#%u>";
static const char CMD_FMT_WAYPOINT_5D[] =
    "<pt#%f#%f#%f#%f#%f#end>";
static const char CMD_FMT_WAYPOINT_4D[] =
    "<pt#%f#%f#%f#%f#end>";

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
  if (strcmp(pkt, CMD_STR_ENABLE) == 0) {
    out_cmd->type = CMD_TYPE_ENABLE;
    return true;
  }

  if (strcmp(pkt, CMD_STR_DISABLE) == 0) {
    out_cmd->type = CMD_TYPE_DISABLE;
    return true;
  }

  if (strcmp(pkt, CMD_STR_ESTOP) == 0) {
    out_cmd->type = CMD_TYPE_ESTOP;
    return true;
  }

  if (strcmp(pkt, CMD_STR_STATUS) == 0) {
    out_cmd->type = CMD_TYPE_STATUS;
    return true;
  }

  if (strcmp(pkt, CMD_STR_HOLD) == 0 || strcmp(pkt, CMD_STR_PAUSE) == 0) {
    out_cmd->type = CMD_TYPE_HOLD;
    return true;
  }

  if (strcmp(pkt, CMD_STR_RESUME) == 0) {
    out_cmd->type = CMD_TYPE_RESUME;
    return true;
  }

  if (strcmp(pkt, CMD_STR_HOME) == 0) {
    out_cmd->type = CMD_TYPE_HOME;
    return true;
  }

  if (strcmp(pkt, CMD_STR_GETPOS) == 0) {
    out_cmd->type = CMD_TYPE_GETPOS;
    return true;
  }

  if (strcmp(pkt, CMD_STR_GET_CONFIG) == 0) {
    out_cmd->type = CMD_TYPE_GET_CONFIG;
    return true;
  }

  if (strcmp(pkt, CMD_STR_SAVE_CONFIG) == 0) {
    out_cmd->type = CMD_TYPE_SAVE_CONFIG;
    return true;
  }

  if (strcmp(pkt, CMD_STR_RESET_CONFIG) == 0) {
    out_cmd->type = CMD_TYPE_RESET_CONFIG;
    return true;
  }

  if (strcmp(pkt, CMD_STR_GET_ELBOW) == 0) {
    out_cmd->type = CMD_TYPE_GET_ELBOW;
    return true;
  }

  if (strcmp(pkt, CMD_STR_SET_ELBOW_LEFT) == 0 || strcmp(pkt, CMD_STR_ELBOW_LEFT) == 0 || strcmp(pkt, CMD_STR_ELBOW_1) == 0) {
    out_cmd->type = CMD_TYPE_SET_ELBOW;
    out_cmd->data.elbow.config = SCARA_ELBOW_LEFT;
    return true;
  }

  if (strcmp(pkt, CMD_STR_SET_ELBOW_RIGHT) == 0 || strcmp(pkt, CMD_STR_ELBOW_RIGHT) == 0 || strcmp(pkt, CMD_STR_ELBOW_0) == 0) {
    out_cmd->type = CMD_TYPE_SET_ELBOW;
    out_cmd->data.elbow.config = SCARA_ELBOW_RIGHT;
    return true;
  }

  /* Check for SET_CONFIG:
   * <CMD:SET_CONFIG#L1=%f#L2=%f#Z_MIN=%f#Z_MAX=%f#MIN_SPEED=%f#MAX_SPEED=%f> */
  float l1, l2, z_min, z_max, min_spd, max_spd;
  int cfg_match = sscanf(
      pkt, CMD_FMT_SET_CONFIG_LONG,
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

  /* Check short-form SET_CONFIG */
  cfg_match = sscanf(
      pkt, CMD_FMT_SET_CONFIG_SHORT,
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

  /* Check for SET_DYNAMICS: <CMD:SET_DYNAMICS#ACCEL=%f#MAX_ACCEL=%f#DEF_SPD=%f> */
  float d_accel, d_max_accel, d_def_spd;
  int dyn_match = sscanf(
      pkt, CMD_FMT_SET_DYNAMICS_1,
      &d_accel, &d_max_accel, &d_def_spd
  );
  if (dyn_match != 3) {
    dyn_match = sscanf(
        pkt, CMD_FMT_SET_DYNAMICS_2,
        &d_accel, &d_max_accel, &d_def_spd
    );
  }
  if (dyn_match == 3) {
    out_cmd->type = CMD_TYPE_SET_DYNAMICS;
    out_cmd->data.dynamics.default_accel = d_accel;
    out_cmd->data.dynamics.max_accel = d_max_accel;
    out_cmd->data.dynamics.default_speed = d_def_spd;
    return true;
  }

  /* Check for SET_HOMING: <CMD:SET_HOMING#OFF_J1=%f#OFF_J2=%f#RATE=%u> */
  float off_j1, off_j2;
  unsigned int h_rate;
  int home_match = sscanf(
      pkt, CMD_FMT_SET_HOMING,
      &off_j1, &off_j2, &h_rate
  );
  if (home_match == 3) {
    out_cmd->type = CMD_TYPE_SET_HOMING;
    out_cmd->data.homing.home_offset_j1 = off_j1;
    out_cmd->data.homing.home_offset_j2 = off_j2;
    out_cmd->data.homing.homing_rate_hz = (uint32_t)h_rate;
    return true;
  }

  /* Check for SET_LIMITS: <CMD:SET_LIMITS#J1_MIN=%f#J1_MAX=%f#J2_MIN=%f#J2_MAX=%f> */
  float j1_min, j1_max, j2_min, j2_max;
  int lim_match = sscanf(
      pkt, CMD_FMT_SET_LIMITS,
      &j1_min, &j1_max, &j2_min, &j2_max
  );
  if (lim_match == 4) {
    out_cmd->type = CMD_TYPE_SET_LIMITS;
    out_cmd->data.limits.j1_min_rad = j1_min;
    out_cmd->data.limits.j1_max_rad = j1_max;
    out_cmd->data.limits.j2_min_rad = j2_min;
    out_cmd->data.limits.j2_max_rad = j2_max;
    return true;
  }

  /* Check for SET_STEPS: <CMD:SET_STEPS#GR_J1=%f#GR_J2=%f#GR_J4=%f#LEAD_Z=%f> */
  float gr_j1, gr_j2, gr_j4, lead_z;
  int step_match = sscanf(
      pkt, CMD_FMT_SET_STEPS,
      &gr_j1, &gr_j2, &gr_j4, &lead_z
  );
  if (step_match == 4) {
    out_cmd->type = CMD_TYPE_SET_STEPS;
    out_cmd->data.steps.gear_ratio_j1 = gr_j1;
    out_cmd->data.steps.gear_ratio_j2 = gr_j2;
    out_cmd->data.steps.gear_ratio_j4 = gr_j4;
    out_cmd->data.steps.leadscrew_pitch_z = lead_z;
    return true;
  }

  /* Check for SETPOS: <CMD:SETPOS#X#Y#Z#PHI#end> */
  float sx, sy, sz, sphi;
  int set_match =
      sscanf(pkt, CMD_FMT_SETPOS, &sx, &sy, &sz, &sphi);

  if (set_match == 4) {
    out_cmd->type = CMD_TYPE_SETPOS;
    out_cmd->data.set_pose.x = sx;
    out_cmd->data.set_pose.y = sy;
    out_cmd->data.set_pose.z = sz;
    out_cmd->data.set_pose.phi = sphi;

    return true;
  }

  /* Check for JOG: <CMD:JOG#AXIS#STEP> */
  char axis_str[8];
  float jog_step;
  if (sscanf(pkt, CMD_FMT_JOG, axis_str, &jog_step) == 2) {
    out_cmd->type = CMD_TYPE_JOG;
    out_cmd->data.jog.axis = (char)toupper((unsigned char)axis_str[0]);
    out_cmd->data.jog.step = jog_step;
    return true;
  }

  /* Check for PUMP: <CMD:PUMP#1> or <CMD:PUMP#0> */
  int pump_val;
  if (sscanf(pkt, CMD_FMT_PUMP, &pump_val) == 1) {
    out_cmd->type = CMD_TYPE_PUMP;
    out_cmd->data.tool.enable = (pump_val != 0);
    return true;
  }

  /* Check for VALVE: <CMD:VALVE#1> or <CMD:VALVE#0> */
  int valve_val;
  if (sscanf(pkt, CMD_FMT_VALVE, &valve_val) == 1) {
    out_cmd->type = CMD_TYPE_VALVE;
    out_cmd->data.tool.enable = (valve_val != 0);
    return true;
  }

  /* Check for WAIT: <CMD:WAIT#ms> or <wait#ms> */
  unsigned int wait_ms;
  if (sscanf(pkt, CMD_FMT_WAIT, &wait_ms) == 1 ||
      sscanf(pkt, CMD_FMT_WAIT_ALT, &wait_ms) == 1) {
    out_cmd->type = CMD_TYPE_WAIT;
    out_cmd->data.wait.delay_ms = (uint32_t)wait_ms;
    return true;
  }

  /* Check for OVERRIDE: <CMD:OVERRIDE#val> */
  unsigned int ovr_val;
  if (sscanf(pkt, CMD_FMT_OVERRIDE, &ovr_val) == 1) {
    out_cmd->type = CMD_TYPE_OVERRIDE;
    out_cmd->data.override.percent = (uint8_t)(ovr_val > 200 ? 200 : ovr_val);
    return true;
  }

  /* Check for Waypoint: 5 fields <pt#X#Y#Z#PHI#SPEED#end> */
  float x, y, z, phi, spd;
  int matched = sscanf(pkt, CMD_FMT_WAYPOINT_5D, &x, &y, &z, &phi, &spd);

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
  matched = sscanf(pkt, CMD_FMT_WAYPOINT_4D, &x, &y, &z, &spd);

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
