/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * cmd_config.c
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
#include "cmd_config.h"
#include "config/config_storage.h"
#include "motion/motion_planner.h"
#include <stdio.h>

/* Configuration response and echo literals */
static const char RESP_CONFIG_FULL_FMT[] =
    "<RESP:CONFIG#L1=%.2f#L2=%.2f#Z_MIN=%.2f#Z_MAX=%.2f#MIN_SPD=%.2f#MAX_SPD="
    "%.2f#ACCEL=%.2f#MAX_ACCEL=%.2f#DEF_SPD=%.2f#OFF_J1=%.4f#OFF_J2=%.4f#RATE="
    "%u#J1_MIN=%.3f#J1_MAX=%.3f#J2_MIN=%.3f#J2_MAX=%.3f#GR_J1=%.2f#GR_J2=%.2f#"
    "GR_J4=%.2f#LEAD_Z=%.2f>\n";

static const char RESP_ACK_CONFIG_STORED_FMT[] =
    "<RESP:ACK#CONFIG_STORED#L1=%.2f#L2=%.2f#Z_MIN=%.2f#Z_MAX=%.2f#MIN_SPD=%."
    "2f#MAX_SPD=%.2f>\n";
static const char RESP_ACK_DYNAMICS_STORED_FMT[] =
    "<RESP:ACK#DYNAMICS_STORED#ACCEL=%.2f#MAX_ACCEL=%.2f#DEF_SPD=%.2f>\n";
static const char RESP_ACK_HOMING_STORED_FMT[] =
    "<RESP:ACK#HOMING_STORED#OFF_J1=%.4f#OFF_J2=%.4f#RATE=%u>\n";
static const char RESP_ACK_LIMITS_STORED_FMT[] =
    "<RESP:ACK#LIMITS_STORED#J1_MIN=%.3f#J1_MAX=%.3f#J2_MIN=%.3f#J2_MAX=%.3f>"
    "\n";
static const char RESP_ACK_STEPS_STORED_FMT[] =
    "<RESP:ACK#STEPS_STORED#GR_J1=%.2f#GR_J2=%.2f#GR_J4=%.2f#LEAD_Z=%.2f>\n";

static const char RESP_ACK_CONFIG_SAVED[] = "<RESP:ACK#CONFIG_SAVED>\n";
static const char RESP_ACK_CONFIG_RESET[] = "<RESP:ACK#CONFIG_RESET>\n";
static const char RESP_NACK_CONFIG_SAVE_FAILED[] =
    "<RESP:NACK_CONFIG_SAVE_FAILED>\n";

static const char RESP_NACK_INVALID_GEOMETRY[] =
    "<RESP:NACK_INVALID_CONFIG#PARAM=GEOMETRY>\n";
static const char RESP_NACK_INVALID_DYNAMICS[] =
    "<RESP:NACK_INVALID_CONFIG#PARAM=DYNAMICS>\n";
static const char RESP_NACK_INVALID_HOMING[] =
    "<RESP:NACK_INVALID_CONFIG#PARAM=HOMING>\n";
static const char RESP_NACK_INVALID_LIMITS[] =
    "<RESP:NACK_INVALID_CONFIG#PARAM=LIMITS>\n";
static const char RESP_NACK_INVALID_STEPS[] =
    "<RESP:NACK_INVALID_CONFIG#PARAM=STEPS>\n";
static const char RESP_ACK_ELBOW_FMT[] = "<RESP:ACK#ELBOW=%s>\n";
static const char RESP_ELBOW_CONFIG_FMT[] = "<RESP:ELBOW#CONFIG=%s>\n";
static const char ELBOW_STR_LEFT[] = "LEFT";
static const char ELBOW_STR_RIGHT[] = "RIGHT";

static void handle_get_config(void) {
  const scara_runtime_config_t *cfg = config_storage_get();
  printf(
      RESP_CONFIG_FULL_FMT, cfg->l1, cfg->l2, cfg->z_min, cfg->z_max,
      cfg->min_speed, cfg->max_speed, cfg->default_accel, cfg->max_accel,
      cfg->default_speed, cfg->home_offset_j1, cfg->home_offset_j2,
      (unsigned int)cfg->homing_rate_hz, cfg->j1_min_rad, cfg->j1_max_rad,
      cfg->j2_min_rad, cfg->j2_max_rad, cfg->gear_ratio_j1, cfg->gear_ratio_j2,
      cfg->gear_ratio_j4, cfg->leadscrew_pitch_z
  );
}

static void handle_set_config(const scara_command_t *cmd) {
  float l1 = cmd->data.config.l1;
  float l2 = cmd->data.config.l2;
  float z_min = cmd->data.config.z_min;
  float z_max = cmd->data.config.z_max;
  float min_spd = cmd->data.config.min_speed;
  float max_spd = cmd->data.config.max_speed;

  /* Validation */
  if (l1 <= 10.0f || l2 <= 10.0f || z_min >= z_max || min_spd <= 0.01f ||
      max_spd < min_spd) {
    printf("%s", RESP_NACK_INVALID_GEOMETRY);
    return;
  }

  scara_runtime_config_t cfg = *config_storage_get();
  cfg.l1 = l1;
  cfg.l2 = l2;
  cfg.z_min = z_min;
  cfg.z_max = z_max;
  cfg.min_speed = min_spd;
  cfg.max_speed = max_spd;

  config_storage_set(&cfg);
  motion_planner_init(cfg.l1, cfg.l2);

  if (config_storage_save_flash()) {
    printf(
        RESP_ACK_CONFIG_STORED_FMT, cfg.l1, cfg.l2, cfg.z_min, cfg.z_max,
        cfg.min_speed, cfg.max_speed
    );
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_set_dynamics(const scara_command_t *cmd) {
  float accel = cmd->data.dynamics.default_accel;
  float max_accel = cmd->data.dynamics.max_accel;
  float def_spd = cmd->data.dynamics.default_speed;

  /* Validation */
  if (accel <= 1.0f || max_accel < accel || def_spd <= 0.1f) {
    printf("%s", RESP_NACK_INVALID_DYNAMICS);
    return;
  }

  scara_runtime_config_t cfg = *config_storage_get();
  cfg.default_accel = accel;
  cfg.max_accel = max_accel;
  cfg.default_speed = def_spd;

  config_storage_set(&cfg);

  if (config_storage_save_flash()) {
    printf(
        RESP_ACK_DYNAMICS_STORED_FMT, cfg.default_accel, cfg.max_accel,
        cfg.default_speed
    );
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_set_homing(const scara_command_t *cmd) {
  float off_j1 = cmd->data.homing.home_offset_j1;
  float off_j2 = cmd->data.homing.home_offset_j2;
  uint32_t rate = cmd->data.homing.homing_rate_hz;

  /* Validation */
  if (off_j1 < -6.28f || off_j1 > 6.28f || off_j2 < -6.28f || off_j2 > 6.28f ||
      rate < 100U || rate > 20000U) {
    printf("%s", RESP_NACK_INVALID_HOMING);
    return;
  }

  scara_runtime_config_t cfg = *config_storage_get();
  cfg.home_offset_j1 = off_j1;
  cfg.home_offset_j2 = off_j2;
  cfg.homing_rate_hz = rate;

  config_storage_set(&cfg);

  if (config_storage_save_flash()) {
    printf(
        RESP_ACK_HOMING_STORED_FMT, cfg.home_offset_j1, cfg.home_offset_j2,
        (unsigned int)cfg.homing_rate_hz
    );
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_set_limits(const scara_command_t *cmd) {
  float j1_min = cmd->data.limits.j1_min_rad;
  float j1_max = cmd->data.limits.j1_max_rad;
  float j2_min = cmd->data.limits.j2_min_rad;
  float j2_max = cmd->data.limits.j2_max_rad;

  /* Validation */
  if (j1_min >= j1_max || j2_min >= j2_max) {
    printf("%s", RESP_NACK_INVALID_LIMITS);
    return;
  }

  scara_runtime_config_t cfg = *config_storage_get();
  cfg.j1_min_rad = j1_min;
  cfg.j1_max_rad = j1_max;
  cfg.j2_min_rad = j2_min;
  cfg.j2_max_rad = j2_max;

  config_storage_set(&cfg);

  if (config_storage_save_flash()) {
    printf(
        RESP_ACK_LIMITS_STORED_FMT, cfg.j1_min_rad, cfg.j1_max_rad,
        cfg.j2_min_rad, cfg.j2_max_rad
    );
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_set_steps(const scara_command_t *cmd) {
  float gr_j1 = cmd->data.steps.gear_ratio_j1;
  float gr_j2 = cmd->data.steps.gear_ratio_j2;
  float gr_j4 = cmd->data.steps.gear_ratio_j4;
  float lead_z = cmd->data.steps.leadscrew_pitch_z;

  /* Validation */
  if (gr_j1 <= 0.1f || gr_j1 > 100.0f || gr_j2 <= 0.1f || gr_j2 > 100.0f ||
      gr_j4 <= 0.1f || gr_j4 > 100.0f || lead_z <= 0.1f || lead_z > 100.0f) {
    printf("%s", RESP_NACK_INVALID_STEPS);
    return;
  }

  scara_runtime_config_t cfg = *config_storage_get();
  cfg.gear_ratio_j1 = gr_j1;
  cfg.gear_ratio_j2 = gr_j2;
  cfg.gear_ratio_j4 = gr_j4;
  cfg.leadscrew_pitch_z = lead_z;

  config_storage_set(&cfg);

  if (config_storage_save_flash()) {
    printf(
        RESP_ACK_STEPS_STORED_FMT, cfg.gear_ratio_j1, cfg.gear_ratio_j2,
        cfg.gear_ratio_j4, cfg.leadscrew_pitch_z
    );
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_save_config(void) {
  if (config_storage_save_flash()) {
    printf("%s", RESP_ACK_CONFIG_SAVED);
  } else {
    printf("%s", RESP_NACK_CONFIG_SAVE_FAILED);
  }
}

static void handle_reset_config(void) {
  config_storage_reset_defaults();
  printf("%s", RESP_ACK_CONFIG_RESET);
}

static void handle_set_elbow(const scara_command_t *cmd) {
  motion_planner_set_elbow_config(cmd->data.elbow.config);
  printf(
      RESP_ACK_ELBOW_FMT,
      cmd->data.elbow.config == SCARA_ELBOW_LEFT ? ELBOW_STR_LEFT
                                                 : ELBOW_STR_RIGHT
  );
}

static void handle_get_elbow(void) {
  printf(
      RESP_ELBOW_CONFIG_FMT,
      motion_planner_get_elbow_config() == SCARA_ELBOW_LEFT ? ELBOW_STR_LEFT
                                                            : ELBOW_STR_RIGHT
  );
}

bool cmd_config_handle(const scara_command_t *cmd) {
  if (!cmd) {
    return false;
  }

  switch (cmd->type) {
  case CMD_TYPE_GET_CONFIG:
    handle_get_config();
    return true;

  case CMD_TYPE_SET_CONFIG:
    handle_set_config(cmd);
    return true;

  case CMD_TYPE_SET_DYNAMICS:
    handle_set_dynamics(cmd);
    return true;

  case CMD_TYPE_SET_HOMING:
    handle_set_homing(cmd);
    return true;

  case CMD_TYPE_SET_LIMITS:
    handle_set_limits(cmd);
    return true;

  case CMD_TYPE_SET_STEPS:
    handle_set_steps(cmd);
    return true;

  case CMD_TYPE_SET_ELBOW:
    handle_set_elbow(cmd);
    return true;

  case CMD_TYPE_GET_ELBOW:
    handle_get_elbow();
    return true;

  case CMD_TYPE_SAVE_CONFIG:
    handle_save_config();
    return true;

  case CMD_TYPE_RESET_CONFIG:
    handle_reset_config();
    return true;

  default:
    return false;
  }
}
