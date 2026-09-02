/* -*- Mode: C; indent-tabs-mode: nil; c-basic-offset: 2; tab-width: 2 -*- */
/*
 * scara_config.h
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

#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define RAD_TO_DEG (180.0f / (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

/* ========================================================================= */
/*                      SCARA Arm Physical Geometry                          */
/* ========================================================================= */
#define SCARA_ARM_L1_MM 150.0f /* Length of inner arm L1 (mm) */
#define SCARA_ARM_L2_MM 120.0f /* Length of outer arm L2 (mm) */
#define SCARA_Z_MIN_MM 0.0f    /* Minimum Z travel (mm) */
#define SCARA_Z_MAX_MM 100.0f  /* Maximum Z travel (mm) */

/* ========================================================================= */
/*                      Stepper & Transmission Parameters                    */
/* ========================================================================= */
#define SCARA_STEPS_PER_REV 200.0f /* 1.8 degree stepper motors */
#define SCARA_MICROSTEPPING 16.0f  /* 1/16 microstepping */

#define SCARA_GEAR_RATIO_J1 4.0f /* Pulley ratio for Joint 1 (Shoulder) */
#define SCARA_GEAR_RATIO_J2 4.0f /* Pulley ratio for Joint 2 (Elbow) */
#define SCARA_GEAR_RATIO_J4 1.0f /* Direct / pulley ratio for Joint 4 (Wrist)  \
                                  */
#define SCARA_LEADSCREW_PITCH_Z 8.0f /* T8 leadscrew pitch: 8.0 mm / rev */

/* Precomputed step scale factors */
#define SCARA_STEPS_PER_RAD_J1                                                 \
  ((SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * SCARA_GEAR_RATIO_J1) /         \
   (2.0f * (float)M_PI))
#define SCARA_STEPS_PER_RAD_J2                                                 \
  ((SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * SCARA_GEAR_RATIO_J2) /         \
   (2.0f * (float)M_PI))
#define SCARA_STEPS_PER_RAD_J4                                                 \
  ((SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING * SCARA_GEAR_RATIO_J4) /         \
   (2.0f * (float)M_PI))
#define SCARA_STEPS_PER_MM_Z                                                   \
  ((SCARA_STEPS_PER_REV * SCARA_MICROSTEPPING) / SCARA_LEADSCREW_PITCH_Z)

/* ========================================================================= */
/*                      Default Speeds & Trajectory Settings                 */
/* ========================================================================= */
#define SCARA_DEFAULT_SPEED_MM_S 50.0f /* Default Cartesian feedrate (mm/s) */
#define SCARA_MAX_SPEED_MM_S 250.0f    /* Max linear speed (mm/s) */
#define SCARA_SEGMENT_LEN_MM 0.5f      /* Cartesian segmentation step (mm) */
#define SCARA_MAX_SEGMENTS_CHUNK 256 /* Segment chunk capacity for DMA buffer  \
                                      */

/* ========================================================================= */
/*                      Hardware Pinout Profiles                             */
/* ========================================================================= */
/* Profile 1: SKR Pico V1.0 Onboard TMC2209 */
#if defined(BOARD_BTT_SKR_PICO)
#define SCARA_J1_STEP_PIN 11
#define SCARA_J1_DIR_PIN 10
#define SCARA_J2_STEP_PIN 6
#define SCARA_J2_DIR_PIN 5
#define SCARA_Z_STEP_PIN 19
#define SCARA_Z_DIR_PIN 28
#define SCARA_J4_STEP_PIN 14
#define SCARA_J4_DIR_PIN 13
#define SCARA_ENABLE_PIN 12 /* Active LOW for all steppers */
#define SCARA_TMC_UART_PIN 9
#define SCARA_TMC_TX_PIN 8

/* Profile 2: Standard Raspberry Pi Pico GPIO Pinout */
#else
#define SCARA_J1_STEP_PIN 2
#define SCARA_J1_DIR_PIN 3
#define SCARA_J2_STEP_PIN 4
#define SCARA_J2_DIR_PIN 5
#define SCARA_Z_STEP_PIN 6
#define SCARA_Z_DIR_PIN 7
#define SCARA_J4_STEP_PIN 8
#define SCARA_J4_DIR_PIN 9
#define SCARA_ENABLE_PIN 12 /* Active LOW */
#endif

/* Status indicators and Limit Switches */
#define SCARA_STATUS_LED_PIN 25 /* On-board LED */
#define SCARA_ENDSTOP_X_PIN 16  /* J1 limit switch */
#define SCARA_ENDSTOP_Y_PIN 17  /* J2 limit switch */
#define SCARA_ENDSTOP_Z_PIN 18  /* Z limit switch */
