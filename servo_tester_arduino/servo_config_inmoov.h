/*
===============================================================================
File:         servo_config_inmoov.h
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
Date:         2026-04-14
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

**************** Specific version for Inmoov project ****************

Multi-servo configuration file used by servo_tester_arduino.ino.

This file isolates servo characterization data from the main sketch so
the user can adapt the tester to different servos or articulations without
modifying the application logic.

The tester can cycle through all entries in this table by using the
NEXT SERVO pushbutton.

This version stores the configuration table in PROGMEM, so it lives in
Flash memory instead of SRAM. This is important when many servo profiles
are defined on an Arduino Nano / ATmega328.

It also includes optional current-sensor calibration parameters so the
library can remain generic and independent from a specific analog front-end.
Each servo may therefore define its own current-sensing pin, threshold,
timing, ADC offset and current conversion factor.

===============================================================================
*/

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include <avr/pgmspace.h>
#include "ServoController.h"

// Servo configuration table used by the tester.
//
// Field order:
// name, pwm_pin, servo_min_deg, servo_max_deg, allowed_min_deg, allowed_max_deg,
// rest_deg, pwm_min_us, pwm_max_us, max_speed_degps, default_speed_pct,
// default_accel_pct, feedback_adc_pin, fb_adc_at_servo_min_deg,
// fb_adc_at_servo_max_deg, current_adc_pin, current_limit_mA,
// overcurrent_time_ms, current_adc_offset, current_mA_per_count,
// inverted, fault_detection_enabled

const ServoConfig testerServoConfigs[] PROGMEM =
{
    {
        "Inmoov R-OMOPLATE", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        38,           // allowed_max_deg
        0,            // rest_deg
        677,          // pwm_min_us
        2350,         // pwm_max_us
        35.0f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-SHOULDER", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        45,           // allowed_min_deg
        150,          // allowed_max_deg
        70,           // rest_deg
        677,          // pwm_min_us
        2350,         // pwm_max_us
        20.0f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        true          // fault_detection_enabled
    },
    {
        "Inmoov R-ROTATE", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        60,           // allowed_min_deg
        145,          // allowed_max_deg
        105,          // rest_deg
        677,          // pwm_min_us
        2350,         // pwm_max_us
        20.0f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-BICEP", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        16,            // allowed_min_deg
        100,          // allowed_max_deg
        20,           // rest_deg
        677,          // pwm_min_us
        2350,         // pwm_max_us
        35.0f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        3,            // feedback_adc_pin (A3 on Arduino Nano)
        148,          // fb_adc_at_servo_min_deg
        568,          // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        1100,         // current_limit_mA
        500,          // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        true          // fault_detection_enabled
    },
    {
        "Inmoov R-F-THUMB", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        106,          // allowed_min_deg
        180,          // allowed_max_deg
        180,          // rest_deg
        578,          // pwm_min_us
        2300,         // pwm_max_us
        315.8f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-F-INDEX", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        070,          // allowed_min_deg
        160,          // allowed_max_deg
        150,          // rest_deg
        578,          // pwm_min_us
        2300,         // pwm_max_us
        315.8f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-F-MIDDLE", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        055,          // allowed_min_deg
        166,          // allowed_max_deg
        150,          // rest_deg
        578,          // pwm_min_us
        2300,         // pwm_max_us
        315.8f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-F-RING", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        060,          // allowed_min_deg
        162,          // allowed_max_deg
        160,          // rest_deg
        578,          // pwm_min_us
        2300,         // pwm_max_us
        315.8f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov R-F-PINKY", // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        075,          // allowed_min_deg
        168,          // allowed_max_deg
        155,          // rest_deg
        578,          // pwm_min_us
        2300,         // pwm_max_us
        315.8f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        6,            // current_adc_pin
        0,            // current_limit_mA
        0,            // overcurrent_time_ms
        499,          // current_adc_offset
        12.2f,        // current_mA_per_count
        false,        // inverted
        false         // fault_detection_enabled
    }
};

#endif
