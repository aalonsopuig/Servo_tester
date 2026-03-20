/*
===============================================================================
File:         servo_config.h
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-15
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Multi-servo configuration file used by servo_tester_arduino.ino.

This file isolates servo characterization data from the main sketch so
the user can adapt the tester to different servos or articulations without
modifying the application logic.

The tester can cycle through all entries in this table by using the
NEXT SERVO pushbutton.

This version stores the configuration table in PROGMEM, so it lives in
Flash memory instead of SRAM. This is important when many servo profiles
are defined on an Arduino Nano / ATmega328.

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
// fb_adc_at_servo_max_deg, inverted, fault_detection_enabled

const ServoConfig testerServoConfigs[] PROGMEM =
{
    {
        "UNKNOWN",    // name
        9,            // pwm_pin
        0,            // servo_min_deg   -> unknown
        0,            // servo_max_deg   -> unknown
        0,            // allowed_min_deg -> unknown
        0,            // allowed_max_deg -> unknown
        0,            // rest_deg        -> unknown
        500,          // pwm_min_us
        2500,         // pwm_max_us
        0.0f,         // max_speed_degps -> unknown
        0,            // default_speed_pct -> unused
        0,            // default_accel_pct -> unused
        3,            // feedback_adc_pin  (A3 on Arduino Nano)
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Hirec HS-805BB",    // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        180,          // allowed_max_deg
        90,           // rest_deg
        677,          // pwm_min_us
        2350,         // pwm_max_us
        428.6f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "TowerPro SG-5010",    // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        180,          // allowed_max_deg
        90,           // rest_deg
        500,          // pwm_min_us
        1806,         // pwm_max_us
        375.0f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Futaba S3003", // name
        9,                // pwm_pin
        0,                // servo_min_deg
        180,              // servo_max_deg
        0,                // allowed_min_deg
        180,              // allowed_max_deg
        90,               // rest_deg
        677,              // pwm_min_us
        2350,             // pwm_max_us
        428.6f,           // max_speed_degps
        100,              // default_speed_pct
        100,              // default_accel_pct
        -1,               // feedback_adc_pin disabled
        0,                // fb_adc_at_servo_min_deg
        0,                // fb_adc_at_servo_max_deg
        false,            // inverted
        false             // fault_detection_enabled
    },
    {
        "DIYMore DM996",    // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        180,          // allowed_max_deg
        90,           // rest_deg
        559,          // pwm_min_us
        2472,         // pwm_max_us
        400.0f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Miuzei MG90",    // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        180,          // allowed_max_deg
        90,           // rest_deg
        561,          // pwm_min_us
        2500,         // pwm_max_us
        750.0f,       // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        -1,           // feedback_adc_pin disabled
        0,            // fb_adc_at_servo_min_deg
        0,            // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    },
    {
        "Inmoov SHOULDER-R",   // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        0,            // allowed_min_deg
        180,          // allowed_max_deg
        90,           // rest_deg
        677,              // pwm_min_us
        2350,             // pwm_max_us
        17.5f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        3,            // feedback_adc_pin  (A3 on Arduino Nano)
        148,          // fb_adc_at_servo_min_deg
        568,          // fb_adc_at_servo_max_deg
        false,        // inverted
        true         // fault_detection_enabled
    },
    {
        "Inmoov BICEP-R",   // name
        9,            // pwm_pin
        0,            // servo_min_deg
        180,          // servo_max_deg
        16,            // allowed_min_deg
        100,          // allowed_max_deg
        20,           // rest_deg
        677,              // pwm_min_us
        2350,             // pwm_max_us
        35.0f,        // max_speed_degps
        100,          // default_speed_pct
        100,          // default_accel_pct
        3,            // feedback_adc_pin  (A3 on Arduino Nano)
        148,          // fb_adc_at_servo_min_deg
        568,          // fb_adc_at_servo_max_deg
        false,        // inverted
        true         // fault_detection_enabled
    }
};

#endif
