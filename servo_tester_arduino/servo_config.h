/*
===============================================================================
File:         servo_config.h
Author:       Alejandro Alonso Puig + GPT
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Single-servo configuration file used by servo_tester_arduino.ino.

This file isolates the servo characterization data from the main sketch so
the user can adapt the tester to another servo or articulation without
modifying the application logic.

===============================================================================
*/

#ifndef SERVO_CONFIG_H
#define SERVO_CONFIG_H

#include "ServoController.h"

// Single servo configuration used by the tester.
//
// Field order:
// name, pwm_pin, servo_min_deg, servo_max_deg, allowed_min_deg, allowed_max_deg,
// rest_deg, pwm_min_us, pwm_max_us, max_speed_degps, default_speed_pct,
// default_accel_pct, feedback_adc_pin, fb_adc_at_servo_min_deg,
// fb_adc_at_servo_max_deg, inverted, fault_detection_enabled

static ServoConfig testerServoConfig =
{
    "SHOULDER",   // name
    13,           // pwm_pin
    0,            // servo_min_deg
    180,          // servo_max_deg
    0,            // allowed_min_deg
    180,          // allowed_max_deg
    90,           // rest_deg
    700,          // pwm_min_us
    2400,         // pwm_max_us
    17.5f,        // max_speed_degps
    100,          // default_speed_pct
    100,          // default_accel_pct
    3,            // feedback_adc_pin  (A3 on Arduino Nano)
    101,          // fb_adc_at_servo_min_deg
    383,          // fb_adc_at_servo_max_deg
    false,        // inverted
    false         // fault_detection_enabled
};

#endif