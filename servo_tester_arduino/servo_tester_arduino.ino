/*
===============================================================================
Name:         servo_tester_arduino
Version:      1.1.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Single-servo tester using the ServoController library.

Hardware:

- Arduino Nano
- Target potentiometer       -> A2
- Speed potentiometer        -> A1
- Acceleration potentiometer -> A0
- Feedback input             -> A3
- Toggle pushbutton          -> D4
- Servo PWM output           -> D13
- OLED SSD1306 I2C display   -> A4 (SDA), A5 (SCL)

Main functions:

- Reads target angle, speed %, and acceleration % from potentiometers
- Drives one servo using ServoController
- Reads and displays feedback ADC and equivalent angle
- Uses a pushbutton as a toggle to enable / disable PWM output

OLED display lines:

1) Servo: <name>
2) PWM On / Off
3) PWM generated (us)
4) Commanded angle (deg)
5) Speed (%)
6) Acceleration (%)
7) Feedback ADC
8) Feedback angle (deg)

Important:

This sketch assumes the ServoController library has three public methods:
- enableOutput()
- disableOutput()
- isOutputEnabled()

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "ServoController.h"
#include "servo_config.h"

// ============================================================================
// Pins
// ============================================================================

#define POT_ACCEL_PIN      A0
#define POT_SPEED_PIN      A1
#define POT_TARGET_PIN     A2
#define POT_FEEDBACK_PIN   A3

#define TOGGLE_BUTTON_PIN  4

// ============================================================================
// OLED configuration
// ============================================================================

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// ADC helpers
// ============================================================================

#define ADC_SCALE   1023.0f
#define POT_SAMPLES 4

// ============================================================================
// Button debounce
// ============================================================================

#define DEBOUNCE_MS 40

// ============================================================================
// Global objects / state
// ============================================================================

ServoController servo;

// PWM starts disabled by default.
bool pwmEnabled = false;

bool lastButtonReading = HIGH;
bool stableButtonState = HIGH;
unsigned long lastDebounceTime = 0;

// ============================================================================
// Utility helpers
// ============================================================================

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

float readAveragedADC(uint8_t pin, int samples)
{
    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / samples;
}

float targetDegFromAdc(float adc)
{
    float deg = (adc / ADC_SCALE) * 180.0f;
    return clampf(deg, 0.0f, 180.0f);
}

uint8_t percentFromAdc(float adc)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

    int pct = (int)(1.0f + t * 99.0f + 0.5f);

    if (pct < 1)   pct = 1;
    if (pct > 100) pct = 100;

    return (uint8_t)pct;
}

// Converts the raw feedback ADC into angle using the calibration
// stored in testerServoConfig. This is shown directly on the display so the
// user can observe both the raw ADC and its equivalent angle.
float feedbackAngleFromAdc(int adc)
{
    long adcMin = testerServoConfig.fb_adc_at_servo_min_deg;
    long adcMax = testerServoConfig.fb_adc_at_servo_max_deg;

    if (adcMax == adcMin)
    {
        return testerServoConfig.servo_min_deg;
    }

    float ratio = (float)(adc - adcMin) / (float)(adcMax - adcMin);

    float angle =
        testerServoConfig.servo_min_deg +
        ratio * (testerServoConfig.servo_max_deg - testerServoConfig.servo_min_deg);

    return clampf(angle,
                  testerServoConfig.servo_min_deg,
                  testerServoConfig.servo_max_deg);
}

// Debounced toggle button using INPUT_PULLUP.
// Returns true only once per valid press.
bool buttonPressedEvent()
{
    bool reading = digitalRead(TOGGLE_BUTTON_PIN);

    if (reading != lastButtonReading)
    {
        lastDebounceTime = millis();
        lastButtonReading = reading;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_MS)
    {
        if (reading != stableButtonState)
        {
            stableButtonState = reading;

            if (stableButtonState == LOW)
            {
                return true;
            }
        }
    }

    return false;
}

// ============================================================================
// OLED display
// ============================================================================

void drawDisplay(const char* name,
                 bool pwmOn,
                 int pwmUs,
                 float cmdDeg,
                 uint8_t speedPct,
                 uint8_t accelPct,
                 int fbAdc,
                 float fbDeg)
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.print("Servo: ");
    display.println(name);

    display.print("PWM ");
    display.println(pwmOn ? "On" : "Off");

    display.print("PWMus ");
    display.println(pwmUs);

    display.print("Cmd ");
    display.println(cmdDeg, 1);

    display.print("V% ");
    display.println(speedPct);

    display.print("A% ");
    display.println(accelPct);

    display.print("ADC ");
    display.println(fbAdc);

    display.print("Ang ");
    display.println(fbDeg, 1);

    display.display();
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    pinMode(TOGGLE_BUTTON_PIN, INPUT_PULLUP);

    // OLED init
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();

    // Initialize the servo controller with the static configuration
    servo.begin(testerServoConfig);

    // Synchronize initial software state to real feedback position
    servo.syncToFeedback();

    // Start with PWM disabled by default
    servo.disableOutput();
    pwmEnabled = false;
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    // Handle pushbutton toggle
    if (buttonPressedEvent())
    {
        pwmEnabled = !pwmEnabled;

        if (pwmEnabled)
        {
            // When enabling PWM again, resynchronize to the current measured
            // position to minimize jumps at restart.
            servo.enableOutput();
            servo.syncToFeedback();
        }
        else
        {
            servo.disableOutput();
        }
    }

    // Read user controls
    float adcTarget = readAveragedADC(POT_TARGET_PIN, POT_SAMPLES);
    float adcSpeed  = readAveragedADC(POT_SPEED_PIN,  POT_SAMPLES);
    float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  POT_SAMPLES);

    float   targetDeg = targetDegFromAdc(adcTarget);
    uint8_t speedPct  = percentFromAdc(adcSpeed);
    uint8_t accelPct  = percentFromAdc(adcAccel);

    // Read feedback directly for display purposes
    int   fbAdc = (int)(readAveragedADC(POT_FEEDBACK_PIN, POT_SAMPLES) + 0.5f);
    float fbDeg = feedbackAngleFromAdc(fbAdc);

    // If PWM is enabled, command and update the controller
    if (pwmEnabled)
    {
        servo.setTarget(targetDeg, speedPct, accelPct);
        servo.update();
    }

    // Update display
    drawDisplay(
        testerServoConfig.name,
        pwmEnabled,
        servo.getPwmUs(),
        servo.getCommandDeg(),
        speedPct,
        accelPct,
        fbAdc,
        fbDeg
    );

    delay(20);
}
