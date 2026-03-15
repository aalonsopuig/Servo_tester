/*
===============================================================================
Name:         servo_tester_arduino
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-10
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Adaptive single-/multi-servo tester.

The sketch automatically adapts its behaviour and the OLED contents to the
amount of information available in the active servo configuration.

Possible levels of available information:

1) Only PWM range known
   - direct PWM test in microseconds
   - no angle shown

2) Servo angular range known
   - PWMang can be shown
   - PWMus can be computed from the selected angle

3) Allowed range known
   - target angle is constrained to the allowed range
   - allowed range is displayed

4) Max speed known
   - ServoController can be used
   - V% and A% become meaningful and are displayed

5) Feedback pin known
   - raw ADC is displayed

6) Feedback calibration known
   - ADC_ang can also be displayed

Multiple servo configurations can be defined in servo_config.h.
A button on D3 cycles through them. Whenever the active servo changes, PWM is
forced OFF first and the whole tester state is rebuilt for the new servo.

Hardware:

- Arduino Nano
- Acceleration potentiometer -> A0
- Speed potentiometer        -> A1
- Target potentiometer       -> A2
- Feedback input             -> defined in servo_config.h
- PWM ON/OFF pushbutton      -> D4
- NEXT SERVO pushbutton      -> D3
- Servo PWM output           -> defined in servo_config.h
- OLED SSD1306 I2C display   -> A4 (SDA), A5 (SCL)

Important:

This sketch assumes both pushbuttons have an EXTERNAL pull-down resistor:
- released = LOW
- pressed  = HIGH

Expected servo_config.h symbols:

- testerServoConfigs[]

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "ServoController.h"
#include "servo_config.h"

// ============================================================================
// Pins
// ============================================================================

#define POT_ACCEL_PIN          A0
#define POT_SPEED_PIN          A1
#define POT_TARGET_PIN         A2

#define PWM_TOGGLE_BUTTON_PIN  4
#define NEXT_SERVO_BUTTON_PIN  3

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

ServoController servoCtrl;
Servo rawServo;

// Number of servo configurations available.
// This avoids having to manually maintain TESTER_SERVO_COUNT.
const uint8_t testerServoCount =
    sizeof(testerServoConfigs) / sizeof(testerServoConfigs[0]);

uint8_t currentServoIndex = 0;
const ServoConfig* activeCfg = nullptr;

bool pwmEnabled = false;

bool lastPwmButtonReading = LOW;
bool stablePwmButtonState = LOW;
unsigned long lastPwmDebounceTime = 0;

bool lastNextButtonReading = LOW;
bool stableNextButtonState = LOW;
unsigned long lastNextDebounceTime = 0;

// Cached display values.
uint8_t displaySpeedPct  = 0;
uint8_t displayAccelPct  = 0;
float   displayPwmAngDeg = 0.0f;
int     displayPwmUs     = 0;

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

uint8_t percentFromAdc(float adc)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

    int pct = (int)(1.0f + t * 99.0f + 0.5f);

    if (pct < 1)   pct = 1;
    if (pct > 100) pct = 100;

    return (uint8_t)pct;
}

int pwmUsFromAdc(float adc, int pwmMin, int pwmMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float us = pwmMin + t * (pwmMax - pwmMin);

    if (us < pwmMin) us = pwmMin;
    if (us > pwmMax) us = pwmMax;

    return (int)(us + 0.5f);
}

// Map target potentiometer to an angle range [angleMin..angleMax].
float targetDegFromAdcRange(float adc, float angleMin, float angleMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float deg = angleMin + t * (angleMax - angleMin);

    if (deg < angleMin) deg = angleMin;
    if (deg > angleMax) deg = angleMax;

    return deg;
}

// Local conversion from logical angle to PWM microseconds.
int pwmUsFromAngle(const ServoConfig& cfg, float angleDeg)
{
    float logicalDeg = clampf(angleDeg, cfg.servo_min_deg, cfg.servo_max_deg);

    float physicalDeg;
    if (!cfg.inverted)
    {
        physicalDeg = logicalDeg;
    }
    else
    {
        physicalDeg = cfg.servo_max_deg - (logicalDeg - cfg.servo_min_deg);
    }

    float spanDeg = cfg.servo_max_deg - cfg.servo_min_deg;
    if (spanDeg == 0.0f)
    {
        return cfg.pwm_min_us;
    }

    float ratio = (physicalDeg - cfg.servo_min_deg) / spanDeg;
    float pwm = cfg.pwm_min_us + ratio * (cfg.pwm_max_us - cfg.pwm_min_us);

    if (pwm < cfg.pwm_min_us) pwm = cfg.pwm_min_us;
    if (pwm > cfg.pwm_max_us) pwm = cfg.pwm_max_us;

    return (int)(pwm + 0.5f);
}

float feedbackAngleFromAdc(const ServoConfig& cfg, int adc)
{
    long adcMin = cfg.fb_adc_at_servo_min_deg;
    long adcMax = cfg.fb_adc_at_servo_max_deg;

    if (adcMax == adcMin)
    {
        return cfg.servo_min_deg;
    }

    float ratio = (float)(adc - adcMin) / (float)(adcMax - adcMin);

    float angle =
        cfg.servo_min_deg +
        ratio * (cfg.servo_max_deg - cfg.servo_min_deg);

    return clampf(angle, cfg.servo_min_deg, cfg.servo_max_deg);
}

// ---------------------------------------------------------------------------
// Capability detection
// ---------------------------------------------------------------------------

bool hasServoAngleInfo(const ServoConfig& cfg)
{
    return (cfg.servo_max_deg > cfg.servo_min_deg);
}

bool hasAllowedRangeInfo(const ServoConfig& cfg)
{
    return (cfg.allowed_max_deg > cfg.allowed_min_deg);
}

bool hasSpeedInfo(const ServoConfig& cfg)
{
    return (cfg.max_speed_degps > 0.0f);
}

bool hasFeedbackPinInfo(const ServoConfig& cfg)
{
    return (cfg.feedback_adc_pin != -1);
}

bool hasFeedbackCalibrationInfo(const ServoConfig& cfg)
{
    return hasFeedbackPinInfo(cfg) &&
           hasServoAngleInfo(cfg) &&
           (cfg.fb_adc_at_servo_max_deg > cfg.fb_adc_at_servo_min_deg);
}

bool canUseServoController(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) &&
           hasAllowedRangeInfo(cfg) &&
           hasSpeedInfo(cfg);
}

bool canUseAngleDirectMode(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) && !canUseServoController(cfg);
}

void getUsableAngleRange(const ServoConfig& cfg, float& outMinDeg, float& outMaxDeg)
{
    if (hasAllowedRangeInfo(cfg))
    {
        outMinDeg = cfg.allowed_min_deg;
        outMaxDeg = cfg.allowed_max_deg;
    }
    else
    {
        outMinDeg = cfg.servo_min_deg;
        outMaxDeg = cfg.servo_max_deg;
    }
}

// ---------------------------------------------------------------------------
// Button handling
// ---------------------------------------------------------------------------

bool buttonPressedEvent(uint8_t pin,
                        bool& lastReading,
                        bool& stableState,
                        unsigned long& lastDebounceTime)
{
    bool reading = digitalRead(pin);

    if (reading != lastReading)
    {
        lastDebounceTime = millis();
        lastReading = reading;
    }

    if ((millis() - lastDebounceTime) > DEBOUNCE_MS)
    {
        if (reading != stableState)
        {
            stableState = reading;

            if (stableState == HIGH)
            {
                return true;
            }
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// Name display helper
// ---------------------------------------------------------------------------

// Copy at most 15 visible characters from the servo name into a local buffer.
// This avoids line wrapping and keeps the first OLED line stable even if the
// configured name is long.
void makeShortServoName(const char* src, char* dst, size_t dstSize)
{
    if (dstSize == 0) return;

    size_t i = 0;

    while (src[i] != '\0' && i < (dstSize - 1) && i < 15)
    {
        dst[i] = src[i];
        i++;
    }

    dst[i] = '\0';
}

// ============================================================================
// OLED helpers
// ============================================================================

void oledBeginFrame()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
}

void oledPrintSplash()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(4, 16);
    display.print(F("Servo Tester"));

    display.setTextSize(1);
    display.setCursor(40, 42);
    display.print(F("v1.0.0"));

    display.display();
}

void oledPrintServoName(const char* rawName)
{
    char shortName[16];
    makeShortServoName(rawName, shortName, sizeof(shortName));

    display.print(F("Servo "));
    display.println(shortName);
}

void oledPrintLineOnOff(const __FlashStringHelper* label, bool on)
{
    display.print(label);
    display.println(on ? F("On") : F("Off"));
}

void oledPrintLineInt(const __FlashStringHelper* label, int value)
{
    display.print(label);
    display.println(value);
}

void oledPrintPwmAngLine(float minDeg, float maxDeg, float valueDeg)
{
    display.print(F("PWMang ["));
    display.print(minDeg, 0);
    display.print(F("..."));
    display.print(maxDeg, 0);
    display.print(F("] "));
    display.println(valueDeg, 0);
}

void oledEndFrame()
{
    display.display();
}

// ============================================================================
// Servo switching / reinitialization
// ============================================================================

void forcePwmOff()
{
    servoCtrl.disableOutput();
    rawServo.detach();
    pwmEnabled = false;
}

void loadActiveServo()
{
    forcePwmOff();

    activeCfg = &testerServoConfigs[currentServoIndex];

    displaySpeedPct  = 0;
    displayAccelPct  = 0;
    displayPwmAngDeg = 0.0f;
    displayPwmUs     = activeCfg->pwm_min_us;

    if (canUseServoController(*activeCfg))
    {
        servoCtrl.begin(*activeCfg, false);

        if (hasFeedbackPinInfo(*activeCfg))
        {
            servoCtrl.syncToFeedback();
            displayPwmAngDeg = servoCtrl.getCommandDeg();
        }
        else
        {
            servoCtrl.syncToAngle(activeCfg->rest_deg);
            displayPwmAngDeg = activeCfg->rest_deg;
        }

        displayPwmUs = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
    }
    else if (canUseAngleDirectMode(*activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(*activeCfg, minDeg, maxDeg);

        displayPwmAngDeg = minDeg;
        displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
    }
    else
    {
        displayPwmUs = activeCfg->pwm_min_us;
    }
}

void selectNextServo()
{
    currentServoIndex++;

    if (currentServoIndex >= testerServoCount)
    {
        currentServoIndex = 0;
    }

    loadActiveServo();
}

// ============================================================================
// Setup
// ============================================================================

void setup()
{
    pinMode(PWM_TOGGLE_BUTTON_PIN, INPUT);
    pinMode(NEXT_SERVO_BUTTON_PIN, INPUT);

    lastPwmButtonReading  = digitalRead(PWM_TOGGLE_BUTTON_PIN);
    stablePwmButtonState  = lastPwmButtonReading;
    lastPwmDebounceTime   = millis();

    lastNextButtonReading = digitalRead(NEXT_SERVO_BUTTON_PIN);
    stableNextButtonState = lastNextButtonReading;
    lastNextDebounceTime  = millis();

    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();

    oledPrintSplash();
    delay(2000);

    currentServoIndex = 0;
    loadActiveServo();
}

// ============================================================================
// Main loop
// ============================================================================

void loop()
{
    // ------------------------------------------------------------------------
    // Handle NEXT SERVO button
    // ------------------------------------------------------------------------
    if (buttonPressedEvent(NEXT_SERVO_BUTTON_PIN,
                           lastNextButtonReading,
                           stableNextButtonState,
                           lastNextDebounceTime))
    {
        selectNextServo();
    }

    // ------------------------------------------------------------------------
    // Handle PWM ON/OFF button
    // ------------------------------------------------------------------------
    if (buttonPressedEvent(PWM_TOGGLE_BUTTON_PIN,
                           lastPwmButtonReading,
                           stablePwmButtonState,
                           lastPwmDebounceTime))
    {
        pwmEnabled = !pwmEnabled;

        if (canUseServoController(*activeCfg))
        {
            if (pwmEnabled)
            {
                servoCtrl.enableOutput();

                if (hasFeedbackPinInfo(*activeCfg))
                {
                    servoCtrl.syncToFeedback();
                }
            }
            else
            {
                servoCtrl.disableOutput();
            }
        }
        else
        {
            if (pwmEnabled)
            {
                rawServo.attach(
                    activeCfg->pwm_pin,
                    activeCfg->pwm_min_us,
                    activeCfg->pwm_max_us
                );
            }
            else
            {
                rawServo.detach();
            }
        }
    }

    // ------------------------------------------------------------------------
    // Read user controls
    // ------------------------------------------------------------------------
    float adcTarget = readAveragedADC(POT_TARGET_PIN, POT_SAMPLES);
    float adcSpeed  = readAveragedADC(POT_SPEED_PIN,  POT_SAMPLES);
    float adcAccel  = readAveragedADC(POT_ACCEL_PIN,  POT_SAMPLES);

    float   targetDeg = targetDegFromAdcRange(adcTarget, 0.0f, 180.0f);
    uint8_t speedPct  = percentFromAdc(adcSpeed);
    uint8_t accelPct  = percentFromAdc(adcAccel);

    // ------------------------------------------------------------------------
    // Full ServoController mode
    // ------------------------------------------------------------------------
    if (canUseServoController(*activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(*activeCfg, minDeg, maxDeg);

        targetDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);

        displaySpeedPct = speedPct;
        displayAccelPct = accelPct;

        servoCtrl.setTarget(targetDeg, speedPct, accelPct);

        if (pwmEnabled)
        {
            servoCtrl.update();

            displayPwmAngDeg = servoCtrl.getCommandDeg();
            displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
        }
        else
        {
            displayPwmAngDeg = targetDeg;
            displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
        }

        bool showAdc    = hasFeedbackPinInfo(*activeCfg);
        bool showAdcAng = hasFeedbackCalibrationInfo(*activeCfg);

        int fbAdc = 0;
        int fbAng = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg->feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showAdcAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(*activeCfg, fbAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg->name);
        oledPrintLineOnOff(F("PWM "), servoCtrl.isOutputEnabled());
        oledPrintLineInt(F("PWMus "), displayPwmUs);
        oledPrintPwmAngLine(minDeg, maxDeg, displayPwmAngDeg);
        oledPrintLineInt(F("V% "), displaySpeedPct);
        oledPrintLineInt(F("A% "), displayAccelPct);

        if (showAdc)
        {
            oledPrintLineInt(F("ADC "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        if (showAdcAng)
        {
            oledPrintLineInt(F("ADC_ang "), fbAng);
        }

        oledEndFrame();
    }

    // ------------------------------------------------------------------------
    // Angle-direct mode
    // ------------------------------------------------------------------------
    else if (canUseAngleDirectMode(*activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(*activeCfg, minDeg, maxDeg);

        displayPwmAngDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);
        displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(displayPwmUs);
        }

        bool showAdc    = hasFeedbackPinInfo(*activeCfg);
        bool showAdcAng = hasFeedbackCalibrationInfo(*activeCfg);

        int fbAdc = 0;
        int fbAng = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg->feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showAdcAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(*activeCfg, fbAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg->name);
        oledPrintLineOnOff(F("PWM "), pwmEnabled);
        oledPrintLineInt(F("PWMus "), displayPwmUs);
        oledPrintPwmAngLine(minDeg, maxDeg, displayPwmAngDeg);

        if (showAdc)
        {
            oledPrintLineInt(F("ADC "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        if (showAdcAng)
        {
            oledPrintLineInt(F("ADC_ang "), fbAng);
        }
        else
        {
            display.println(F(""));
        }

        display.println(F(""));
        oledEndFrame();
    }

    // ------------------------------------------------------------------------
    // Raw PWM-only mode
    // ------------------------------------------------------------------------
    else
    {
        int pwmUs = pwmUsFromAdc(
            adcTarget,
            activeCfg->pwm_min_us,
            activeCfg->pwm_max_us
        );

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(pwmUs);
        }

        bool showAdc = hasFeedbackPinInfo(*activeCfg);
        int fbAdc = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg->feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg->name);
        oledPrintLineOnOff(F("PWM "), pwmEnabled);
        oledPrintLineInt(F("PWMus "), pwmUs);

        if (showAdc)
        {
            oledPrintLineInt(F("ADC "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        display.println(F(""));
        display.println(F(""));
        display.println(F(""));
        display.println(F(""));

        oledEndFrame();
    }

    delay(20);
}
