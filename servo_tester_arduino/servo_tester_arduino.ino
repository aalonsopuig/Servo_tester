/*
===============================================================================
Name:         servo_tester_arduino
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-15
License:      Apache 2.0
-------------------------------------------------------------------------------
Description:

Arduino-based servo tester for calibration and verification of one or more
servo configurations defined in servo_config.h.

Main functions:

- Select one servo profile from several predefined configurations
- Enable or disable PWM output safely from a pushbutton
- Change active servo configuration cyclically from another pushbutton
- Test unknown servos directly in PWM microseconds
- Test characterized servos in angular mode
- Read optional analog feedback if available
- Show the main operating values on an SSD1306 OLED display

The program is intended for:

- finding safe PWM limits of an unknown servo
- checking calibrated servo configurations
- verifying angular limits
- observing feedback ADC values
- testing speed and acceleration settings on characterized servos

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

- Both pushbuttons must use an external pull-down resistor
- PWM output starts OFF at power-up
- Servo configurations are defined in servo_config.h
- Multiple servo profiles are supported
- Configuration data is stored in PROGMEM to save SRAM
- We use external reference Vref for ADC conversions. Use either 5v o 3v3 depending on your servo.

Expected servo_config.h symbols:

- testerServoConfigs[]

===============================================================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <avr/pgmspace.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "ServoController.h"
#include "servo_config.h"

// ============================================================================
// Pins
// ============================================================================
//
// Three potentiometers:
//
// A0 -> acceleration percentage
// A1 -> speed percentage
// A2 -> target position
//
// Two buttons:
//
// D4 -> PWM output ON/OFF
// D3 -> select next servo configuration
//

#define POT_ACCEL_PIN          A0
#define POT_SPEED_PIN          A1
#define POT_TARGET_PIN         A2

#define PWM_TOGGLE_BUTTON_PIN  4
#define NEXT_SERVO_BUTTON_PIN  3

// ============================================================================
// OLED configuration
// ============================================================================
//
// Standard 128x64 SSD1306 display over I2C.
//

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// ADC helpers
// ============================================================================
//
// Arduino Nano uses 10-bit ADC:
// 0 .. 1023
//

#define ADC_SCALE   1023.0f
#define POT_SAMPLES 4

// ============================================================================
// Button debounce
// ============================================================================

#define DEBOUNCE_MS 40

// ============================================================================
// Global objects / state
// ============================================================================

// ServoController instance used only when enough data exists to operate in
// full angular/profile mode.
ServoController servoCtrl;

// Raw Servo object used in direct PWM modes.
Servo rawServo;

// Number of servo configurations available.
//
// Because testerServoConfigs[] is an array defined in servo_config.h, sizeof()
// still works correctly here and avoids maintaining a separate count manually.
const uint8_t testerServoCount =
    sizeof(testerServoConfigs) / sizeof(testerServoConfigs[0]);

// Index of the currently selected servo configuration in Flash.
uint8_t currentServoIndex = 0;

// Active servo configuration copied from PROGMEM into RAM.
//
// This is the key RAM-saving idea:
// - the full table lives in Flash
// - only the currently active entry lives in SRAM
ServoConfig activeCfg;

// PWM starts disabled by default for safety.
bool pwmEnabled = false;

// Debounce state for PWM ON/OFF button (D4)
bool lastPwmButtonReading = LOW;
bool stablePwmButtonState = LOW;
unsigned long lastPwmDebounceTime = 0;

// Debounce state for NEXT SERVO button (D3)
bool lastNextButtonReading = LOW;
bool stableNextButtonState = LOW;
unsigned long lastNextDebounceTime = 0;

// Cached display values.
//
// These are updated continuously, even while PWM is OFF, so the user can
// pre-adjust the target position and still see what will happen before
// enabling the servo output.
uint8_t displaySpeedPct  = 0;
uint8_t displayAccelPct  = 0;
float   displayPwmAngDeg = 0.0f;
int     displayPwmUs     = 0;

// ============================================================================
// Utility helpers
// ============================================================================

// Simple clamp helper used throughout the sketch.
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Read an analog pin several times and return the average.
//
// This reduces visible jitter from the potentiometers and from analog noise.
float readAveragedADC(uint8_t pin, int samples)
{
    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / samples;
}

// Convert a potentiometer ADC value into an integer percentage in [1..100].
//
// This is used for the speed and acceleration controls.
uint8_t percentFromAdc(float adc)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

    int pct = (int)(1.0f + t * 99.0f + 0.5f);

    if (pct < 1)   pct = 1;
    if (pct > 100) pct = 100;

    return (uint8_t)pct;
}

// In raw-PWM mode, the position potentiometer directly controls PWM pulse width
// between pwm_min_us and pwm_max_us.
int pwmUsFromAdc(float adc, int pwmMin, int pwmMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float us = pwmMin + t * (pwmMax - pwmMin);

    if (us < pwmMin) us = pwmMin;
    if (us > pwmMax) us = pwmMax;

    return (int)(us + 0.5f);
}

// Map target potentiometer to an angle range [angleMin..angleMax].
//
// This is used when at least angular information is known.
float targetDegFromAdcRange(float adc, float angleMin, float angleMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float deg = angleMin + t * (angleMax - angleMin);

    if (deg < angleMin) deg = angleMin;
    if (deg > angleMax) deg = angleMax;

    return deg;
}

// Convert a logical angle into PWM microseconds using the active servo config.
//
// This is used for display purposes and also in angle-direct mode.
// It respects inversion and calibrated PWM limits.
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

// Convert raw feedback ADC into angle using feedback calibration.
//
// This is only meaningful when:
// - a feedback pin exists
// - the servo angular range is known
// - the feedback ADC limits are known
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
//
// The sketch adapts itself dynamically to the information available in the
// current servo configuration.
//

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

// Full ServoController mode requires:
// - servo angular range
// - allowed range
// - speed data
bool canUseServoController(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) &&
           hasAllowedRangeInfo(cfg) &&
           hasSpeedInfo(cfg);
}

// Angle-direct mode means:
// - servo angular range exists
// - but not enough information exists for full ServoController mode
bool canUseAngleDirectMode(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) && !canUseServoController(cfg);
}

// Return the currently usable angular range.
//
// If an allowed range is available, use it.
// Otherwise use the full calibrated servo range.
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

// Generic debounced pushbutton event detector.
//
// Buttons use EXTERNAL pull-down:
// - released = LOW
// - pressed  = HIGH
//
// Returns true only once for each valid press event.
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
// This prevents wrapping and avoids corrupting the OLED layout even if the
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

// Startup splash screen.
// "Servo Tester" does not fit nicely on one large line, so it is split.
void oledPrintSplash()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);

    display.setTextSize(2);
    display.setCursor(34, 10);
    display.print(F("Servo"));

    display.setCursor(30, 30);
    display.print(F("Tester"));

    display.setTextSize(1);
    display.setCursor(40, 54);
    display.print(F("v1.0.0"));

    display.display();
}

// Print servo name without ":" and clipped to 15 chars.
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

// Print angle line in the requested format:
//
// PWMang [min-max] value
//
// Everything is shown without decimals to save space and improve readability.
void oledPrintPwmAngLine(float minDeg, float maxDeg, float valueDeg)
{
    display.print(F("PWMang ["));
    display.print(minDeg, 0);
    display.print(F("-"));
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

// Force every kind of PWM output OFF before changing servo configuration.
void forcePwmOff()
{
    servoCtrl.disableOutput();
    rawServo.detach();
    pwmEnabled = false;
}

// Copy one ServoConfig entry from PROGMEM into the RAM copy activeCfg.
void readServoConfigFromProgmem(uint8_t index)
{
    memcpy_P(&activeCfg, &testerServoConfigs[index], sizeof(ServoConfig));
}

// Load the currently selected servo configuration.
//
// Called:
// - once at startup
// - whenever the user presses NEXT SERVO
//
// Always starts the new servo with PWM OFF.
void loadActiveServo()
{
    forcePwmOff();

    // Read active configuration from Flash into RAM.
    readServoConfigFromProgmem(currentServoIndex);

    displaySpeedPct  = 0;
    displayAccelPct  = 0;
    displayPwmAngDeg = 0.0f;
    displayPwmUs     = activeCfg.pwm_min_us;

    if (canUseServoController(activeCfg))
    {
        servoCtrl.begin(activeCfg, false);

        if (hasFeedbackPinInfo(activeCfg))
        {
            servoCtrl.syncToFeedback();
            displayPwmAngDeg = servoCtrl.getCommandDeg();
        }
        else
        {
            servoCtrl.syncToAngle(activeCfg.rest_deg);
            displayPwmAngDeg = activeCfg.rest_deg;
        }

        displayPwmUs = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
    }
    else if (canUseAngleDirectMode(activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        displayPwmAngDeg = minDeg;
        displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
    }
    else
    {
        displayPwmUs = activeCfg.pwm_min_us;
    }
}

// Advance currentServoIndex cyclically.
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

    analogReference(EXTERNAL); // We use de 3v3 voltage as reference for ADC

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

        if (canUseServoController(activeCfg))
        {
            if (pwmEnabled)
            {
                servoCtrl.enableOutput();

                if (hasFeedbackPinInfo(activeCfg))
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
                    activeCfg.pwm_pin,
                    activeCfg.pwm_min_us,
                    activeCfg.pwm_max_us
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
    if (canUseServoController(activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        // In full controller mode the target potentiometer is interpreted over
        // the usable angular range, not simply over 0..180.
        targetDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);

        displaySpeedPct = speedPct;
        displayAccelPct = accelPct;

        // Always update target, even with PWM OFF, so the user can pre-adjust
        // the future command before enabling the output.
        servoCtrl.setTarget(targetDeg, speedPct, accelPct);

        if (pwmEnabled)
        {
            servoCtrl.update();

            // With PWM ON, show the real internal command generated by the
            // motion profile.
            displayPwmAngDeg = servoCtrl.getCommandDeg();
            displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
        }
        else
        {
            // With PWM OFF, show directly the user-selected target angle.
            displayPwmAngDeg = targetDeg;
            displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
        }

        bool showAdc   = hasFeedbackPinInfo(activeCfg);
        bool showFbAng = hasFeedbackCalibrationInfo(activeCfg);

        int fbAdc = 0;
        int fbAng = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showFbAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(activeCfg, fbAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintLineOnOff(F("PWM "), servoCtrl.isOutputEnabled());
        oledPrintLineInt(F("PWMus "), displayPwmUs);
        oledPrintPwmAngLine(minDeg, maxDeg, displayPwmAngDeg);
        oledPrintLineInt(F("Vel% "), displaySpeedPct);
        oledPrintLineInt(F("Accel% "), displayAccelPct);

        if (showAdc)
        {
            oledPrintLineInt(F("FBadc "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        if (showFbAng)
        {
            oledPrintLineInt(F("FBang "), fbAng);
        }

        oledEndFrame();
    }

    // ------------------------------------------------------------------------
    // Angle-direct mode
    // ------------------------------------------------------------------------
    else if (canUseAngleDirectMode(activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        // In angle-direct mode, the target potentiometer directly selects an
        // angle in the known usable range. No motion profile is used.
        displayPwmAngDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);
        displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(displayPwmUs);
        }

        bool showAdc   = hasFeedbackPinInfo(activeCfg);
        bool showFbAng = hasFeedbackCalibrationInfo(activeCfg);

        int fbAdc = 0;
        int fbAng = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showFbAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(activeCfg, fbAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintLineOnOff(F("PWM "), pwmEnabled);
        oledPrintLineInt(F("PWMus "), displayPwmUs);
        oledPrintPwmAngLine(minDeg, maxDeg, displayPwmAngDeg);

        if (showAdc)
        {
            oledPrintLineInt(F("FBadc "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        if (showFbAng)
        {
            oledPrintLineInt(F("FBang "), fbAng);
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
        // In raw mode the target potentiometer directly controls PWM pulse width.
        int pwmUs = pwmUsFromAdc(
            adcTarget,
            activeCfg.pwm_min_us,
            activeCfg.pwm_max_us
        );

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(pwmUs);
        }

        bool showAdc = hasFeedbackPinInfo(activeCfg);
        int fbAdc = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintLineOnOff(F("PWM "), pwmEnabled);
        oledPrintLineInt(F("PWMus "), pwmUs);

        if (showAdc)
        {
            oledPrintLineInt(F("FBadc "), fbAdc);
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
