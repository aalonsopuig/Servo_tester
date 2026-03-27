/*
===============================================================================
Name:         servo_tester_arduino
Version:      1.0.0
Author:       Alejandro Alonso Puig + GPT
GitHub:       https://github.com/aalonsopuig
Date:         2026-03-27
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
- Read optional analog current sensor if available
- Show the main operating values on an SSD1306 OLED display
- Automatically cut PWM output if a current-safety-enabled servo enters FAULT

The program is intended for:

- finding safe PWM limits of an unknown servo
- checking calibrated servo configurations
- verifying angular limits
- observing feedback ADC values
- observing current consumption
- testing speed and acceleration settings on characterized servos

Hardware:

- Arduino Nano
- Acceleration potentiometer -> A0
- Speed potentiometer        -> A1
- Target potentiometer       -> A2
- Feedback input             -> defined in servo_config.h
- Current sense input        -> defined in servo_config.h
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
- This sketch uses EXTERNAL ADC reference. Connect AREF to the chosen Vref
  (typically 5 V or 3.3 V according to your hardware design)

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
// User-adjustable safety behaviour
// ============================================================================
//
// The library now performs fault detection only from current measurement.
// This sketch adds the application-level reaction once the library reports
// that fault.
//
// FAULT_CONFIRM_CYCLES:
//   Number of consecutive loop iterations in which hasFault()==true must be
//   observed before this sketch cuts PWM output.
//
// AUTO_CUT_PWM_ON_FAULT:
//   If true, the sketch automatically disables PWM after a confirmed fault.
//
// In the current configuration the default reaction is intentionally immediate.
// If at some point harmless false positives appeared, this could be increased.
// ============================================================================

#define FAULT_CONFIRM_CYCLES   1
#define AUTO_CUT_PWM_ON_FAULT  true

// ============================================================================
// Pins
// ============================================================================
//
// Three potentiometers control:
// - acceleration percentage
// - speed percentage
// - target position / target PWM
//
// Two buttons control:
// - PWM ON/OFF
// - NEXT SERVO selection
// ============================================================================

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
// ============================================================================

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   64
#define OLED_ADDRESS  0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ============================================================================
// ADC helpers
// ============================================================================
//
// Arduino Nano ADC range:
// 0 .. 1023
//
// POT_SAMPLES:
//   Small averaging factor used to reduce visible jitter from the pots and
//   from analog noise.
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

// Full-featured controller object used when the selected servo profile has
// enough information to operate in angle/profile mode.
ServoController servoCtrl;

// Raw Servo object used for simpler modes where only direct PWM control is
// possible or desired.
Servo rawServo;

// Number of servo profiles stored in Flash.
//
// Because testerServoConfigs[] is a real array visible here, sizeof()
// correctly gives the number of entries without needing a separate constant.
const uint8_t testerServoCount =
    sizeof(testerServoConfigs) / sizeof(testerServoConfigs[0]);

// Index of the currently selected servo profile.
uint8_t currentServoIndex = 0;

// RAM copy of the currently active configuration.
//
// The full configuration table remains in PROGMEM, and only one entry at a
// time is copied into SRAM. This is important on small AVR boards.
ServoConfig activeCfg;

// Global logical PWM state controlled by the ON/OFF button.
bool pwmEnabled = false;

// Sketch-level fault latch.
//
// The library already has its own internal fault latch. This additional flag
// lets the tester remember that PWM was disabled as a UI/application reaction
// to that fault, so it can be shown clearly on the OLED.
bool testerFaultLatched = false;

// Small configurable confirmation counter before this sketch reacts to fault.
uint8_t faultConfirmCounter = 0;

// Debounce state for PWM ON/OFF button.
bool lastPwmButtonReading = LOW;
bool stablePwmButtonState = LOW;
unsigned long lastPwmDebounceTime = 0;

// Debounce state for NEXT SERVO button.
bool lastNextButtonReading = LOW;
bool stableNextButtonState = LOW;
unsigned long lastNextDebounceTime = 0;

// Cached values shown on the OLED.
//
// These values are continuously updated even when PWM is OFF, so the user can
// pre-position the controls and see the expected command before enabling PWM.
uint8_t displaySpeedPct  = 0;
uint8_t displayAccelPct  = 0;
float   displayPwmAngDeg = 0.0f;
int     displayPwmUs     = 0;

// ============================================================================
// Utility helpers
// ============================================================================

// Small float clamp utility used throughout the sketch.
static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

// Read an analog input several times and return the arithmetic mean.
//
// This is a simple and effective way to reduce jitter both from pots and from
// noisy analog sources shown in the tester.
float readAveragedADC(uint8_t pin, int samples)
{
    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / samples;
}

// Convert an ADC value into a percentage in [1..100].
//
// Used for speed and acceleration potentiometers.
//
// Note that 0 is deliberately not allowed, because the library expects motion
// percentages in a meaningful positive range.
uint8_t percentFromAdc(float adc)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

    int pct = (int)(1.0f + t * 99.0f + 0.5f);

    if (pct < 1)   pct = 1;
    if (pct > 100) pct = 100;

    return (uint8_t)pct;
}

// In direct PWM mode, map the target potentiometer into calibrated PWM
// microseconds between pwmMin and pwmMax.
int pwmUsFromAdc(float adc, int pwmMin, int pwmMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float us = pwmMin + t * (pwmMax - pwmMin);

    if (us < pwmMin) us = pwmMin;
    if (us > pwmMax) us = pwmMax;

    return (int)(us + 0.5f);
}

// Map the target potentiometer into an angular interval [angleMin..angleMax].
float targetDegFromAdcRange(float adc, float angleMin, float angleMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float deg = angleMin + t * (angleMax - angleMin);

    if (deg < angleMin) deg = angleMin;
    if (deg > angleMax) deg = angleMax;

    return deg;
}

// Convert a logical angle into PWM microseconds according to the active servo
// calibration.
//
// This function is used by the tester mainly for display and for direct-angle
// mode. It reproduces the same angle->PWM idea used by the library:
//
// - clamp logical angle
// - apply inversion if configured
// - normalize inside servo span
// - map to PWM pulse width
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

// Convert raw feedback ADC into a logical angle using the static feedback
// calibration present in the servo profile.
//
// This is used only for display/observation in the tester. Feedback is no
// longer used here for safety decisions.
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

// Convert raw current ADC into mA using the calibration stored in the servo
// profile.
//
// This mirrors the library's calibration model, but is used here when PWM is
// OFF or when the current is being read directly by the tester.
float currentMilliAmpsFromAdc(const ServoConfig& cfg, int adc)
{
    float current_mA =
        (adc - (float)cfg.current_adc_offset) *
        cfg.current_mA_per_count;

    if (current_mA < 0.0f)
        current_mA = 0.0f;

    return current_mA;
}

// ---------------------------------------------------------------------------
// Capability detection
// ---------------------------------------------------------------------------
//
// The tester adapts automatically to the information available in the selected
// servo profile. This allows one single sketch to test:
//
// - unknown servos only by PWM
// - known-angle servos directly by angle
// - fully characterized servos with ServoController
// ---------------------------------------------------------------------------

// True when the servo profile defines a valid calibrated angular span.
bool hasServoAngleInfo(const ServoConfig& cfg)
{
    return (cfg.servo_max_deg > cfg.servo_min_deg);
}

// True when the servo profile defines a valid application-level allowed range.
bool hasAllowedRangeInfo(const ServoConfig& cfg)
{
    return (cfg.allowed_max_deg > cfg.allowed_min_deg);
}

// True when the servo profile defines a non-zero physical max speed.
bool hasSpeedInfo(const ServoConfig& cfg)
{
    return (cfg.max_speed_degps > 0.0f);
}

// True when an analog feedback pin is configured.
bool hasFeedbackPinInfo(const ServoConfig& cfg)
{
    return (cfg.feedback_adc_pin != -1);
}

// True when analog feedback can be meaningfully converted into angle.
bool hasFeedbackCalibrationInfo(const ServoConfig& cfg)
{
    return hasFeedbackPinInfo(cfg) &&
           hasServoAngleInfo(cfg) &&
           (cfg.fb_adc_at_servo_max_deg > cfg.fb_adc_at_servo_min_deg);
}

// True when a current sensor pin and calibration factor are configured.
bool hasCurrentSenseInfo(const ServoConfig& cfg)
{
    return (cfg.current_adc_pin != -1) &&
           (cfg.current_mA_per_count > 0.0f);
}

// True when the selected profile can actually use current-based safety.
//
// In the current project stage, "safe" means "current safety available",
// not angular safety.
bool hasSafetyFeatures(const ServoConfig& cfg)
{
    return cfg.fault_detection_enabled &&
           hasCurrentSenseInfo(cfg);
}

// Full ServoController mode requires:
// - angular calibration
// - allowed range
// - speed data
bool canUseServoController(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) &&
           hasAllowedRangeInfo(cfg) &&
           hasSpeedInfo(cfg);
}

// Angle-direct mode is the intermediate case:
// - angle info exists
// - but not enough data exists for full ServoController mode
bool canUseAngleDirectMode(const ServoConfig& cfg)
{
    return hasServoAngleInfo(cfg) && !canUseServoController(cfg);
}

// Return the angle interval that should be presented to the user.
//
// If an application-level allowed range exists, prefer it.
// Otherwise fall back to the full calibrated servo range.
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

// Debounced "pressed event" detector.
//
// Buttons are assumed to use EXTERNAL pull-down resistors:
//
// - released = LOW
// - pressed  = HIGH
//
// The function returns true only once per valid debounced press.
bool buttonPressedEvent(uint8_t pin,
                        bool& lastReading,
                        bool& stableState,
                        unsigned long& lastDebounceTime)
{
    bool reading = digitalRead(pin);

    // Any raw edge restarts debounce timing.
    if (reading != lastReading)
    {
        lastDebounceTime = millis();
        lastReading = reading;
    }

    // Once the signal has remained unchanged long enough, accept it.
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

// Copy at most 19 visible characters from a servo name into a destination
// buffer. This prevents OLED layout corruption from long names.
void makeShortServoName(const char* src, char* dst, size_t dstSize)
{
    if (dstSize == 0) return;

    size_t i = 0;

    while (src[i] != '\0' && i < (dstSize - 1) && i < 19)
    {
        dst[i] = src[i];
        i++;
    }

    dst[i] = '\0';
}

// ============================================================================
// OLED helpers
// ============================================================================

// Start a new OLED frame with the standard text settings used by the tester.
void oledBeginFrame()
{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
}

// Small startup splash screen.
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

// Print current servo name, clipped to fit nicely.
void oledPrintServoName(const char* rawName)
{
    char shortName[20];
    makeShortServoName(rawName, shortName, sizeof(shortName));

    display.print(F("S "));
    display.println(shortName);
}

// Print PWM state line.
//
// Cases:
// - "PWM Fault"     -> sketch already reacted to a detected fault
// - "PWM On (safe)" -> output enabled and current safety available
// - "PWM On"        -> output enabled but without safety
// - "PWM Off"       -> output disabled
void oledPrintPwmStateLine(bool on, bool fault, bool safeEnabled)
{
    display.print(F("PWM "));

    if (fault)
    {
        display.println(F("Fault"));
    }
    else if (on)
    {
        if (safeEnabled)
            display.println(F("On (safe)"));
        else
            display.println(F("On"));
    }
    else
    {
        display.println(F("Off"));
    }
}

// Generic one-line integer printer.
void oledPrintLineInt(const __FlashStringHelper* label, int value)
{
    display.print(label);
    display.println(value);
}

// Print current PWM angle line in the format:
// PWMang [min-max] value
//
// Values are shown without decimals to keep the display compact.
void oledPrintPwmAngLine(float minDeg, float maxDeg, float valueDeg)
{
    display.print(F("PWMang ["));
    display.print(minDeg, 0);
    display.print(F("-"));
    display.print(maxDeg, 0);
    display.print(F("] "));
    display.println(valueDeg, 0);
}

// Print one shared line containing:
// - optional feedback angle
// - optional current in mA
//
// This keeps the OLED dense but readable.
void oledPrintFbAngCurrLine(bool showFbAng, int fbAng, bool showCurr, int currmA)
{
    if (showFbAng)
    {
        display.print(F("FBang "));
        display.print(fbAng);
    }

    if (showCurr)
    {
        if (showFbAng)
            display.print(F("  "));
        display.print(F("Curr "));
        display.print(currmA);
    }

    display.println();
}

// Flush the prepared frame to the OLED.
void oledEndFrame()
{
    display.display();
}

// ============================================================================
// Fault reaction helpers
// ============================================================================

// Clear the sketch-level fault state.
//
// This does NOT automatically clear the library's internal fault latch.
// The caller must explicitly call servoCtrl.resetFault() when appropriate.
void clearTesterFaultState()
{
    testerFaultLatched = false;
    faultConfirmCounter = 0;
}

// Apply the sketch-level reaction to a confirmed current fault.
//
// Current reaction policy:
// - remember that the tester disabled PWM due to fault
// - set logical PWM state OFF
// - detach PWM output from ServoController
void applyFaultReaction()
{
    testerFaultLatched = true;
    pwmEnabled = false;
    servoCtrl.disableOutput();
}

// ============================================================================
// Servo switching / reinitialization
// ============================================================================

// Force every kind of PWM output OFF before changing mode/profile.
void forcePwmOff()
{
    servoCtrl.disableOutput();
    rawServo.detach();
    pwmEnabled = false;
}

// Copy one configuration entry from PROGMEM into activeCfg in SRAM.
void readServoConfigFromProgmem(uint8_t index)
{
    memcpy_P(&activeCfg, &testerServoConfigs[index], sizeof(ServoConfig));
}

// Load the currently selected servo profile.
//
// This function:
//
// - forces all PWM outputs OFF
// - clears sketch-level fault state
// - copies the selected profile from Flash to SRAM
// - initializes the display cache
// - initializes ServoController if full mode is available
//
// New profiles always start with PWM logically OFF.
void loadActiveServo()
{
    forcePwmOff();
    clearTesterFaultState();

    readServoConfigFromProgmem(currentServoIndex);

    displaySpeedPct  = 0;
    displayAccelPct  = 0;
    displayPwmAngDeg = 0.0f;
    displayPwmUs     = activeCfg.pwm_min_us;

    if (canUseServoController(activeCfg))
    {
        // Full-featured mode: use the library.
        servoCtrl.begin(activeCfg, false);

        // If feedback exists, synchronize to real startup position.
        // Otherwise initialize from the configured rest angle.
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
        // Intermediate mode: angle is known, but full controller data is not.
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        displayPwmAngDeg = minDeg;
        displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
    }
    else
    {
        // Raw PWM-only mode.
        displayPwmUs = activeCfg.pwm_min_us;
    }
}

// Select next servo profile cyclically.
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
    // Configure pushbuttons as plain inputs because the hardware uses external
    // pull-down resistors.
    pinMode(PWM_TOGGLE_BUTTON_PIN, INPUT);
    pinMode(NEXT_SERVO_BUTTON_PIN, INPUT);

    // Initialize debounce state from actual current pin states.
    lastPwmButtonReading  = digitalRead(PWM_TOGGLE_BUTTON_PIN);
    stablePwmButtonState  = lastPwmButtonReading;
    lastPwmDebounceTime   = millis();

    lastNextButtonReading = digitalRead(NEXT_SERVO_BUTTON_PIN);
    stableNextButtonState = lastNextButtonReading;
    lastNextDebounceTime  = millis();

    // Use external analog reference. The hardware must provide AREF explicitly.
    analogReference(EXTERNAL);

    // Initialize OLED.
    display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
    display.clearDisplay();
    display.display();

    oledPrintSplash();
    delay(2000);

    // Load the first servo profile.
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
    //
    // Changing servo profile is an explicit hard mode switch:
    // - current output is forced OFF
    // - current sketch-level fault is cleared
    // - a new profile is loaded from Flash
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
    //
    // Important behaviour:
    //
    // If the selected mode uses ServoController and the user turns PWM ON,
    // this is interpreted as an explicit retry:
    //
    // - clear sketch-level fault state
    // - clear library fault state
    // - re-enable PWM output
    // - re-sync to feedback if feedback exists
    //
    // If the selected mode is a simpler raw mode, the raw Servo object is
    // attached/detached directly.
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
                clearTesterFaultState();
                servoCtrl.resetFault();

                servoCtrl.enableOutput();

                if (hasFeedbackPinInfo(activeCfg))
                {
                    servoCtrl.syncToFeedback();
                }
            }
            else
            {
                servoCtrl.disableOutput();
                clearTesterFaultState();
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

            clearTesterFaultState();
        }
    }

    // ------------------------------------------------------------------------
    // Read user controls
    // ------------------------------------------------------------------------
    //
    // Controls are always read, even if PWM is OFF, so the display can show
    // what would be commanded if the user enabled the output right now.
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
    //
    // This is the richest operating mode:
    // - target angle comes from the target pot
    // - speed/accel come from the other pots
    // - the internal motion profile runs inside ServoController
    // - current-based fault detection is active if configured
    // - startup sync to feedback is possible
    // ------------------------------------------------------------------------
    if (canUseServoController(activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        // In full-controller mode, target must be mapped to the usable
        // angular interval, not simply to 0..180.
        targetDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);

        displaySpeedPct = speedPct;
        displayAccelPct = accelPct;

        // Always update target in the controller, even with PWM OFF,
        // so the future command is ready and visible on screen.
        servoCtrl.setTarget(targetDeg, speedPct, accelPct);

        if (pwmEnabled)
        {
            // Run controller update only while output is logically enabled.
            servoCtrl.update();

            // Library fault -> sketch-level reaction.
            //
            // The library latches the fault internally. This sketch decides
            // whether and when to cut PWM.
            if (servoCtrl.hasFault())
            {
                if (faultConfirmCounter < 255)
                {
                    faultConfirmCounter++;
                }

                if (AUTO_CUT_PWM_ON_FAULT &&
                    faultConfirmCounter >= FAULT_CONFIRM_CYCLES)
                {
                    applyFaultReaction();
                }
            }
            else
            {
                // No fault currently observed -> clear confirmation counter.
                faultConfirmCounter = 0;
            }

            // With PWM ON, show the real internal command generated by the
            // profile, not merely the user's target knob position.
            displayPwmAngDeg = servoCtrl.getCommandDeg();
            displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);
        }
        else
        {
            // With PWM OFF, simply show the user-selected target angle.
            displayPwmAngDeg = targetDeg;
            displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);

            // If output is off, no sketch-level fault confirmation is ongoing.
            faultConfirmCounter = 0;
        }

        // Decide what extra telemetry can be shown.
        bool showAdc   = hasFeedbackPinInfo(activeCfg);
        bool showFbAng = hasFeedbackCalibrationInfo(activeCfg);
        bool showCurr  = hasCurrentSenseInfo(activeCfg);

        int fbAdc  = 0;
        int fbAng  = 0;
        int currmA = 0;

        // Read feedback ADC directly for display.
        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        // Convert feedback ADC to angle if calibration exists.
        if (showFbAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(activeCfg, fbAdc) + 0.5f);
        }

        // Current display:
        //
        // - if PWM is ON and ServoController is active, use the current already
        //   measured/calculated by the library
        // - if PWM is OFF, read current directly from ADC for display only
        if (showCurr)
        {
            if (pwmEnabled)
            {
                currmA = (int)(servoCtrl.getCurrentMilliAmps() + 0.5f);
            }
            else
            {
                int currAdc = (int)(readAveragedADC(activeCfg.current_adc_pin, POT_SAMPLES) + 0.5f);
                currmA = (int)(currentMilliAmpsFromAdc(activeCfg, currAdc) + 0.5f);
            }
        }

        // Build OLED frame.
        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintPwmStateLine(servoCtrl.isOutputEnabled(),
                              testerFaultLatched,
                              hasSafetyFeatures(activeCfg));
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

        oledPrintFbAngCurrLine(showFbAng, fbAng, showCurr, currmA);

        oledEndFrame();
    }

    // ------------------------------------------------------------------------
    // Angle-direct mode
    // ------------------------------------------------------------------------
    //
    // This mode is used when an angular calibration exists, but the profile
    // controller does not have enough information to be used.
    //
    // In this case:
    // - target pot directly selects an angle
    // - that angle is converted to PWM immediately
    // - no motion profile is used
    // - no library fault logic is used
    // ------------------------------------------------------------------------
    else if (canUseAngleDirectMode(activeCfg))
    {
        float minDeg, maxDeg;
        getUsableAngleRange(activeCfg, minDeg, maxDeg);

        displayPwmAngDeg = targetDegFromAdcRange(adcTarget, minDeg, maxDeg);
        displayPwmUs     = pwmUsFromAngle(activeCfg, displayPwmAngDeg);

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(displayPwmUs);
        }

        bool showAdc   = hasFeedbackPinInfo(activeCfg);
        bool showFbAng = hasFeedbackCalibrationInfo(activeCfg);
        bool showCurr  = hasCurrentSenseInfo(activeCfg);

        int fbAdc  = 0;
        int fbAng  = 0;
        int currmA = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showFbAng)
        {
            fbAng = (int)(feedbackAngleFromAdc(activeCfg, fbAdc) + 0.5f);
        }

        if (showCurr)
        {
            int currAdc = (int)(readAveragedADC(activeCfg.current_adc_pin, POT_SAMPLES) + 0.5f);
            currmA = (int)(currentMilliAmpsFromAdc(activeCfg, currAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintPwmStateLine(pwmEnabled, false, hasSafetyFeatures(activeCfg));
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

        oledPrintFbAngCurrLine(showFbAng, fbAng, showCurr, currmA);
        display.println(F(""));
        display.println(F(""));

        oledEndFrame();
    }

    // ------------------------------------------------------------------------
    // Raw PWM-only mode
    // ------------------------------------------------------------------------
    //
    // This is the fallback for poorly characterized or unknown servos.
    //
    // In this mode:
    // - target pot directly selects PWM microseconds
    // - no angle model is used
    // - optional feedback ADC can still be shown if wired
    // - optional current can still be shown if wired
    // ------------------------------------------------------------------------
    else
    {
        int pwmUs = pwmUsFromAdc(
            adcTarget,
            activeCfg.pwm_min_us,
            activeCfg.pwm_max_us
        );

        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(pwmUs);
        }

        bool showAdc  = hasFeedbackPinInfo(activeCfg);
        bool showCurr = hasCurrentSenseInfo(activeCfg);

        int fbAdc  = 0;
        int currmA = 0;

        if (showAdc)
        {
            fbAdc = (int)(readAveragedADC(activeCfg.feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        if (showCurr)
        {
            int currAdc = (int)(readAveragedADC(activeCfg.current_adc_pin, POT_SAMPLES) + 0.5f);
            currmA = (int)(currentMilliAmpsFromAdc(activeCfg, currAdc) + 0.5f);
        }

        oledBeginFrame();

        oledPrintServoName(activeCfg.name);
        oledPrintPwmStateLine(pwmEnabled, false, hasSafetyFeatures(activeCfg));
        oledPrintLineInt(F("PWMus "), pwmUs);

        if (showAdc)
        {
            oledPrintLineInt(F("FBadc "), fbAdc);
        }
        else
        {
            display.println(F(""));
        }

        oledPrintFbAngCurrLine(false, 0, showCurr, currmA);
        display.println(F(""));
        display.println(F(""));
        display.println(F(""));

        oledEndFrame();
    }

    // Small loop pacing delay.
    delay(20);
}
