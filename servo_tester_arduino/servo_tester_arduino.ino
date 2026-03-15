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

Single-/multi-servo tester for two different use cases:

1) CHARACTERIZED SERVO
   The servo has known calibration data in servo_config.h:
   - angular range
   - PWM range
   - optional feedback calibration
   - maximum speed

   In this case the sketch uses ServoController and can display:
   - PWM state
   - PWM value
   - PWM reference angle
   - speed %
   - acceleration %
   - optional feedback ADC
   - optional feedback angle

2) UNKNOWN / UNCHARACTERIZED SERVO
   The servo does not yet have a valid angular characterization.

   In this case the sketch behaves as a direct PWM tester:
   - A2 directly controls PWM from pwm_min_us to pwm_max_us
   - no angle is shown
   - no speed or acceleration are shown
   - if feedback_adc_pin != -1, the raw ADC value is shown

Additional feature:

- Multiple servo configurations can be defined in servo_config.h
- A second pushbutton on D3 cycles through the available configurations
- Whenever the active servo changes, PWM is forced OFF first, and the whole
  tester state is reloaded for the new servo

How characterization is detected:

The sketch assumes the servo is characterized only if:

- servo_max_deg > servo_min_deg
- allowed_max_deg > allowed_min_deg
- max_speed_degps > 0

Otherwise it switches automatically to unknown-servo mode.

Hardware:

- Arduino Nano
- Acceleration potentiometer -> A0
- Speed potentiometer        -> A1
- Target potentiometer       -> A2
- Feedback input             -> A3
- PWM ON/OFF pushbutton      -> D4
- NEXT SERVO pushbutton      -> D3
- Servo PWM output           -> defined in servo_config.h
- OLED SSD1306 I2C display   -> A4 (SDA), A5 (SCL)

Important:

This sketch assumes both pushbuttons have an EXTERNAL pull-down resistor:
- released = LOW
- pressed  = HIGH

Expected servo_config.h symbols:

- TESTER_SERVO_COUNT
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
#define POT_FEEDBACK_PIN       A3

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

// Used only when the active servo is characterized and the sketch operates
// through the ServoController library.
ServoController servoCtrl;

// Used only when the active servo is not characterized and we want direct
// manual control in microseconds.
Servo rawServo;

// Index of the currently selected servo configuration.
uint8_t currentServoIndex = 0;

// Convenience pointer to the currently active configuration.
// It is updated whenever the servo selection changes.
const ServoConfig* activeCfg = nullptr;

// PWM starts disabled by default.
bool pwmEnabled = false;

// Automatic mode detection:
// characterized = true  -> use ServoController
// characterized = false -> direct PWM tester
bool characterized = false;

// Debounce state for PWM ON/OFF button (D4)
bool lastPwmButtonReading = LOW;
bool stablePwmButtonState = LOW;
unsigned long lastPwmDebounceTime = 0;

// Debounce state for NEXT SERVO button (D3)
bool lastNextButtonReading = LOW;
bool stableNextButtonState = LOW;
unsigned long lastNextDebounceTime = 0;

// Cached display values.
// These are updated continuously even when PWM output is OFF so the user can
// pre-adjust the target position and immediately see the expected PWM values.
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

// Read an analog pin several times and return the average.
// This reduces visible jitter in potentiometer readings.
float readAveragedADC(uint8_t pin, int samples)
{
    long sum = 0;

    for (int i = 0; i < samples; i++)
    {
        sum += analogRead(pin);
    }

    return (float)sum / samples;
}

// Convert the target potentiometer ADC value into an angle in degrees.
// This is only meaningful when the servo is already characterized.
float targetDegFromAdc(float adc)
{
    float deg = (adc / ADC_SCALE) * 180.0f;
    return clampf(deg, 0.0f, 180.0f);
}

// Convert a potentiometer ADC value into an integer percentage in [1..100].
uint8_t percentFromAdc(float adc)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);

    int pct = (int)(1.0f + t * 99.0f + 0.5f);

    if (pct < 1)   pct = 1;
    if (pct > 100) pct = 100;

    return (uint8_t)pct;
}

// In unknown-servo mode, the position potentiometer controls PWM directly
// between pwm_min_us and pwm_max_us.
int pwmUsFromAdc(float adc, int pwmMin, int pwmMax)
{
    float t = clampf(adc / ADC_SCALE, 0.0f, 1.0f);
    float us = pwmMin + t * (pwmMax - pwmMin);

    if (us < pwmMin) us = pwmMin;
    if (us > pwmMax) us = pwmMax;

    return (int)(us + 0.5f);
}

// Local conversion from logical angle to PWM microseconds.
// This is used for display purposes in characterized mode so that PWMus can
// be shown correctly even when the real PWM output is OFF.
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

// Convert raw feedback ADC into angle using the calibration stored in the
// current active servo configuration. This is only meaningful when feedback
// exists and when the servo has already been characterized.
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

// Determine automatically whether the current servo configuration is complete
// enough to operate in characterized mode.
bool isServoCharacterized(const ServoConfig& cfg)
{
    if (cfg.servo_max_deg <= cfg.servo_min_deg) return false;
    if (cfg.allowed_max_deg <= cfg.allowed_min_deg) return false;
    if (cfg.max_speed_degps <= 0.0f) return false;

    return true;
}

// Generic debounced pushbutton event detector.
//
// The button has EXTERNAL pull-down:
// - released = LOW
// - pressed  = HIGH
//
// This function returns true only once for each valid press event.
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

// ============================================================================
// OLED display
// ============================================================================

void drawCharacterizedDisplay(const char* name,
                              bool pwmOn,
                              int pwmUs,
                              float pwmAngDeg,
                              uint8_t speedPct,
                              uint8_t accelPct,
                              bool hasFeedback,
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

    display.print("PWMang ");
    display.println(pwmAngDeg, 1);

    display.print("V% ");
    display.println(speedPct);

    display.print("A% ");
    display.println(accelPct);

    if (hasFeedback)
    {
        display.print("ADC ");
        display.println(fbAdc);

        display.print("Ang ");
        display.println(fbDeg, 1);
    }
    else
    {
        display.println("");
        display.println("");
    }

    display.display();
}

// Unknown mode display.
// If a feedback pin exists, the raw ADC value is still shown, even though the
// angular meaning is not yet known.
void drawUnknownDisplay(const char* name,
                        bool pwmOn,
                        int pwmUs,
                        bool hasFeedback,
                        int fbAdc)
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

    if (hasFeedback)
    {
        display.print("ADC ");
        display.println(fbAdc);
    }
    else
    {
        display.println("");
    }

    display.println("");
    display.println("");
    display.println("");
    display.println("");

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

// Load the currently selected servo configuration.
//
// This function is called:
// - once at startup
// - each time the user presses the NEXT SERVO button
//
// It always starts the newly selected servo with PWM OFF.
void loadActiveServo()
{
    // Safety first: turn all outputs off before changing configuration.
    forcePwmOff();

    // Update pointer to the currently selected configuration.
    activeCfg = &testerServoConfigs[currentServoIndex];

    // Detect automatically whether this servo is characterized.
    characterized = isServoCharacterized(*activeCfg);

    // Reset display cache.
    displaySpeedPct  = 0;
    displayAccelPct  = 0;
    displayPwmAngDeg = 0.0f;
    displayPwmUs     = activeCfg->pwm_min_us;

    if (characterized)
    {
        // Initialize controller in PWM-OFF mode.
        servoCtrl.begin(*activeCfg, false);

        // Synchronize internal state. If feedback exists, use it. Otherwise
        // initialize the controller at the configured rest position.
        if (activeCfg->feedback_adc_pin != -1)
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
    else
    {
        // Unknown-servo mode: no ServoController motion logic is needed.
        // The user will control PWM directly in microseconds.
        displayPwmUs = activeCfg->pwm_min_us;
    }
}

// Advance currentServoIndex cyclically.
void selectNextServo()
{
    currentServoIndex++;

    if (currentServoIndex >= TESTER_SERVO_COUNT)
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

    // Load the first servo configuration defined in servo_config.h
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
    // On each valid press:
    // - PWM is forced OFF
    // - the next servo configuration becomes active
    // - the whole tester state is rebuilt
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
    // The button toggles PWM generation in either mode.
    if (buttonPressedEvent(PWM_TOGGLE_BUTTON_PIN,
                           lastPwmButtonReading,
                           stablePwmButtonState,
                           lastPwmDebounceTime))
    {
        pwmEnabled = !pwmEnabled;

        if (characterized)
        {
            if (pwmEnabled)
            {
                // Re-enable PWM and resynchronize to current feedback position
                // before starting motion generation, if feedback exists.
                servoCtrl.enableOutput();

                if (activeCfg->feedback_adc_pin != -1)
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

    float   targetDeg = targetDegFromAdc(adcTarget);
    uint8_t speedPct  = percentFromAdc(adcSpeed);
    uint8_t accelPct  = percentFromAdc(adcAccel);

    // ------------------------------------------------------------------------
    // Characterized mode
    // ------------------------------------------------------------------------
    if (characterized)
    {
        // Keep speed and acceleration display values updated continuously,
        // even when PWM output is OFF.
        displaySpeedPct = speedPct;
        displayAccelPct = accelPct;

        // Always update the controller target so the application remembers the
        // user's desired command, even while PWM is OFF.
        servoCtrl.setTarget(targetDeg, speedPct, accelPct);

        // If PWM is ON, the controller advances its trapezoidal profile.
        if (pwmEnabled)
        {
            servoCtrl.update();

            // When PWM is ON, show the real internal PWM reference generated
            // by the controller.
            displayPwmAngDeg = servoCtrl.getCommandDeg();
            displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
        }
        else
        {
            // When PWM is OFF, update the screen directly from the position
            // potentiometer so the user can pre-select a desired angle before
            // enabling PWM output.
            displayPwmAngDeg = targetDeg;
            displayPwmUs     = pwmUsFromAngle(*activeCfg, displayPwmAngDeg);
        }

        bool hasFb = (activeCfg->feedback_adc_pin != -1);

        int fbAdc = 0;
        float fbDeg = 0.0f;

        if (hasFb)
        {
            fbAdc = (int)(readAveragedADC(POT_FEEDBACK_PIN, POT_SAMPLES) + 0.5f);
            fbDeg = feedbackAngleFromAdc(*activeCfg, fbAdc);
        }

        drawCharacterizedDisplay(
            activeCfg->name,
            servoCtrl.isOutputEnabled(),
            displayPwmUs,
            displayPwmAngDeg,
            displaySpeedPct,
            displayAccelPct,
            hasFb,
            fbAdc,
            fbDeg
        );
    }

    // ------------------------------------------------------------------------
    // Unknown-servo mode
    // ------------------------------------------------------------------------
    else
    {
        // Here the target potentiometer directly controls PWM microseconds.
        // This is the correct behaviour when the servo has not yet been fully
        // characterized and angle information is not reliable.
        int pwmUs = pwmUsFromAdc(
            adcTarget,
            activeCfg->pwm_min_us,
            activeCfg->pwm_max_us
        );

        // If PWM is enabled, send the manually selected pulse width.
        if (pwmEnabled)
        {
            rawServo.writeMicroseconds(pwmUs);
        }

        bool hasFb = (activeCfg->feedback_adc_pin != -1);
        int fbAdc = 0;

        if (hasFb)
        {
            fbAdc = (int)(readAveragedADC(activeCfg->feedback_adc_pin, POT_SAMPLES) + 0.5f);
        }

        drawUnknownDisplay(
            activeCfg->name,
            pwmEnabled,
            pwmUs,
            hasFb,
            fbAdc
        );
    }

    delay(20);
}
