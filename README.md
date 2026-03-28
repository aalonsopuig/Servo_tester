# Arduino Servo Tester

**Author:** Alejandro Alonso Puig + GPT  
**License:** Apache 2.0  
**Repository:** https://github.com/aalonsopuig  
**Status:** Validated v1.0.0 (March 2026)

---

## Overview

Servo tester and characterization tool for Arduino Nano.

This tool allows testing, calibrating, and validating different servo configurations using a single hardware setup.  
It is especially useful in robotic systems where multiple servos with different characteristics or mechanical reductions are used.

The tester supports both:

- **Unknown servos** (PWM exploration mode)
- **Characterized servos** (angle control with limits, speed, and acceleration)

This application is based on the **Enhanced Servo Library**:  
https://github.com/aalonsopuig/Enhanced_Servo_Library

---

## Main Features

- PWM-based testing for **unknown servos**
- Angle-based control for **calibrated servos**
- Support for **multiple servo profiles**
- Runtime selection of servo configuration via pushbutton
- Safe **PWM enable/disable control**
- Optional **analog feedback reading**
- OLED display with real-time system parameters
- Efficient use of memory via **PROGMEM**
- Compatible with **Arduino Nano / ATmega328**

---

## Typical Use Cases

This tool is designed for:

- discovering safe PWM limits of a servo
- calibrating servo angular range
- validating mechanical limits of joints
- testing servos with gear reductions
- evaluating analog feedback signals
- tuning speed and acceleration profiles
- validating configurations before robot integration

---

## Hardware

Tested with:

- Arduino Nano
- SSD1306 OLED display (128x64, I2C)
- Standard hobby servos

<br>
<p align="center">
  <img src="servo_tester_arduino/servo_tester_arduino.jpg" width="600">
</p>

---

## Connections

| Use | Pin | Connection |
|------|------|------|
| Target angle | A2 | 10K potentiometer (voltage divider) |
| Speed | A1 | 10K potentiometer (voltage divider) |
| Acceleration | A0 | 10K potentiometer (voltage divider) |
| PWM enable/disable | D4 | Push button with 1K pulldown resistor |
| Next servo | D3 | Push button with 1K pulldown resistor |
| PWM output | defined in `servo_config.h` | Connected to servo signal |
| Feedback Angle | defined in `servo_config.h` (typically A3) | Connected via RC filter (10k / 4.7µF) to servo internal potentiometer |
| Feedback Current | defined in `servo_config.h` (typically A6) | Connected to ACS712 Hall-effect current sensor. Recommended filter: https://is.gd/E6WZuH |
| ADC reference | Aref | Must match potentiometer voltage (5V or 3.3V; 3.3V recommended for stability) |
| OLED SDA | A4 | I2C data |
| OLED SCL | A5 | I2C clock |

---

## User Controls

### Target Potentiometer (A2)

Controls servo position.

Depending on configuration:

- PWM pulse width (unknown servo mode)
- Angular position within calibrated limits

---

### Speed Potentiometer (A1)

Controls speed percentage.

Only active when the servo profile includes speed control.

---

### Acceleration Potentiometer (A0)

Controls acceleration percentage.

Only active when motion profiling is enabled.

---

### PWM Button (D4)

Toggles PWM output:

- OFF → servo detached (safe state)
- ON → servo actively driven

The system always starts with **PWM disabled** for safety.

---

### Next Servo Button (D3)

Cycles through servo configurations defined in:

```cpp
servo_config.h
```

When switching servo:

- PWM is automatically disabled
- configuration is reloaded
- the tester is reinitialized

---

## Display Information

The OLED display shows the most relevant parameters depending on the available configuration data.

Typical information includes:

- **Servo**: Name of the currently selected servo profile.

- **PWM**: Indicates whether PWM output to the servo is enabled or disabled.

- **PWMus**: Pulse width in microseconds currently sent to the servo.

- **PWMang**: Commanded servo angle in degrees. Displayed within the allowed motion range.

- **Vel%**: Commanded speed percentage relative to the configured maximum servo speed.

- **Accel%**: Commanded acceleration percentage used by the motion controller.

- **FBadc**: Raw analog value read from the servo feedback pin.

- **FBang**: Estimated servo angle derived from the feedback ADC calibration.

- **Curr**: estimated current in milliamperes, derived from the configured current sensor calibration
  
Fields appear only when the corresponding data exists.


---

## Servo Configuration

Servo configurations are defined in: `servo_config.h`

Each entry describes one servo profile, including:

- **name**: Human-readable identifier of the servo profile.  
- **pwm_pin**: Arduino pin used to generate the PWM signal.  
- **servo_min_deg**: Minimum calibrated physical angle of the servo.  
- **servo_max_deg**: Maximum calibrated physical angle of the servo.  
- **allowed_min_deg**: Minimum application-level allowed angle.  
- **allowed_max_deg**: Maximum application-level allowed angle.  
- **rest_deg**: Neutral or startup position used during initialization.  
- **pwm_min_us**: Minimum PWM pulse width in microseconds.  
- **pwm_max_us**: Maximum PWM pulse width in microseconds.  
- **max_speed_degps**: Maximum physical speed of the servo in degrees per second.  
- **default_speed_pct**: Default speed percentage.  
- **default_accel_pct**: Default acceleration percentage.  
- **feedback_adc_pin**: Analog input used to read optional servo position feedback.  
- **fb_adc_at_servo_min_deg**: Feedback ADC value measured at `servo_min_deg`.  
- **fb_adc_at_servo_max_deg**: Feedback ADC value measured at `servo_max_deg`.  
- **current_adc_pin**: Analog input used to read the optional current sensor.  
- **current_adc_offset**: ADC offset value corresponding to 0 mA.  
- **current_mA_per_count**: Calibration factor used to convert ADC counts into milliamperes.  
- **inverted**: Indicates whether servo direction must be inverted.  
- **fault_detection_enabled**: Enables current-based fault detection inside the servo controller.  



Example:

```cpp
{
    "SHOULDER",   // name
    9,            // pwm_pin
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
    3,            // feedback_adc_pin
    101,          // fb_adc_at_servo_min_deg
    383,          // fb_adc_at_servo_max_deg
    A5,           // current_adc_pin
    512,          // current_adc_offset
    12.5f,        // current_mA_per_count
    false,        // inverted
    true          // fault_detection_enabled
}
```
---

## Memory Optimization

Servo configurations are stored in **PROGMEM** (Flash memory) instead of SRAM.

This allows defining multiple servo profiles without exhausting the limited RAM available on microcontrollers such as the Arduino Nano (ATmega328).

At runtime, only the active configuration is copied from Flash into SRAM, minimizing memory usage while keeping flexibility.

---

## Servo calibration procedure

This section describes a simple procedure to obtain the parameters required in `servo_config.h` when using the **ServoController library**.

Servo configuration parameters come from three different sources:

- **Servo model** (electrical and internal characteristics)
- **Mechanical joint** (limits imposed by the robot articulation)
- **Application settings** (desired behavior)


During calibration the **speed and acceleration potentiometers should be set to 100%**.

<br>

### 1. Servo electrical limits

Determine the PWM range corresponding to the full travel of the servo.

This depends only on the **servo model** and is the same for all joints using that servo.

Parameters obtained:

- **servo_min_deg**: Minimum physical angle of the servo.

- **servo_max_deg**: Maximum physical angle of the servo.
- **pwm_min_us**: Pulse width corresponding to the minimum servo position.

- **pwm_max_us**: Pulse width corresponding to the maximum servo position.

In this library the servo range is normally assumed to be:

```cpp
servo_min_deg = 0  
servo_max_deg = 180  
```

Example measurements:

Hitec HS-805BB  

```cpp
servo_min_deg = 0  
servo_max_deg = 180  
pwm_min_us ≈ 700  
pwm_max_us ≈ 2400  
```

<br>

### 2. Mechanical joint limits

Once the servo is installed in the robot articulation, determine the safe motion limits of the joint.

Using the calibration program, move the servo slowly and observe the articulation.

Record the safe operating range:

- **allowed_min_deg**: Minimum allowed operating angle for this application.

- **allowed_max_deg**: Maximum allowed operating angle for this application.

These limits depend on the **mechanical design of the joint**, not on the servo itself.

<br>

### 3. Rest position

Define the position used when the system initializes.

This should correspond to a **mechanically safe neutral pose**.

Parameter:

- **rest_deg**: Rest or neutral position used for initialization.

<br>

### 4. Servo speed

If the servo is used directly, the maximum speed can be taken from the **manufacturer datasheet**.

Example:

Futaba S3003  

0.23 s / 60° → 261 deg/s  

`max_speed_degps` represents the maximum angular speed of the **controlled joint**, not necessarily the raw servo shaft speed. If the servo drives a joint through **gears or mechanical transformations**, the real joint speed must be measured.

Move the joint between its limits and measure the travel time.

```cpp
max_speed_degps = angle_travel / time  
```

Example:

180° in 10.27 s  
max_speed_degps ≈ 17.5 deg/s  

<br>

### 5. Angle feedback calibration (optional)

If the servo feedback potentiometer is accessible, its ADC values can be used to estimate the real angle.

Using the calibration program:

1. Move the servo to its minimum angle  
2. Record the ADC value  
3. Move the servo to its maximum angle  
4. Record the ADC value  

Parameters obtained:

- **fb_adc_at_servo_min_deg**: ADC value measured when the servo is at its minimum angle.

- **fb_adc_at_servo_max_deg**: ADC value measured when the servo is at its maximum angle.

Example (HS-805BB modification):

```cpp
fb_adc_at_servo_min_deg ≈ 101  
fb_adc_at_servo_max_deg ≈ 383  
```

These values depend on the **servo electronics**, not the articulation.

<br>

### 6. Current Feedback Calibration (Optional)

This step allows converting raw ADC readings from the current sensor into meaningful current values (mA).

The calibration is performed using the standalone **Current Sensing Calibration Tool**. Use the calibration tool described in the repository: https://github.com/aalonsopuig/Current_sensing_calibration

#### Procedure

1. Connect the current sensor output to the configured `current_adc_pin`.
2. Ensure the ADC reference (AREF) is stable and known (e.g. 3.3V or 5V).
3. Upload and run the calibration tool.
4. With **no load (0 A)**:
   - Record the ADC value → this is the **offset** (`current_adc_offset`)
5. Apply one or more **known current loads**:
   - Record corresponding ADC values
6. Compute the calibration factor:

```text
k = (current_mA) / (adc - offset)
```
Where:

- `adc` → measured ADC value under load
- `offset` → ADC value at 0 A
- `current_mA` → known current in milliamperes
Configuration

Once calibrated, update the servo profile:

```Cpp
current_adc_pin         = <your pin>
current_adc_offset      = <measured offset>
current_mA_per_count    = <k>
```

### 7. Final configuration

Once these values are known, the corresponding entry can be completed in `servo_config.h`.

Each joint should define:

- the servo model characteristics
- the articulation limits
- the desired motion parameters

This separation allows the same servo model to be used in different joints with different mechanical constraints.

### Example servo model parameters

The following table summarizes typical parameters for several hobby servos
used with this library. Speed values correspond to **6 V supply** when available.

| Servo model        | servo_min_deg | servo_max_deg | pwm_min_us | pwm_max_us | max_speed_degps |
|--------------------|--------------|--------------|-----------|-----------|----------------|
| Hitec HS-805BB     | 0            | 180          | 677       | 2350      | 428.6          |
| TowerPro SG-5010   | 0            | 180          | 500       | 1806      | 375.0          |
| Miuzei MF90        | 0            | 180          | 561       | 2500      | 750.0          |
| Futaba S3003       | 0            | 180          | 578       | 2300      | 315.8          |
| DIYMore DM996      | 0            | 180          | 559       | 2472      | 400.0          |

Note: PWM limits are approximate values measured experimentally and may vary
between units. Always verify safe limits before operating the servo in a
mechanical assembly.