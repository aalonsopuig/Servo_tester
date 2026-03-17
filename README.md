# Arduino Servo Tester

Author: Alejandro Alonso Puig + GPT

License: Apache 2.0 

Repository: https://github.com/aalonsopuig

Status: Validation in progress (March 2026)

---
Servo tester and characterization tool for Arduino Nano.

This project allows testing and calibrating multiple servo configurations using a single hardware setup.  
It is especially useful when working with robots that contain many different servos or mechanical reductions.

The tester supports both **unknown servos** (PWM exploration) and **fully characterized servos** (angular control with limits, speed and acceleration).

This application uses the Enhanced servo library https://github.com/aalonsopuig/Enhanced_Servo_Library

---

## Main Features

- Test **unknown servos** by directly controlling PWM pulse width
- Test **characterized servos** using angle control
- Support for **multiple servo profiles**
- Change active servo configuration with a pushbutton
- Enable / disable PWM safely
- Optional **analog feedback reading**
- OLED display showing operating parameters
- Uses **PROGMEM** to store many servo configurations without exhausting RAM
- Works on **Arduino Nano / ATmega328**

---

## Typical Use Cases

This tool is useful for:

- discovering safe PWM limits of a servo
- verifying servo calibration data
- validating mechanical limits of a joint
- testing servos with gear reductions
- reading analog feedback signals
- tuning speed and acceleration parameters
- validating servo configurations before integrating them in a robot

---

## Hardware

Tested with:

- Arduino Nano
- SSD1306 OLED display (128x64 I2C)
- standard hobby servos
<br>
<p align="center">
  <img src="servo_tester_arduino/servo_tester_arduino.jpg" width="600">
</p>

### Connections

| Use | Pin | Connection |
|------|------|------|
| Target angle | A2 | Connected to 10K pot voltage divider |
| Speed | A1 | Connected to 10K pot voltage divider |
| Acceleration | A0 | Connected to 10K pot voltage divider |
| PWM enable/disable | D4 | Connected to push button with 1K pulldown resistor |
| Next servo | D3 | Connected to push button with 1K pulldown resistor |
| PWM output | defined in `servo_config.h` | Connected to servo |
| Feedback ADC | defined in `servo_config.h` but in this circuit is A3| Connected to low-pass RC filter (10k/4.7uF) and from it to internal pot of servo |
| ADC reference | Aref | This voltage reference should be the same voltage connected to the potentiometers. It may be 5v (from +5V pin) or 3.3v (from 3v3 pin) which is better to avoid fluctuations of ADC when power voltage drops for any reason |
| OLED SDA | A4 | Connected to OLED display |
| OLED SCL | A5 | Connected to OLED display |


---

## User Controls

### Target Potentiometer (A2)

Controls the servo position.

Depending on the servo configuration it may represent:

- PWM pulse width
- angular position within allowed limits

<br>

### Speed Potentiometer (A1)

Controls speed percentage.

Used only when the servo configuration includes speed data.

<br>

### Acceleration Potentiometer (A0)

Controls acceleration percentage.

Used only when the servo configuration supports motion profiling.

<br>

### PWM Button (D4)

Toggles PWM output:
- OFF → servo detached
- ON → servo driven


The system always starts with **PWM disabled** for safety.

<br>

### Next Servo Button (D3)

Cycles through the servo configurations defined in: servo_config.h


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

Fields appear only when the corresponding data exists.


---

## Servo Configuration

Servo configurations are defined in: servo_config.h

Each entry describes one servo profile, including:

- **name**: Human-readable identifier of the servo configuration.

- **pwm_pin**: Arduino pin used to generate the PWM signal for the servo.

- **servo_min_deg**: Minimum physical angle of the servo.

- **servo_max_deg**: Maximum physical angle of the servo.

- **allowed_min_deg**: Minimum allowed operating angle for this application.

- **allowed_max_deg**: Maximum allowed operating angle for this application.

- **rest_deg**: Rest or neutral position used for initialization.

- **pwm_min_us**: Pulse width corresponding to the minimum servo position.

- **pwm_max_us**: Pulse width corresponding to the maximum servo position.

- **max_speed_degps**: Maximum servo speed in degrees per second.

- **default_speed_pct**: Default speed percentage used by the controller.

- **default_accel_pct**: Default acceleration percentage used by the controller.

- **feedback_adc_pin**: Analog pin used to read servo position feedback.

- **fb_adc_at_servo_min_deg**: ADC value measured when the servo is at its minimum angle.

- **fb_adc_at_servo_max_deg**: ADC value measured when the servo is at its maximum angle.

- **inverted**: Indicates whether the servo direction must be inverted.

- **fault_detection_enabled**: Enables or disables fault detection logic.

Example:

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
        3,            // feedback_adc_pin  (A3 on Arduino Nano)
        101,          // fb_adc_at_servo_min_deg
        383,          // fb_adc_at_servo_max_deg
        false,        // inverted
        false         // fault_detection_enabled
    }

---

## Memory Optimization

Servo configurations are stored in PROGMEM (Flash memory) instead of SRAM.

This allows defining many servo profiles without exhausting RAM. Memory Optimization

Servo configurations are stored in PROGMEM (Flash memory) instead of SRAM.

This allows defining many servo profiles without exhausting RAM.

---

## Servo calibration procedure

This section describes a simple procedure to obtain the parameters required in `servo_config.h` when using the **ServoController library**.

Servo configuration parameters come from three different sources:

- **Servo model** (electrical and internal characteristics)
- **Mechanical joint** (limits imposed by the robot articulation)
- **Application settings** (desired behavior)

A convenient calibration tool is the program:

https://github.com/aalonsopuig/Servo_tester

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

### 5. Feedback calibration (optional)

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

### 6. Final configuration

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
| Hitec HS-805BB     | 0            | 180          | 700       | 2400      | 428.6          |
| TowerPro SG-5010   | 0            | 180          | 500       | 1800      | 375.0          |
| HobbyKing HK15298  | —            | —            | —         | —         | 461.5          |
| Miuzei MF90        | 0            | 180          | 500       | 2500      | 750.0          |
| Corona DS929HV     | —            | —            | —         | —         | 600.0          |
| Futaba S3003       | —            | —            | —         | —         | 315.8          |
| DIYMore DM996      | 0            | 180          | 500       | 2500      | 400.0          |

Note: PWM limits are approximate values measured experimentally and may vary
between units. Always verify safe limits before operating the servo in a
mechanical assembly.