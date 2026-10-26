# Build your own AI implementation prompt

Use this checklist to plan your own prompt; it is not a prompt to copy. Give the assistant this entire kit, your rig notes and a clearly labeled sketch or photo.

## First choose the hardware and output protocol

Choose one IMU: HiLetgo MPU6050 (GY-521), Adafruit MPU6050 or Adafruit LSM6DSO32. Choose PWM or OneShot125 after checking ESC support. Then ask your AI for a complete Teensy 4.0 project that balances your beam and works with FlightBench. Build your own prompt using the checklist; attach the whole kit and labeled rig sketch.

## Describe the actual rig

- Teensy model, exact IMU module, library choices and all sensor bus/power connections. For LSM6DSO32 SPI include Mode 0, MSB first, 4 MHz and WHO_AM_I 0x6C; for MPU boards include the I²C pins/address. Verify module pinouts.
- Beam ends A/B, pivot axis and what counts as a positive tilt, viewed from a stated direction.
- Where IMU +X and +Y point relative to the two motors, +Z/board-face direction, and which axis measures the controlled rotation. Do not infer sensor axes or feedback sign from CW/CCW rotation.
- Each motor's physical location, measured CW/CCW rotation, matched propeller and ESC signal GPIO. Choose and document GPIOs compatible with your selected driver and other module connections.
- ESC type, selected PWM or OneShot125 protocol, verified support/frame rate, pulse-width units, stopped command, power arrangement and instructor-approved limits.

## Define the behavior and timing

- Hold the beam at a target angle and recover from a small disturbance using both motors.
- Use a 250 Hz control loop (4 ms nominal period), measured elapsed time and documented sensor/filter configuration. State the chosen ESC-supported output frame rate separately; measure both and describe them truthfully in the descriptor.
- Choose and explain your own angle estimator, controller architecture and motor mixing. Specify units, bounds and which motor command must change for each tilt.
- Start all PID gains at zero and motor outputs stopped. Require actual output STOP, deliberate Resume with valid calibration/sensor/level/stillness checks, bounded commands and stop on stale data or missed deadlines. No automatic rearming.

## Sensor setup hints for code generation

**MPU6050:** Explicitly configure the gyro and accelerometer sampling and filtering for the 250 Hz controller. With the digital low-pass filter enabled, a sample-rate divider of 3 produces a 250 Hz output rate. The approximately 42 Hz gyro / 44 Hz accelerometer filter setting is a starting option. Verify fresh measurements, axis directions and units, and apply calibration exactly once. Calculate software filter coefficients using the actual control timestep.

**LSM6DSO32:** Explicitly configure gyro and accelerometer output rates suitable for the 250 Hz controller; 416 Hz for both is a starting option. Configure and justify filtering, verify fresh measurements, and use the correct sensor class, measurement ranges, axis directions and units. Apply calibration exactly once and calculate software filter coefficients using the actual control timestep.

**LSM filtering:** At a 416 Hz sensor output rate, gyro LPF1 at approximately 48 Hz and accelerometer LPF2 at ODR/10 (41.6 Hz) are starting options. Enable the filters explicitly and verify their configuration. These are initial settings to evaluate, not guaranteed optimal settings. Filter high-frequency vibration before reducing measurements to the 250 Hz controller. [ST datasheet, Tables 60 and 65](https://www.st.com/resource/en/datasheet/lsm6dso32.pdf).

**Fresh measurements:** Use data-ready status to identify new measurements. Attempt to acquire available fresh data before declaring a previously cached sample stale. Reusing cached data must not refresh its timestamp. Enable block-data-update and automatic register increment when implementing burst reads.

**Orientation and correction direction:** Match both the accelerometer angle calculation and gyro axis/sign to the actual mounting. Verify that gyro integration and accelerometer tilt agree during movement. Changing sensor orientation may also require changing the motor-mixer signs. Recalibrate after changing the mounting.

**Fault reporting:** Report the specific failure—incorrect identity, configuration mismatch, missing fresh data or missed deadline. For register mismatches, include the register address, expected value and actual value. Keep outputs stopped after a latched fault until the required recovery checks and explicit Resume are completed.

**Adafruit LSM library:** Use Adafruit_LSM6DSO32 as the sensor class. Its getEvent() return value alone must not serve as the sensor-health or freshness check. [Adafruit driver](https://github.com/adafruit/Adafruit_LSM6DS/blob/master/Adafruit_LSM6DS.cpp).

## Calibration and recovery checks for either sensor

- Use the kit's 4000 microsecond calibration request interval and verify each accepted calibration sample represents an appropriate fresh conversion. Coordinate sensor reads with the 250 Hz control schedule.
- Acquire available fresh data before rejecting an old cache. After deliberately discarding pre-fault data, keep outputs stopped and allow a bounded first-conversion wait; do not immediately fault because the cache was intentionally cleared. Retain timeout, identity/configuration and explicit Resume checks.
- Separate or clear calibration storage when changing sensor type or mounting. Save/reboot/read must preserve the appropriate calibration only.
- Provide detailed comma-free fault reasons that remain visible after STOP/Resume attempts. Do not treat successful calibration as proof that all runtime fault conditions have cleared.

## Require app compatibility

- Use supplied `PidLink.h`; keep `TunerDescriptor.h` beside your sketch and update sensor type/bus, GPIOs, output protocol/rate, units and other metadata to describe your implementation. The MPU6050/I²C/OneShot125 example is not a mandated choice. Do not invent another serial protocol.
- Implement the get/set/reset, STOP/Resume, target/zero, telemetry-rate, raw-IMU and stopped-state callback responsibilities in `INTEGRATION.md`.
- Keep IDs, telemetry order, timestamps and units consistent. Report actual gains and P/I/D contributions. Validate partial gain changes atomically and preserve omitted fields.
- Apply calibration exactly once and implement only advertised features and sustainable telemetry rates. Service discovery and STOP promptly.

## Ask for work you can inspect

- A complete project, library requirements, explanations of signs/units/timing and a staged test plan.
- Clarification of missing hardware details rather than invented assumptions.
- USB-only acceptance checks with ESC power disconnected before powered testing; follow `ACCEPTANCE.md` and the instructor's restrained-beam procedure.

Revise your prompt after checking the physical rig and the assistant's assumptions. Keep your prompt and code available for the TA discussion; no course-website upload is required.
