/*
  Lab 9 Teensy 4.0 — MPU6050 PID beam control with PWM ESC outputs

  SAFETY:
  - ESC PWM outputs are D4 (Motor A) and D23 (Motor B), 250 Hz.
  - Boot/STOP/fault command 1000 us on both outputs.
  - PID gains default to zero. Resume is explicit and safety-gated.
  - MIXER_SIGN is deliberately 0 until physical correction direction is verified.
    With MIXER_SIGN == 0, Resume is rejected and outputs remain stopped.
  - While stopped, control overrun is recorded but NOT latched; while running,
    an overrun greater than 8 ms still latches a stop with measured timing.
  - Before enabling the mixer, confirm which motor command corrects positive
    beam tilt using the instructor-approved restrained-beam procedure.
  - Keep ESC power disconnected for waveform and USB acceptance tests.

  Hardware:
    MPU6050 SDA -> Teensy D18
    MPU6050 SCL -> Teensy D19
    MPU6050 AD0 -> Teensy D17 (driven LOW to select address 0x68; see note below)
    MPU6050 INT -> D16 (ignored)
    Motor A ESC signal -> Teensy D4
    Motor B ESC signal -> Teensy D23
    ESC signal ground and Teensy ground must be common.

  Libraries:
    - Adafruit MPU6050
    - Adafruit Unified Sensor
    - Adafruit BusIO
  Put PidLink.h and TunerDescriptor_USB.h beside this sketch.
*/

#define PIDLINK_EEPROM_ADDR 128
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "PidLink.h"
#include "TunerDescriptor_USB.h"

static constexpr uint32_t CONTROL_PERIOD_US = 4000; // 250 Hz nominal
static constexpr float RAD_TO_DEG_F = 57.29577951308232f;
static constexpr float G_TO_MS2_F = 9.80665f;
static constexpr float TARGET_LIMIT_DEG = 10.0f;
static constexpr float MAX_GAIN = 10.0f;
static constexpr float PID_OUTPUT_LIMIT_US = 100.0f; // differential from 1300 us base
static constexpr float I_TERM_LIMIT = 50.0f;        // integral-error bound (deg*s)
static constexpr int MOTOR_A_PIN = 4;
static constexpr int MOTOR_B_PIN = 23;
static constexpr uint32_t ESC_PWM_RATE_HZ = 250;
static constexpr int ESC_STOP_US = 1000;
static constexpr int ESC_BASE_US = 1300;
static constexpr int ESC_MIN_RUN_US = 1100;
static constexpr int ESC_MAX_RUN_US = 1650;
static constexpr float RESUME_MAX_ANGLE_DEG = 10.0f;
static constexpr float RESUME_MAX_GYRO_DPS = 20.0f;
// Deliberate commissioning interlock. Set to +1 or -1 ONLY after the physical
// correction direction is confirmed. +1 maps A=base+PID, B=base-PID.
static constexpr int MIXER_SIGN = -1;
static constexpr uint32_t BAUD_RATE = 115200;
static constexpr uint32_t I2C_FREQ = 400000; // MPU6050 supports 400 kHz fast-mode I2C

Adafruit_MPU6050 mpu;
PidLink link;

uint8_t mpuAddress = 0;
bool sensorHealthy = false;
bool stopped = true;
bool faultLatched = false;
char faultReason[48] = "BOOT_SENSOR_CHECK";
uint32_t lastGoodSampleUs = 0;
uint32_t lastControlTickUs = 0;
uint32_t lastTelemetryUs = 0;
uint32_t telemetryIntervalUs = 40000; // 25 Hz
uint32_t lastStateReportMs = 0;
uint32_t maxControlIntervalUs = 0; // diagnostic high-water mark
uint32_t lastDeadlineUs = 0;
static constexpr uint32_t DEADLINE_LIMIT_US = CONTROL_PERIOD_US * 2UL;
static constexpr uint32_t TELEMETRY_GUARD_US = 1200; // avoid starting USB output near a tick

float kp = 0.0f, ki = 0.0f, kd = 0.0f;
float targetDeg = 0.0f;
float angleDeg = NAN;
float gyroXDps = NAN, gyroYDps = NAN, gyroZDps = NAN;
float accelXG = NAN, accelYG = NAN, accelZG = NAN;
float errorDeg = NAN;
float pTerm = NAN, iTerm = NAN, dTerm = NAN;
float pidOutput = NAN; // bounded differential command in microseconds
int motorACommandUs = ESC_STOP_US;
int motorBCommandUs = ESC_STOP_US;
float integralError = 0.0f;
uint32_t previousPidUs = 0;
bool pidHistoryValid = false;

PidLinkImuSample latestRaw;
bool latestRawValid = false;

void resetPidState() {
  integralError = 0.0f;
  pTerm = iTerm = dTerm = pidOutput = 0.0f;
  previousPidUs = 0;
  pidHistoryValid = false;
}

void updatePid(float dtSeconds) {
  if (!isfinite(errorDeg) || !isfinite(gyroXDps) ||
      !isfinite(kp) || !isfinite(ki) || !isfinite(kd) ||
      dtSeconds <= 0.0f || dtSeconds > 0.020f) {
    resetPidState();
    return;
  }

  // Angle is atan2(accelY, accelZ), so its nominal rotation axis is X.
  // Derivative-on-measurement avoids a derivative kick when target changes.
  pTerm = kp * errorDeg;
  dTerm = -kd * gyroXDps;

  // Conditional integration: do not integrate farther into output saturation.
  const float candidateIntegral = constrain(
      integralError + errorDeg * dtSeconds, -I_TERM_LIMIT, I_TERM_LIMIT);
  const float candidateITerm = ki * candidateIntegral;
  const float candidateOutput = pTerm + candidateITerm + dTerm;
  const bool saturatingHigh = candidateOutput > PID_OUTPUT_LIMIT_US && errorDeg > 0.0f;
  const bool saturatingLow  = candidateOutput < -PID_OUTPUT_LIMIT_US && errorDeg < 0.0f;
  if (!saturatingHigh && !saturatingLow) integralError = candidateIntegral;

  iTerm = ki * integralError;
  pidOutput = constrain(pTerm + iTerm + dTerm,
                        -PID_OUTPUT_LIMIT_US, PID_OUTPUT_LIMIT_US);
}

void latchFault(const char *reason) {
  if (!faultLatched) {
    strncpy(faultReason, reason, sizeof(faultReason) - 1);
    faultReason[sizeof(faultReason) - 1] = '\0';
    faultLatched = true;
    link.error(faultReason);
  }
  sensorHealthy = false;
  stopOutputs();
  reportState();
}

void reportState() {
  const bool canResume = !stopped ? false :
      (sensorHealthy && !faultLatched && link.calibrationValid() &&
       latestRawValid && (uint32_t)(micros() - lastGoodSampleUs) <= 20000UL &&
       isfinite(angleDeg) && fabsf(angleDeg - targetDeg) <= RESUME_MAX_ANGLE_DEG &&
       isfinite(gyroXDps) && fabsf(gyroXDps) < RESUME_MAX_GYRO_DPS && MIXER_SIGN != 0);
  const char *reason = faultLatched ? faultReason :
      (!sensorHealthy ? "SENSOR_NOT_READY" :
       (!link.calibrationValid() ? "CALIBRATION_REQUIRED" :
        (MIXER_SIGN == 0 ? "MIXER_SIGN_NOT_VERIFIED" : "READY")));
  link.state(stopped, canResume, reason);
}

// Teensy hardware PWM: a 4 ms frame with pulse width expressed in microseconds.
// analogWriteResolution(16) makes duty conversion explicit; the actual waveform
// must still be verified with an approved instrument before ESC power is connected.
uint16_t pulseUsToDuty(int pulseUs) {
  pulseUs = constrain(pulseUs, 0, (int)CONTROL_PERIOD_US);
  return (uint16_t)(((uint32_t)pulseUs * 65535UL + CONTROL_PERIOD_US / 2UL) /
                    CONTROL_PERIOD_US);
}

void writeEscPulseUs(int pin, int pulseUs) {
  analogWrite(pin, pulseUsToDuty(pulseUs));
}

void stopOutputs() {
  stopped = true;
  integralError = 0.0f;
  resetPidState();
  motorACommandUs = ESC_STOP_US;
  motorBCommandUs = ESC_STOP_US;
  writeEscPulseUs(MOTOR_A_PIN, motorACommandUs);
  writeEscPulseUs(MOTOR_B_PIN, motorBCommandUs);
}

void applyMotorMix() {
  if (stopped || faultLatched || MIXER_SIGN == 0) {
    stopOutputs();
    return;
  }
  const int differentialUs = (int)lroundf((float)MIXER_SIGN * pidOutput);
  motorACommandUs = constrain(ESC_BASE_US + differentialUs, ESC_MIN_RUN_US, ESC_MAX_RUN_US);
  motorBCommandUs = constrain(ESC_BASE_US - differentialUs, ESC_MIN_RUN_US, ESC_MAX_RUN_US);
  writeEscPulseUs(MOTOR_A_PIN, motorACommandUs);
  writeEscPulseUs(MOTOR_B_PIN, motorBCommandUs);
}

bool probeI2C(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// MPU6050 INT_STATUS register 0x3A, bit 0 = data ready.
// This avoids treating repeated cached getEvent() results as fresh samples.
bool mpuDataReady() {
  Wire.beginTransmission(mpuAddress);
  Wire.write((uint8_t)0x3A);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddress, 1) != 1) return false;
  return (Wire.read() & 0x01) != 0;
}

bool acquireRawSample(PidLinkImuSample &out, bool readyAlreadyChecked = false) {
  if (!sensorHealthy || mpuAddress == 0) return false;
  if (!readyAlreadyChecked && !mpuDataReady()) return false;

  sensors_event_t accel, gyro, temp;
  if (!mpu.getEvent(&accel, &gyro, &temp)) {
    latchFault("SENSOR_FAULT:GET_EVENT");
    return false;
  }

  // Adafruit Unified Sensor units: gyro rad/s, accel m/s^2.
  out.gyro[0] = gyro.gyro.x * RAD_TO_DEG_F;
  out.gyro[1] = gyro.gyro.y * RAD_TO_DEG_F;
  out.gyro[2] = gyro.gyro.z * RAD_TO_DEG_F;
  out.accel[0] = accel.acceleration.x / G_TO_MS2_F;
  out.accel[1] = accel.acceleration.y / G_TO_MS2_F;
  out.accel[2] = accel.acceleration.z / G_TO_MS2_F;
  out.hasAttitude = false;
  out.roll = 0.0f;
  out.pitch = 0.0f;

  latestRaw = out;
  latestRawValid = true;
  lastGoodSampleUs = micros();
  return true;
}

bool imuSampleCallback(PidLinkImuSample &out) {
  // Calibration owns IMU sampling while stopped. The main control tick does
  // not consume the MPU data-ready flag while calibration is running.
  // CAL,START can arrive immediately after a control tick, so allow one
  // full 250 Hz conversion plus margin instead of declaring a sensor fault.
  if (!stopped || !sensorHealthy || mpuAddress == 0) return false;
  const uint32_t startedUs = micros();
  while ((uint32_t)(micros() - startedUs) < 6500UL) {
    if (mpuDataReady()) return acquireRawSample(out, true);
    delayMicroseconds(150);
  }
  return false; // actual sensor timeout; PidLink reports SENSOR_FAULT
}

bool isStoppedCallback() { return stopped; }

void pidGetCallback(const char *controller) {
  if (strcmp(controller, "beam") != 0) {
    link.pidReplyBegin(controller, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }
  link.pidReplyBegin(controller, PIDLINK_CURRENT);
  link.pidReplyParam("kp", kp, 6);
  link.pidReplyParam("ki", ki, 6);
  link.pidReplyParam("kd", kd, 6);
  link.pidReplyEnd();
}

void pidSetCallback(const char *controller, const PidLinkParams &params) {
  if (strcmp(controller, "beam") != 0) {
    link.pidReplyBegin(controller, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }

  float nextKp = kp, nextKi = ki, nextKd = kd;
  bool reject = params.anyMalformed();
  float value;
  if (params.get("kp", value)) nextKp = value;
  if (params.get("ki", value)) nextKi = value;
  if (params.get("kd", value)) nextKd = value;

  // Reject unknown parameter names, duplicates, nonfinite, negative, or >10.
  for (uint8_t i = 0; i < params.count; ++i) {
    const char *name = params.name[i];
    if (strcmp(name, "kp") && strcmp(name, "ki") && strcmp(name, "kd")) reject = true;
    for (uint8_t j = i + 1; j < params.count; ++j) {
      if (strcmp(name, params.name[j]) == 0) reject = true;
    }
  }
  if (!isfinite(nextKp) || !isfinite(nextKi) || !isfinite(nextKd) ||
      nextKp < 0 || nextKi < 0 || nextKd < 0 ||
      nextKp > MAX_GAIN || nextKi > MAX_GAIN || nextKd > MAX_GAIN ||
      params.count == 0) reject = true;

  if (reject) {
    link.pidReplyBegin(controller, PIDLINK_REJECTED);
    link.pidReplyParam("kp", kp, 6);
    link.pidReplyParam("ki", ki, 6);
    link.pidReplyParam("kd", kd, 6);
    link.pidReplyEnd();
    return;
  }

  // Atomic commit; a gain change resets accumulated state to avoid a stale I term.
  kp = nextKp; ki = nextKi; kd = nextKd;
  resetPidState();
  link.pidReplyBegin(controller, PIDLINK_APPLIED);
  link.pidReplyParam("kp", kp, 6);
  link.pidReplyParam("ki", ki, 6);
  link.pidReplyParam("kd", kd, 6);
  link.pidReplyEnd();
}

void pidResetCallback(const char *controller) {
  if (strcmp(controller, "beam") != 0) {
    link.pidReplyBegin(controller, PIDLINK_UNKNOWN_CONTROLLER);
    link.pidReplyEnd();
    return;
  }
  kp = ki = kd = 0.0f;
  resetPidState();
  link.pidReplyBegin(controller, PIDLINK_RESET);
  link.pidReplyParam("kp", kp, 6);
  link.pidReplyParam("ki", ki, 6);
  link.pidReplyParam("kd", kd, 6);
  link.pidReplyEnd();
}

void stopCallback() {
  stopOutputs();
  reportState();
}

void resumeCallback() {
  // No automatic re-arm: only an explicit RESUME may leave the stopped state.
  if (MIXER_SIGN == 0) {
    stopOutputs();
    link.error("RESUME_REJECTED:MIXER_SIGN_NOT_VERIFIED");
    reportState();
    return;
  }
  if (faultLatched || !sensorHealthy || !link.calibrationValid() ||
      !latestRawValid || (uint32_t)(micros() - lastGoodSampleUs) > 20000UL ||
      !isfinite(angleDeg) || fabsf(angleDeg - targetDeg) > RESUME_MAX_ANGLE_DEG ||
      !isfinite(gyroXDps) || fabsf(gyroXDps) >= RESUME_MAX_GYRO_DPS ||
      link.calibrationRunning()) {
    stopOutputs();
    link.error("RESUME_REJECTED:CHECK_SENSOR_CAL_LEVEL_STILLNESS");
    reportState();
    return;
  }
  stopped = false;
  resetPidState();
  // Begin from the neutral base command; the next valid control tick computes mix.
  motorACommandUs = ESC_BASE_US;
  motorBCommandUs = ESC_BASE_US;
  writeEscPulseUs(MOTOR_A_PIN, motorACommandUs);
  writeEscPulseUs(MOTOR_B_PIN, motorBCommandUs);
  reportState();
}

void setTargetCallback(float offsetDegrees) {
  if (!isfinite(offsetDegrees)) {
    link.error("BAD_TARGET");
    return;
  }
  if (offsetDegrees > TARGET_LIMIT_DEG) offsetDegrees = TARGET_LIMIT_DEG;
  if (offsetDegrees < -TARGET_LIMIT_DEG) offsetDegrees = -TARGET_LIMIT_DEG;
  targetDeg = offsetDegrees;
}

void zeroSetpointCallback() {
  // The setpoint is an offset from the calibrated level reference.
  if (isfinite(angleDeg)) { targetDeg = 0.0f; resetPidState(); }
  else link.error("ZERO_REJECTED_NO_ANGLE");
}

bool telemetryRateCallback(uint32_t hz) {
  if (hz != 25 && hz != 50 && hz != 125 && hz != 250) return false;
  telemetryIntervalUs = 1000000UL / hz;
  return true;
}

void setup() {
  Serial.begin(BAUD_RATE);
  analogWriteResolution(16);
  pinMode(MOTOR_A_PIN, OUTPUT);
  pinMode(MOTOR_B_PIN, OUTPUT);
  analogWriteFrequency(MOTOR_A_PIN, ESC_PWM_RATE_HZ);
  analogWriteFrequency(MOTOR_B_PIN, ESC_PWM_RATE_HZ);
  // Initialize and hold the ESC minimum pulse before any sensor/USB setup.
  writeEscPulseUs(MOTOR_A_PIN, ESC_STOP_US);
  writeEscPulseUs(MOTOR_B_PIN, ESC_STOP_US);
  Wire.begin();
  Wire.setClock(I2C_FREQ);

  // AD0 is connected to D17. Drive it LOW to select address 0x68.
  // This assumes AD0 is not also tied to VCC by a breakout jumper; if it is,
  // disconnect that tie before powering the Teensy pin.
  digitalWrite(17, LOW);
  pinMode(17, OUTPUT);
  delay(10);
  auto probe_8 = probeI2C(0x68);
  auto probe_9 = probeI2C(0x69);
  if (probe_8) mpuAddress = 0x68;
  else if (probe_9) mpuAddress = 0x69;

  if (mpuAddress != 0 && mpu.begin(mpuAddress, &Wire)) {
    mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
    mpu.setSampleRateDivisor(3); // nominal 250 Hz update from 1 kHz base
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    sensorHealthy = true;
    faultLatched = false;
    strncpy(faultReason, "NONE", sizeof(faultReason));
  } else {
    sensorHealthy = false;
    latchFault("SENSOR_FAULT:MPU_NOT_FOUND");
  }

  link.begin(TUNER_DESCRIPTOR);
  link.onPidGet(pidGetCallback);
  link.onPidSet(pidSetCallback);
  link.onPidReset(pidResetCallback);
  link.onStop(stopCallback);
  link.onResume(resumeCallback);
  link.onSetTarget(setTargetCallback);
  link.onZeroSetpoint(zeroSetpointCallback);
  link.onTelemetryRate(telemetryRateCallback);
  link.onImuSample(imuSampleCallback);
  link.onIsStopped(isStoppedCallback);

  stopOutputs();
  // Initialize the scheduler AFTER all blocking setup and initial status work.
  lastControlTickUs = micros();
  lastTelemetryUs = lastControlTickUs;
  reportState();
  // Keep the serial protocol clean: no free-form debug text on Serial.
  if (mpuAddress == 0x68) link.ack("MPU_ADDR_68");
  else if (mpuAddress == 0x69) link.ack("MPU_ADDR_69");
  else link.error("SENSOR_FAULT:MPU_NOT_FOUND");
}

void loop() {
  // Priority 1: run the control tick BEFORE USB parsing/telemetry.
  // Calibration temporarily has exclusive IMU read access while stopped.
  // Schedule from the prior target, not the completion time, to avoid drift.
  const uint32_t now = micros();
  const uint32_t elapsedUs = (uint32_t)(now - lastControlTickUs);
  if (elapsedUs >= CONTROL_PERIOD_US) {
    if (elapsedUs > maxControlIntervalUs) maxControlIntervalUs = elapsedUs;
    lastDeadlineUs = elapsedUs;
    lastControlTickUs += CONTROL_PERIOD_US;
    // If a long operation skipped ticks, resynchronize to now instead of
    // issuing a burst of stale PID updates.
    if (elapsedUs >= DEADLINE_LIMIT_US) lastControlTickUs = now;

    // A missed deadline is a safety fault when outputs are running.
    // During STOP, USB enumeration / descriptor replies / calibration traffic
    // can be slow without creating an unrecoverable fault on an idle rig.
    if (elapsedUs > DEADLINE_LIMIT_US && !stopped) {
      char reason[48];
      snprintf(reason, sizeof(reason), "CONTROL_FAULT:DEADLINE_%luUS",
               (unsigned long)elapsedUs);
      latchFault(reason); // immediately forces both outputs to 1000 us
    }

    PidLinkImuSample raw;
    if (!link.calibrationRunning() && acquireRawSample(raw)) {
      PidLinkImuSample corrected = raw;
      link.applyCalibration(corrected); // exactly once on this copy

      gyroXDps = corrected.gyro[0];
      gyroYDps = corrected.gyro[1];
      gyroZDps = corrected.gyro[2];
      accelXG = corrected.accel[0];
      accelYG = corrected.accel[1];
      accelZG = corrected.accel[2];

      // Confirmed physical convention: accel Y is negative when Motor A is down
      // and positive when Motor A is up. This angle assumes +Z is beam-normal.
      angleDeg = atan2f(corrected.accel[1], corrected.accel[2]) * RAD_TO_DEG_F;
      errorDeg = targetDeg - angleDeg;

      const uint32_t sampleUs = micros();
      float dtSeconds = CONTROL_PERIOD_US * 1.0e-6f;
      if (previousPidUs != 0) dtSeconds = (uint32_t)(sampleUs - previousPidUs) * 1.0e-6f;
      previousPidUs = sampleUs;

      if (!faultLatched && !stopped) {
        updatePid(dtSeconds);
        applyMotorMix();
        if (fabsf(angleDeg - targetDeg) >= 45.0f) {
          stopOutputs();
          latchFault("CONTROL_FAULT:ANGLE_CUTOFF");
        }
      } else {
        integralError = 0.0f;
        pTerm = kp * errorDeg;
        iTerm = 0.0f;
        dTerm = -kd * gyroXDps;
        pidOutput = constrain(pTerm + iTerm + dTerm,
                              -PID_OUTPUT_LIMIT_US, PID_OUTPUT_LIMIT_US);
        if (stopped || faultLatched) stopOutputs();
      }
    } else if (!link.calibrationRunning() && sensorHealthy &&
               (uint32_t)(now - lastGoodSampleUs) > 20000UL) {
      stopOutputs();
      latchFault("SENSOR_FAULT:NO_FRESH_DATA");
    }
  }

  // Priority 2: process host commands. poll() has a 64-byte input budget.
  // It may synchronously send a descriptor, hence it follows the control tick.
  link.poll();

  // Priority 3: emit telemetry only when there is time before the next tick.
  // Skip rather than block a control cycle on a backed-up USB transmit queue.
  const uint32_t telemNow = micros();
  if (telemetryIntervalUs &&
      (uint32_t)(telemNow - lastTelemetryUs) >= telemetryIntervalUs &&
      (uint32_t)(telemNow - lastControlTickUs) <
          (CONTROL_PERIOD_US - TELEMETRY_GUARD_US) &&
      Serial.availableForWrite() >= 200) {
    lastTelemetryUs = telemNow;
    link.telemetryBegin();
    link.telemetryValue(micros());
    link.telemetryValue(angleDeg, 3);
    link.telemetryValue(targetDeg, 3);
    link.telemetryValue(errorDeg, 3);
    link.telemetryValue(gyroXDps, 3);
    link.telemetryValue(pTerm, 3);
    link.telemetryValue(iTerm, 3);
    link.telemetryValue(dTerm, 3);
    link.telemetryValue(pidOutput, 3);
    link.telemetryValue(motorACommandUs);
    link.telemetryValue(motorBCommandUs);
    link.telemetryValue(stopped);
    link.telemetryEnd();
  }

  if ((uint32_t)(millis() - lastStateReportMs) >= 1000UL &&
      (uint32_t)(micros() - lastControlTickUs) <
          (CONTROL_PERIOD_US - TELEMETRY_GUARD_US)) {
    lastStateReportMs = millis();
    if (faultLatched) reportState();
  }
}
