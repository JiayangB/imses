/*
  Elbow Sleeve System V4
  Board: Seeed Studio XIAO nRF52840 Sense

  BLE architecture has been simplified for stability:
  - vitalsPacket: heart rate, SpO2, temperature, MAX30102 raw levels, status
  - motionPacket: flex raw, EMG raw, NTC raw, angle, angular velocity,
                  angular acceleration, bend speed, motor state telemetry
  - imuPacket: raw accel XYZ + raw gyro XYZ

  Serial Plotter columns
  flexRaw emgRaw temperatureC heartRateBpm spo2Pct currentAngleDeg angularVelocityDegPerSec
  angularAccelerationDegPerSec2 bendClosingSpeedDegPerSec motorState motorPhaseRemainingMs
  imuAxRaw imuAyRaw imuAzRaw imuGxRaw imuGyRaw imuGzRaw

  Required libraries
  - ArduinoBLE
  - SparkFun MAX3010x Pulse and Proximity Sensor Library
*/

#include <ArduinoBLE.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

const uint8_t FLEX_PIN = A0;
const uint8_t EMG_PIN = A1;
const uint8_t NTC_PIN = A3;
const uint8_t MOTOR_IN1_PIN = D2;
const uint8_t MOTOR_IN2_PIN = D6;
const uint8_t MAX30102_I2C_ADDR = 0x57;

const unsigned long SERIAL_BAUD = 115200;
const uint16_t SENSOR_SAMPLE_HZ = 50;
const uint16_t PLOT_OUTPUT_HZ = 20;
const uint16_t BLE_UPDATE_HZ = 8;
const uint32_t EXTERNAL_I2C_CLOCK_HZ = 100000UL;
const uint32_t INTERNAL_I2C_CLOCK_HZ = 100000UL;

const float ADC_MAX = 4095.0f;
const float NTC_FIXED_RESISTOR_OHMS = 300.0f;
const float NTC_R25_OHMS = 10.0f;
const float NTC_BETA = 2950.0f;
const float NTC_T0_KELVIN = 298.15f;

const float FLEX_STRAIGHT_RAW = 1100.0f;
const float FLEX_BENT_RAW = 2100.0f;
const float FLEX_STRAIGHT_ANGLE_DEG = 180.0f;
const float FLEX_BENT_ANGLE_DEG = 40.0f;
const float FLEX_FILTER_ALPHA = 0.18f;
const float ANGULAR_ACCEL_FILTER_ALPHA = 0.22f;
const float BEND_SPEED_TRIGGER_DEG_PER_SEC = 200.0f;
const uint8_t BEND_SPEED_TRIGGER_CONFIRM_SAMPLES = 2;

// No encoder is present, so the motor phase duration is capped conservatively
// from the highest expected RPM to keep commanded cable travel at or below
// the equivalent of three motor turns.
const float MOTOR_TARGET_ROTATIONS = 3.0f;
const float MOTOR_MAX_RPM_ASSUMPTION = 200.0f;
const float MOTOR_RUNTIME_SAFETY_FACTOR = 0.95f;
const unsigned long MOTOR_SETTLE_MS = 2000UL;

const uint8_t IMU_I2C_ADDR_PRIMARY = 0x6A;
const uint8_t IMU_I2C_ADDR_SECONDARY = 0x6B;
const uint8_t IMU_REG_WHO_AM_I = 0x0F;
const uint8_t IMU_REG_CTRL1_XL = 0x10;
const uint8_t IMU_REG_CTRL2_G = 0x11;
const uint8_t IMU_REG_CTRL3_C = 0x12;
const uint8_t IMU_REG_OUTX_L_G = 0x22;
const uint8_t IMU_REG_OUTX_L_XL = 0x28;
const uint8_t IMU_WHO_AM_I_VALUE_1 = 0x69;
const uint8_t IMU_WHO_AM_I_VALUE_2 = 0x6A;
const uint8_t IMU_INTERNAL_POWER_PIN = 14;
const uint8_t IMU_INTERNAL_SCL_PIN = 15;
const uint8_t IMU_INTERNAL_SDA_PIN = 16;
const uint8_t IMU_INTERNAL_PULLUP_PIN = D29;
const uint8_t IMU_INTERNAL_ENV_POWER_PIN = D30;

const uint8_t MAX30102_BUFFER_SIZE = 100;
const uint8_t MAX30102_RECALC_SAMPLES = 25;
const uint32_t MIN_IR_FOR_FINGER = 12000;

const unsigned long SENSOR_SAMPLE_PERIOD_MS = 1000UL / SENSOR_SAMPLE_HZ;
const unsigned long PLOT_OUTPUT_PERIOD_MS = 1000UL / PLOT_OUTPUT_HZ;
const unsigned long BLE_UPDATE_PERIOD_MS = 1000UL / BLE_UPDATE_HZ;

const char *DEVICE_NAME = "ElbowSleeveV4";

BLEService elbowService("19B10010-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic vitalsPacketChar("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20, true);
BLECharacteristic motionPacketChar("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 20, true);
BLECharacteristic imuPacketChar("19B10013-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify, 14, true);

TwoWire internalImuWire(IMU_INTERNAL_SDA_PIN, IMU_INTERNAL_SCL_PIN);
MAX30105 pulseSensor;
TwoWire *imuWire = nullptr;

enum MotorProtectionState : uint8_t {
  MOTOR_STATE_IDLE = 0,
  MOTOR_STATE_FORWARD = 1,
  MOTOR_STATE_SETTLE = 2,
  MOTOR_STATE_REVERSE = 3
};

uint32_t redRing[MAX30102_BUFFER_SIZE];
uint32_t irRing[MAX30102_BUFFER_SIZE];
uint8_t ringHead = 0;
uint8_t ringCount = 0;
uint8_t samplesSinceSpo2Calc = 0;

bool bleReady = false;
bool max30102Present = false;
bool imuPresent = false;
bool fingerPresent = false;
bool heartRateValid = false;
bool spo2Valid = false;
bool flexKinematicsReady = false;

int flexRaw = 0;
int emgRaw = 0;
int ntcRaw = 0;

int16_t imuAxRaw = 0;
int16_t imuAyRaw = 0;
int16_t imuAzRaw = 0;
int16_t imuGxRaw = 0;
int16_t imuGyRaw = 0;
int16_t imuGzRaw = 0;

uint32_t latestRedRaw = 0;
uint32_t latestIrRaw = 0;
uint16_t packetSequence = 0;
uint8_t imuI2cAddress = IMU_I2C_ADDR_PRIMARY;
uint8_t imuWhoAmI = 0;
uint8_t bendSpeedTriggerSamples = 0;
uint16_t motorCycleCount = 0;

float filteredFlexRaw = 0.0f;
float temperatureC = -1000.0f;
float heartRateBpm = -1.0f;
float spo2Pct = -1.0f;
float currentAngleDeg = FLEX_STRAIGHT_ANGLE_DEG;
float angularVelocityDegPerSec = 0.0f;
float angularAccelerationDegPerSec2 = 0.0f;
float bendClosingSpeedDegPerSec = 0.0f;

unsigned long lastBeatMs = 0;
unsigned long lastKinematicsSampleAtMs = 0;
unsigned long motorStateStartedAtMs = 0;
unsigned long motorStateDurationMs = 0;

unsigned long nextSensorSampleAtMs = 0;
unsigned long nextPlotOutputAtMs = 0;
unsigned long nextBleUpdateAtMs = 0;

MotorProtectionState motorProtectionState = MOTOR_STATE_IDLE;

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

int16_t clampToInt16(int32_t value) {
  if (value > 32767) {
    return 32767;
  }
  if (value < -32768) {
    return -32768;
  }
  return (int16_t)value;
}

void writeU16LE(uint8_t *buffer, int offset, uint16_t value) {
  buffer[offset] = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

void writeI16LE(uint8_t *buffer, int offset, int16_t value) {
  writeU16LE(buffer, offset, (uint16_t)value);
}

void writeU32LE(uint8_t *buffer, int offset, uint32_t value) {
  buffer[offset] = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
  buffer[offset + 3] = (uint8_t)((value >> 24) & 0xFF);
}

void writeI32LE(uint8_t *buffer, int offset, int32_t value) {
  writeU32LE(buffer, offset, (uint32_t)value);
}

unsigned long getMotorPhaseDurationMs() {
  const float minutes = MOTOR_TARGET_ROTATIONS / MOTOR_MAX_RPM_ASSUMPTION;
  return (unsigned long)(minutes * 60000.0f * MOTOR_RUNTIME_SAFETY_FACTOR);
}

unsigned long getMotorPhaseRemainingMs(unsigned long nowMs) {
  if (motorProtectionState == MOTOR_STATE_IDLE) {
    return 0;
  }

  const unsigned long elapsedMs = nowMs - motorStateStartedAtMs;
  if (elapsedMs >= motorStateDurationMs) {
    return 0;
  }
  return motorStateDurationMs - elapsedMs;
}

void setMotorDrive(int8_t direction) {
  if (direction > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
    return;
  }
  if (direction < 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
    return;
  }

  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

void enterMotorProtectionState(MotorProtectionState nextState, unsigned long nowMs, unsigned long durationMs) {
  motorProtectionState = nextState;
  motorStateStartedAtMs = nowMs;
  motorStateDurationMs = durationMs;

  if (nextState == MOTOR_STATE_FORWARD) {
    setMotorDrive(1);
  } else if (nextState == MOTOR_STATE_REVERSE) {
    setMotorDrive(-1);
  } else {
    setMotorDrive(0);
  }
}

void waitForSerialMonitor() {
  const unsigned long startMs = millis();
  while (!Serial && (millis() - startMs < 2000UL)) {
    delay(10);
  }
}

void configureAdc() {
#if defined(ARDUINO_ARCH_MBED) || defined(ARDUINO_ARCH_NRF52)
  analogReadResolution(12);
#endif
}

void configureMotorPinsSafe() {
  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  digitalWrite(MOTOR_IN1_PIN, LOW);
  digitalWrite(MOTOR_IN2_PIN, LOW);
}

bool i2cAddressPresent(TwoWire &bus, uint8_t address) {
  bus.beginTransmission(address);
  return bus.endTransmission() == 0;
}

void configureI2cBuses() {
  Wire.begin();
  Wire.setClock(EXTERNAL_I2C_CLOCK_HZ);

  pinMode(IMU_INTERNAL_PULLUP_PIN, OUTPUT);
  digitalWrite(IMU_INTERNAL_PULLUP_PIN, HIGH);

  pinMode(IMU_INTERNAL_ENV_POWER_PIN, OUTPUT);
  digitalWrite(IMU_INTERNAL_ENV_POWER_PIN, HIGH);

  pinMode(IMU_INTERNAL_POWER_PIN, OUTPUT);
  digitalWrite(IMU_INTERNAL_POWER_PIN, HIGH);

  delay(30);
  internalImuWire.begin();
  internalImuWire.setClock(INTERNAL_I2C_CLOCK_HZ);
  delay(10);
}

void configureBle() {
  bleReady = BLE.begin();
  if (!bleReady) {
    return;
  }

  BLE.setLocalName(DEVICE_NAME);
  BLE.setDeviceName(DEVICE_NAME);
  BLE.setAdvertisedService(elbowService);

  elbowService.addCharacteristic(vitalsPacketChar);
  elbowService.addCharacteristic(motionPacketChar);
  elbowService.addCharacteristic(imuPacketChar);
  BLE.addService(elbowService);

  uint8_t zero20[20] = {0};
  uint8_t zero14[14] = {0};
  vitalsPacketChar.writeValue(zero20, sizeof(zero20));
  motionPacketChar.writeValue(zero20, sizeof(zero20));
  imuPacketChar.writeValue(zero14, sizeof(zero14));

  BLE.advertise();
}

void configureMax30102() {
  max30102Present = false;

  if (!i2cAddressPresent(Wire, MAX30102_I2C_ADDR)) {
    return;
  }

  max30102Present = pulseSensor.begin(Wire, I2C_SPEED_STANDARD);
  if (!max30102Present) {
    return;
  }

  const byte ledBrightness = 60;
  const byte sampleAverage = 4;
  const byte ledMode = 2;
  const int sampleRate = 100;
  const int pulseWidth = 411;
  const int adcRange = 4096;

  pulseSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
  pulseSensor.setPulseAmplitudeGreen(0);
}

bool imuWriteRegister(uint8_t reg, uint8_t value) {
  if (imuWire == nullptr) {
    return false;
  }

  imuWire->beginTransmission(imuI2cAddress);
  imuWire->write(reg);
  imuWire->write(value);
  return imuWire->endTransmission() == 0;
}

bool imuReadBytes(uint8_t reg, uint8_t *buffer, uint8_t length) {
  if (imuWire == nullptr) {
    return false;
  }

  imuWire->beginTransmission(imuI2cAddress);
  imuWire->write(reg);
  if (imuWire->endTransmission(false) != 0) {
    return false;
  }

  const uint8_t bytesRead = imuWire->requestFrom((int)imuI2cAddress, (int)length);
  if (bytesRead != length) {
    return false;
  }

  for (uint8_t i = 0; i < length; ++i) {
    buffer[i] = imuWire->read();
  }
  return true;
}

void configureImu() {
  imuPresent = false;
  imuWire = nullptr;
  imuWhoAmI = 0;

  const uint8_t candidateAddresses[] = { IMU_I2C_ADDR_PRIMARY, IMU_I2C_ADDR_SECONDARY };
  for (uint8_t i = 0; i < sizeof(candidateAddresses); ++i) {
    if (!i2cAddressPresent(internalImuWire, candidateAddresses[i])) {
      continue;
    }

    imuWire = &internalImuWire;
    imuI2cAddress = candidateAddresses[i];

    uint8_t whoAmI = 0;
    if (!imuReadBytes(IMU_REG_WHO_AM_I, &whoAmI, 1)) {
      continue;
    }
    if (whoAmI != IMU_WHO_AM_I_VALUE_1 && whoAmI != IMU_WHO_AM_I_VALUE_2) {
      continue;
    }

    imuWhoAmI = whoAmI;
    imuPresent =
      imuWriteRegister(IMU_REG_CTRL3_C, 0x44) &&
      imuWriteRegister(IMU_REG_CTRL1_XL, 0x40) &&
      imuWriteRegister(IMU_REG_CTRL2_G, 0x40);

    if (imuPresent) {
      delay(20);
      return;
    }
  }

  imuWire = nullptr;
}

float computeNtcResistanceOhms(int adcValue) {
  if (adcValue <= 0) {
    return -1.0f;
  }

  const float ratio = ADC_MAX / (float)adcValue;
  return NTC_FIXED_RESISTOR_OHMS * (ratio - 1.0f);
}

float computeTemperatureCFromResistance(float resistanceOhms) {
  if (resistanceOhms <= 0.0f) {
    return -1000.0f;
  }

  const float inverseT = (1.0f / NTC_T0_KELVIN) + (log(resistanceOhms / NTC_R25_OHMS) / NTC_BETA);
  return (1.0f / inverseT) - 273.15f;
}

float mapFlexRawToAngleDeg(float rawValue) {
  const float slope = (FLEX_BENT_ANGLE_DEG - FLEX_STRAIGHT_ANGLE_DEG) / (FLEX_BENT_RAW - FLEX_STRAIGHT_RAW);
  const float angle = FLEX_STRAIGHT_ANGLE_DEG + (rawValue - FLEX_STRAIGHT_RAW) * slope;
  return clampFloat(angle, FLEX_BENT_ANGLE_DEG, FLEX_STRAIGHT_ANGLE_DEG);
}

void updateFlexKinematics(unsigned long nowMs) {
  if (!flexKinematicsReady) {
    filteredFlexRaw = (float)flexRaw;
    currentAngleDeg = mapFlexRawToAngleDeg(filteredFlexRaw);
    angularVelocityDegPerSec = 0.0f;
    angularAccelerationDegPerSec2 = 0.0f;
    bendClosingSpeedDegPerSec = 0.0f;
    lastKinematicsSampleAtMs = nowMs;
    flexKinematicsReady = true;
    return;
  }

  const float dt = (nowMs - lastKinematicsSampleAtMs) / 1000.0f;
  lastKinematicsSampleAtMs = nowMs;
  if (dt <= 0.0f) {
    return;
  }

  filteredFlexRaw += FLEX_FILTER_ALPHA * ((float)flexRaw - filteredFlexRaw);

  const float previousAngleDeg = currentAngleDeg;
  const float previousAngularVelocityDegPerSec = angularVelocityDegPerSec;

  currentAngleDeg = mapFlexRawToAngleDeg(filteredFlexRaw);
  angularVelocityDegPerSec = (currentAngleDeg - previousAngleDeg) / dt;

  const float rawAngularAcceleration = (angularVelocityDegPerSec - previousAngularVelocityDegPerSec) / dt;
  angularAccelerationDegPerSec2 += ANGULAR_ACCEL_FILTER_ALPHA * (rawAngularAcceleration - angularAccelerationDegPerSec2);
  bendClosingSpeedDegPerSec = angularVelocityDegPerSec < 0.0f ? -angularVelocityDegPerSec : 0.0f;
}

void maybeStartMotorProtectionCycle(unsigned long nowMs) {
  if (motorProtectionState != MOTOR_STATE_IDLE) {
    bendSpeedTriggerSamples = 0;
    return;
  }

  if (bendClosingSpeedDegPerSec >= BEND_SPEED_TRIGGER_DEG_PER_SEC) {
    if (bendSpeedTriggerSamples < 255) {
      bendSpeedTriggerSamples++;
    }
  } else {
    bendSpeedTriggerSamples = 0;
  }

  if (bendSpeedTriggerSamples < BEND_SPEED_TRIGGER_CONFIRM_SAMPLES) {
    return;
  }

  bendSpeedTriggerSamples = 0;
  motorCycleCount++;
  enterMotorProtectionState(MOTOR_STATE_FORWARD, nowMs, getMotorPhaseDurationMs());
}

void updateMotorProtection(unsigned long nowMs) {
  if (motorProtectionState == MOTOR_STATE_IDLE) {
    return;
  }

  if (getMotorPhaseRemainingMs(nowMs) > 0) {
    return;
  }

  if (motorProtectionState == MOTOR_STATE_FORWARD) {
    enterMotorProtectionState(MOTOR_STATE_SETTLE, nowMs, MOTOR_SETTLE_MS);
    return;
  }

  if (motorProtectionState == MOTOR_STATE_SETTLE) {
    enterMotorProtectionState(MOTOR_STATE_REVERSE, nowMs, getMotorPhaseDurationMs());
    return;
  }

  enterMotorProtectionState(MOTOR_STATE_IDLE, nowMs, 0);
}

void sampleImuRaw() {
  if (!imuPresent) {
    imuAxRaw = 0;
    imuAyRaw = 0;
    imuAzRaw = 0;
    imuGxRaw = 0;
    imuGyRaw = 0;
    imuGzRaw = 0;
    return;
  }

  uint8_t accelBytes[6];
  uint8_t gyroBytes[6];
  if (!imuReadBytes(IMU_REG_OUTX_L_XL, accelBytes, 6) ||
      !imuReadBytes(IMU_REG_OUTX_L_G, gyroBytes, 6)) {
    imuAxRaw = 0;
    imuAyRaw = 0;
    imuAzRaw = 0;
    imuGxRaw = 0;
    imuGyRaw = 0;
    imuGzRaw = 0;
    return;
  }

  imuAxRaw = (int16_t)((accelBytes[1] << 8) | accelBytes[0]);
  imuAyRaw = (int16_t)((accelBytes[3] << 8) | accelBytes[2]);
  imuAzRaw = (int16_t)((accelBytes[5] << 8) | accelBytes[4]);
  imuGxRaw = (int16_t)((gyroBytes[1] << 8) | gyroBytes[0]);
  imuGyRaw = (int16_t)((gyroBytes[3] << 8) | gyroBytes[2]);
  imuGzRaw = (int16_t)((gyroBytes[5] << 8) | gyroBytes[4]);
}

void sampleSensorFrame(unsigned long nowMs) {
  flexRaw = analogRead(FLEX_PIN);
  emgRaw = analogRead(EMG_PIN);
  ntcRaw = analogRead(NTC_PIN);

  const float ntcResistanceOhms = computeNtcResistanceOhms(ntcRaw);
  temperatureC = computeTemperatureCFromResistance(ntcResistanceOhms);

  updateFlexKinematics(nowMs);
  sampleImuRaw();
  maybeStartMotorProtectionCycle(nowMs);
  packetSequence++;
}

void resetPulseComputation() {
  ringHead = 0;
  ringCount = 0;
  samplesSinceSpo2Calc = 0;
  heartRateBpm = -1.0f;
  spo2Pct = -1.0f;
  heartRateValid = false;
  spo2Valid = false;
  latestRedRaw = 0;
  latestIrRaw = 0;
  lastBeatMs = 0;
}

void calculateSpo2FromRingBuffer() {
  if (ringCount < MAX30102_BUFFER_SIZE) {
    return;
  }

  uint32_t redBuffer[MAX30102_BUFFER_SIZE];
  uint32_t irBuffer[MAX30102_BUFFER_SIZE];

  for (uint8_t i = 0; i < MAX30102_BUFFER_SIZE; ++i) {
    const uint8_t index = (ringHead + i) % MAX30102_BUFFER_SIZE;
    redBuffer[i] = redRing[index];
    irBuffer[i] = irRing[index];
  }

  int32_t spo2 = 0;
  int8_t validSpo2 = 0;
  int32_t algorithmHeartRate = 0;
  int8_t validHeartRate = 0;

  maxim_heart_rate_and_oxygen_saturation(
    irBuffer,
    MAX30102_BUFFER_SIZE,
    redBuffer,
    &spo2,
    &validSpo2,
    &algorithmHeartRate,
    &validHeartRate
  );

  if (validHeartRate && algorithmHeartRate > 20 && algorithmHeartRate < 255) {
    heartRateBpm = (float)algorithmHeartRate;
    heartRateValid = true;
  } else {
    heartRateBpm = -1.0f;
    heartRateValid = false;
  }

  if (validSpo2 && spo2 >= 0 && spo2 <= 100) {
    spo2Pct = (float)spo2;
    spo2Valid = true;
  } else {
    spo2Pct = -1.0f;
    spo2Valid = false;
  }
}

void updateMax30102() {
  if (!max30102Present) {
    return;
  }

  pulseSensor.check();

  while (pulseSensor.available()) {
    latestRedRaw = pulseSensor.getRed();
    latestIrRaw = pulseSensor.getIR();
    pulseSensor.nextSample();

    fingerPresent = latestIrRaw >= MIN_IR_FOR_FINGER;

    if (!fingerPresent) {
      resetPulseComputation();
      continue;
    }

    if (checkForBeat((long)latestIrRaw)) {
      const long deltaMs = millis() - lastBeatMs;
      lastBeatMs = millis();

      if (deltaMs > 0) {
        const float bpm = 60.0f / (deltaMs / 1000.0f);
        if (bpm > 20.0f && bpm < 255.0f) {
          heartRateBpm = bpm;
          heartRateValid = true;
        }
      }
    }

    redRing[ringHead] = latestRedRaw;
    irRing[ringHead] = latestIrRaw;
    ringHead = (ringHead + 1) % MAX30102_BUFFER_SIZE;
    if (ringCount < MAX30102_BUFFER_SIZE) {
      ringCount++;
    }

    samplesSinceSpo2Calc++;
    if (ringCount == MAX30102_BUFFER_SIZE && samplesSinceSpo2Calc >= MAX30102_RECALC_SAMPLES) {
      samplesSinceSpo2Calc = 0;
      calculateSpo2FromRingBuffer();
    }
  }
}

byte buildStatusFlags() {
  byte flags = 0;
  if (max30102Present) {
    flags |= 0x01;
  }
  if (fingerPresent) {
    flags |= 0x02;
  }
  if (heartRateValid) {
    flags |= 0x04;
  }
  if (spo2Valid) {
    flags |= 0x08;
  }
  if (imuPresent) {
    flags |= 0x10;
  }
  if (motorProtectionState != MOTOR_STATE_IDLE) {
    flags |= 0x20;
  }
  return flags;
}

void updateBlePackets() {
  if (!bleReady) {
    return;
  }

  const byte flags = buildStatusFlags();

  const int16_t heartRateX10 = heartRateValid ? clampToInt16((int32_t)(heartRateBpm * 10.0f)) : (int16_t)-1;
  const int16_t spo2X10 = spo2Valid ? clampToInt16((int32_t)(spo2Pct * 10.0f)) : (int16_t)-1;
  const int16_t tempX100 = temperatureC > -500.0f ? clampToInt16((int32_t)(temperatureC * 100.0f)) : (int16_t)-32768;
  const int16_t angleX100 = clampToInt16((int32_t)(currentAngleDeg * 100.0f));
  const int16_t angularVelocityX10 = clampToInt16((int32_t)(angularVelocityDegPerSec * 10.0f));
  const int16_t angularAccelerationX10 = clampToInt16((int32_t)(angularAccelerationDegPerSec2 * 10.0f));
  const uint16_t bendClosingSpeedX10 = (uint16_t)clampFloat(bendClosingSpeedDegPerSec * 10.0f, 0.0f, 65535.0f);
  const uint16_t motorPhaseRemainingMs = (uint16_t)clampFloat((float)getMotorPhaseRemainingMs(millis()), 0.0f, 65535.0f);

  uint8_t vitalsPacket[20] = {0};
  writeU16LE(vitalsPacket, 0, packetSequence);
  vitalsPacket[2] = flags;
  vitalsPacket[3] = ringCount;
  writeI16LE(vitalsPacket, 4, heartRateX10);
  writeI16LE(vitalsPacket, 6, spo2X10);
  writeI16LE(vitalsPacket, 8, tempX100);
  writeU32LE(vitalsPacket, 10, latestRedRaw);
  writeU32LE(vitalsPacket, 14, latestIrRaw);
  vitalsPacket[18] = samplesSinceSpo2Calc;
  vitalsPacket[19] = fingerPresent ? 1 : 0;

  uint8_t motionPacket[20] = {0};
  writeU16LE(motionPacket, 0, packetSequence);
  writeU16LE(motionPacket, 2, (uint16_t)flexRaw);
  writeU16LE(motionPacket, 4, (uint16_t)emgRaw);
  writeU16LE(motionPacket, 6, (uint16_t)ntcRaw);
  writeI16LE(motionPacket, 8, angleX100);
  writeI16LE(motionPacket, 10, angularVelocityX10);
  writeI16LE(motionPacket, 12, angularAccelerationX10);
  writeU16LE(motionPacket, 14, bendClosingSpeedX10);
  motionPacket[16] = (uint8_t)motorProtectionState;
  motionPacket[17] = (uint8_t)(motorCycleCount & 0xFF);
  writeU16LE(motionPacket, 18, motorPhaseRemainingMs);

  uint8_t imuPacket[14] = {0};
  writeU16LE(imuPacket, 0, packetSequence);
  writeI16LE(imuPacket, 2, imuAxRaw);
  writeI16LE(imuPacket, 4, imuAyRaw);
  writeI16LE(imuPacket, 6, imuAzRaw);
  writeI16LE(imuPacket, 8, imuGxRaw);
  writeI16LE(imuPacket, 10, imuGyRaw);
  writeI16LE(imuPacket, 12, imuGzRaw);

  vitalsPacketChar.writeValue(vitalsPacket, sizeof(vitalsPacket));
  motionPacketChar.writeValue(motionPacket, sizeof(motionPacket));
  imuPacketChar.writeValue(imuPacket, sizeof(imuPacket));
}

void outputSerialPlotterFrame() {
  Serial.print(flexRaw);
  Serial.print('\t');
  Serial.print(emgRaw);
  Serial.print('\t');
  Serial.print(temperatureC, 2);
  Serial.print('\t');
  Serial.print(heartRateBpm, 1);
  Serial.print('\t');
  Serial.print(spo2Pct, 1);
  Serial.print('\t');
  Serial.print(currentAngleDeg, 2);
  Serial.print('\t');
  Serial.print(angularVelocityDegPerSec, 2);
  Serial.print('\t');
  Serial.print(angularAccelerationDegPerSec2, 2);
  Serial.print('\t');
  Serial.print(bendClosingSpeedDegPerSec, 2);
  Serial.print('\t');
  Serial.print((int)motorProtectionState);
  Serial.print('\t');
  Serial.print(getMotorPhaseRemainingMs(millis()));
  Serial.print('\t');
  Serial.print(imuAxRaw);
  Serial.print('\t');
  Serial.print(imuAyRaw);
  Serial.print('\t');
  Serial.print(imuAzRaw);
  Serial.print('\t');
  Serial.print(imuGxRaw);
  Serial.print('\t');
  Serial.print(imuGyRaw);
  Serial.print('\t');
  Serial.println(imuGzRaw);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  waitForSerialMonitor();

  pinMode(FLEX_PIN, INPUT);
  pinMode(EMG_PIN, INPUT);
  pinMode(NTC_PIN, INPUT);

  configureAdc();
  configureMotorPinsSafe();

  configureI2cBuses();
  configureMax30102();
  configureImu();
  configureBle();
  resetPulseComputation();

  nextSensorSampleAtMs = millis();
  nextPlotOutputAtMs = millis();
  nextBleUpdateAtMs = millis();
}

void loop() {
  if (bleReady) {
    BLE.poll();
  }

  updateMax30102();

  const unsigned long nowMs = millis();
  updateMotorProtection(nowMs);

  if ((long)(nowMs - nextSensorSampleAtMs) >= 0) {
    nextSensorSampleAtMs += SENSOR_SAMPLE_PERIOD_MS;
    sampleSensorFrame(nowMs);
  }

  if ((long)(nowMs - nextPlotOutputAtMs) >= 0) {
    nextPlotOutputAtMs += PLOT_OUTPUT_PERIOD_MS;
    outputSerialPlotterFrame();
  }

  if ((long)(nowMs - nextBleUpdateAtMs) >= 0) {
    nextBleUpdateAtMs += BLE_UPDATE_PERIOD_MS;
    updateBlePackets();
  }
}
