/*
 * Drone Flight Controller — Teensy 4.0 + MPU-6050
 * Reads IMU, fuses angles with a complementary filter,
 * computes dual-axis PID (roll & pitch).
 */

#include <Wire.h>

// ===== GLOBAL STATE (shared between functions) =====
int16_t gyroX, gyroY, gyroZ;          // raw gyroscope
int16_t accelX, accelY, accelZ;       // raw accelerometer
float angleRoll = 0, anglePitch = 0;  // fused angles
float temps_precedent = 0;
float dt;

// PID gains (shared, tuned later)
float Kp = 1.0, Ki = 0.0, Kd = 0.0;
float angle_voulu = 0;                // target: level (0°)

// PID memory — one set per axis
float somme_erreurs_roll = 0, erreur_precedente_roll = 0;
float somme_erreurs_pitch = 0, erreur_precedente_pitch = 0;
float correctionRoll, correctionPitch;

// ===== SETUP =====
void setup() {
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(0x68);   // wake up the MPU-6050
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  temps_precedent = micros();
}

// ===== 1. READ — get raw data over I2C =====
void readSensors() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);               // accelerometer registers
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  accelX = Wire.read() << 8 | Wire.read();
  accelY = Wire.read() << 8 | Wire.read();
  accelZ = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);               // gyroscope registers
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  gyroX = Wire.read() << 8 | Wire.read();
  gyroY = Wire.read() << 8 | Wire.read();
  gyroZ = Wire.read() << 8 | Wire.read();
}

// ===== 2. FUSE — complementary filter (roll & pitch) =====
void computeAngle() {
  dt = (micros() - temps_precedent) / 1000000.0;
  temps_precedent = micros();

  // Roll (rotation around X)
  float angle_accel_roll = atan2(accelY, accelZ) * 180.0 / PI;
  float gyro_roll = gyroX / 131.0;
  angleRoll = 0.98 * (angleRoll + gyro_roll * dt) + 0.02 * angle_accel_roll;

  // Pitch (rotation around Y)
  float angle_accel_pitch = atan2(accelX, accelZ) * 180.0 / PI;
  float gyro_pitch = gyroY / 131.0;
  anglePitch = 0.98 * (anglePitch + gyro_pitch * dt) + 0.02 * angle_accel_pitch;
}

// ===== 3. CORRECT — one PID per axis =====
void computePID() {
  // Roll
  float erreur_roll = angle_voulu - angleRoll;
  somme_erreurs_roll += erreur_roll * dt;
  float variation_roll = (erreur_roll - erreur_precedente_roll) / dt;
  correctionRoll = Kp * erreur_roll + Ki * somme_erreurs_roll + Kd * variation_roll;
  erreur_precedente_roll = erreur_roll;

  // Pitch
  float erreur_pitch = angle_voulu - anglePitch;
  somme_erreurs_pitch += erreur_pitch * dt;
  float variation_pitch = (erreur_pitch - erreur_precedente_pitch) / dt;
  correctionPitch = Kp * erreur_pitch + Ki * somme_erreurs_pitch + Kd * variation_pitch;
  erreur_precedente_pitch = erreur_pitch;
}

// ===== 4. OUTPUT — send over serial =====
void displayData() {
  Serial.print("Roll: ");  Serial.print(angleRoll);
  Serial.print("  Pitch: "); Serial.print(anglePitch);
  Serial.print("  CorrRoll: "); Serial.print(correctionRoll);
  Serial.print("  CorrPitch: "); Serial.println(correctionPitch);
}

// ===== LOOP — the chain, repeated forever =====
void loop() {
  readSensors();     // 1. read
  computeAngle();    // 2. fuse
  computePID();      // 3. correct
  displayData();     // 4. output
  delay(10);
}
