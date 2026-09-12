# Drone Flight Controller

A quadcopter flight controller built from scratch on a Teensy 4.0,
reading a 6-axis IMU and computing a stable attitude estimate with
a complementary filter and dual-axis PID control.

## What it does

- Reads raw gyroscope and accelerometer data from an MPU-6050 over I2C
- Fuses both sensors into a reliable tilt angle using a complementary filter
  (gyro for short-term precision, accelerometer for long-term drift correction)
- Computes independent PID corrections for the roll and pitch axes
- Outputs angle and correction values over serial for tuning and debugging

## Hardware

- Teensy 4.0 (ARM Cortex-M7 microcontroller)
- MPU-6050 (GY-521 breakout) — 6-axis IMU (gyroscope + accelerometer)
- I2C bus: SDA on pin 18, SCL on pin 19

## How it works

The main loop follows a clear 4-stage chain:

1. **Read** — fetch raw gyro and accelerometer values over I2C
2. **Fuse** — combine them into roll and pitch angles (complementary filter)
3. **Correct** — compute a PID correction for each axis
4. **Output** — send angle and correction over serial

Each stage is a dedicated function, sharing data through global state.

## Key concepts

- **Sensor fusion**: the gyroscope drifts over time; the accelerometer is
  noisy but stable. The complementary filter keeps the best of both.
- **PID control**: proportional + integral + derivative correction, tuned
  per axis, to bring each tilt angle back to the target.

## Status

Work in progress — attitude estimation and dual-axis PID implemented and
tested on bench. Next steps: motor output (ESC + PWM) and flight testing.

## Author

[Ton nom] — returning to embedded/aerospace engineering.
