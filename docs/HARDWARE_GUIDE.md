# Drone Flight Controller - Hardware Guide

## Architecture
```
IMU (MPU6050) --> Madgwick AHRS --> Cascaded PID --> ESC/Motors
Barometer -----> Kalman Filter --> Altitude Hold
GPS -----------> Kalman Filter --> Position Hold
ESP32-C6 ------> Telemetry Dashboard
SD Card -------> Blackbox Logger
```

## Hardware Requirements
| Component | Model | Purpose |
|-----------|-------|---------|
| MCU | ESP32-S3 | Main flight controller |
| IMU | MPU6050/ICM20948 | Attitude sensing |
| Barometer | BMP280 | Altitude estimation |
| GPS | NEO-6M/8M | Position hold |
| ESC | 4x 30A BLHeli | Motor control |
| Telemetry | ESP32-C6 | Dashboard link |

## PID Tuning
| Parameter | Start Value | Description |
|-----------|------------|-------------|
| Roll P | 4.0 | Proportional response |
| Roll I | 0.02 | Steady-state error |
| Roll D | 30.0 | Damping oscillation |

## Building
```bash
idf.py set-target esp32s3
idf.py build
idf.py flash monitor
```