# ESP32 Drone Flight Controller - V13.8

![Status](https://img.shields.io/badge/Status-FLIGHT_READY-green)
![Platform](https://img.shields.io/badge/Platform-ESP32-blue)
![Author](https://img.shields.io/badge/Author-Nimsara_Karunarathna-orange)

An advanced, dual-core ESP32 flight controller featuring an **Extended Kalman Filter (EKF)** for orientation sensing and **MQTT-based** telemetry and control. This version (V13.8) is optimized for stability with a specialized Heading Lock system.



## 🚀 Key Features (V13.8)
* **Navigation:** Heading Lock ENABLED (Automatic North-finding on boot).
* **Stability:** Custom EKF implementation for precise Pitch/Roll/Yaw estimation.
* **Dual-Core Execution:** * **Core 0:** High-speed sensor fusion and PID loops (250Hz).
    * **Core 1:** MQTT communication, failsafe logic, and battery monitoring.
* **Failsafe:** Automatic controlled descent if the MQTT connection is lost for >4 seconds.
* **Compensation:** Real-time voltage compensation for consistent thrust as battery drops.

## 🛠 Hardware Configuration
| Component | Pin / Address | Note |
|---|---|---|
| **MPU6050** | 0x68 (I2C) | IMU - Accel/Gyro |
| **HMC/QMC5883L** | 0x1E / 0x0D | Magnetometer (Heading Lock) |
| **BMP280** | 0x76 / 0x77 | Barometer (Altitude sensing) |
| **ESC 1** | GPIO 26 | Front Left (CW) |
| **ESC 2** | GPIO 32 | Front Right (CCW) |
| **ESC 3** | GPIO 25 | Rear Left (CCW) |
| **ESC 4** | GPIO 27 | Rear Right (CW) |
| **Battery Pin** | GPIO 34 | 3S LiPo Monitoring |

## 📡 MQTT Telemetry Stream
The drone publishes a JSON object to `esp32/simple_drone/telemetry` containing:
* `p`, `r`, `y`: Pitch, Roll, and Yaw angles.
* `a`: Current altitude (meters).
* `b`: Battery voltage (V).
* `r1-r4`: Individual ESC output speeds.

## ⚠️ Safety Warnings
1.  **Propellers OFF:** Always remove propellers when testing code or performing indoor calibrations.
2.  **Calibration:** Ensure the drone is perfectly level during the "Finding North" boot phase.
3.  **Failsafe:** The drone is set to auto-disarm after 8 seconds of failsafe descent.

## 📝 Author
**Nimsara Karunarathna** IT Undergraduate, University of Moratuwa  
*Specializing in Cybersecurity and Robotics*