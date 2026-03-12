# 🛸 Tri-Module Drone System (Custom STM32 & ESP32 Bridge)

This repository contains the firmware and documentation for a custom-built drone system. The architecture features a dedicated radio remote, a high-speed communication bridge on the drone, and a custom STM32-based flight controller.

---

## 📡 System Architecture

The control signal follows this path:
**[Nano Remote]** --(NRF24L01)--> **[ESP32 Bridge (Drone)]** --(Serial/UART)--> **[STM32 Flight Controller]**

### 1. Remote Control (Transmitter)
* **Hardware:** Arduino Nano + NRF24L01 + Joysticks.
* **Role:** Reads analog joystick values (Pitch, Roll, Yaw, Throttle), packages them into a struct, and transmits them via 2.4GHz radio.

### 2. Communication Bridge (Receiver)
* **Hardware:** ESP32 WROOM + NRF24L01.
* **Role:** Receives the NRF24L01 packets from the Nano. It acts as a "bridge," forwarding the control data to the STM32 via Hardware Serial while simultaneously providing a Web Dashboard for telemetry.

### 3. Flight Controller (Core)
* **Hardware:** Custom PCB with STM32 Chip.
* **Role:** Receives bridged control data, processes PID loops, merges sensor data (IMU), and generates PWM signals for the ESCs/Motors.

---

## 📂 Repository Structure

```text
├── nano-remote/              # Transmitter firmware (Arduino/C++)
├── esp32-bridge/             # Receiver & Telemetry Bridge (C++/ESP-IDF or Arduino)
│   └── web-dashboard/        # HTML/CSS/JS for the Wi-Fi interface
├── stm32-fc/                 # Core Flight Controller logic (C++)
├── hardware/                 # Schematics, PCB layouts, and pinouts
└── shared/                   # Common header files (e.g., protocol.h)
