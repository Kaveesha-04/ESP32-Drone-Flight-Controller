# 🛸 Tri-Module Drone Control System

This repository contains the firmware for a custom drone and remote control system. The project is split into three main modules: the Radio Remote, the Communication Bridge, and the Flight Controller.

---

## 📡 System Architecture

The control signal flows as follows:
**[Nano Remote]** --(NRF24L01)--> **[ESP32 Bridge]** --(UART Serial)--> **[STM32 Flight Controller]**

### 1. Nano Remote (Transmitter)
* **Location:** `/nano_code`
* **Role:** Reads joystick analog values and transmits them wirelessly.
* **Hardware:** Arduino Nano + NRF24L01 + Joysticks.

### 2. ESP32 Bridge (Receiver & Forwarder)
* **Location:** `/esp_code`
* **Role:** Receives NRF24L01 radio packets and bridges the data to the STM32 via Serial. It also handles Wi-Fi telemetry.
* **Hardware:** ESP32 WROOM + NRF24L01.

### 3. STM32 Flight Controller (Core)
* **Location:** `/stm_code`
* **Role:** The "brain" of the drone. Receives control data from the ESP32 bridge, processes PID loops, and controls the motors.
* **Hardware:** Custom PCB with STM32 chip.

---

## 📂 Repository Structure

```text
├── nano_code/    # Firmware for the Transmitter (Arduino Nano)
├── esp_code/     # Firmware for the Drone-side Bridge (ESP32)
└── stm_code/     # Main Flight Controller logic (STM32)
