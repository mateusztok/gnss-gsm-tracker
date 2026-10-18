# GNSS/GSM Vehicle Tracker

A real-time vehicle tracking system built on the STM32 microcontroller. The project utilizes an intelligent state machine driven by an inertial measurement unit (IMU) to dynamically adjust GNSS polling and GSM transmission rates based on the vehicle's movement.

## Architecture & Hardware

The system is built on a modular C architecture, separating hardware abstraction, data parsing, and network communication:
* **MCU:** STM32L476RG (ARM Cortex-M4)
* **Connectivity:** SIM7000 / SIM800 (GSM/GPRS/GNSS module)
* **Sensors:** MPU6050 (6-axis accelerometer and gyroscope)
  
<img width="642" height="365" alt="obraz" src="https://github.com/user-attachments/assets/f0b52d98-adec-41b4-b2ec-54f99d13bf74" />

## System State Machine

The core logic relies on a motion-activated state machine to optimize network traffic and hardware usage:
* **MODE_DRIVING:** Activated immediately upon detecting vibration or movement via the MPU6050. GNSS polling occurs every 5 seconds, and data is transmitted to the cloud every 15 seconds for a smooth map trail.
* **MODE_PARKING:** Entered after 2 minutes of complete inactivity. The transmission rate drops to 60 seconds to maintain a live connection without redundant data streaming.
* **MODE_SLEEP:** Entered after 5 minutes of deep inactivity. Active polling is paused, and the STM32 waits for external interrupts from the accelerometer to instantly resume tracking upon new movement.

## Cloud Integration

The tracker parses raw NMEA sentences to extract latitude, longitude, and speed parameters. The data is streamed via TCP/IP (GPRS) to a **ThingSpeak** REST API. The cloud dashboard provides real-time route visualization and integrates with map services for precise address resolution.

## Build

The project is structured around the STM32 HAL library and can be compiled using standard ARM GNU tools:

```bash
mkdir build && cd build
cmake -DCMAKE_TOOLCHAIN_FILE=arm-none-eabi.cmake ..
make
