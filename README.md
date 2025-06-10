##  Autonomous Robot with RTOS

This project demonstrates how to build a simple autonomous robot using an **ESP32** and **FreeRTOS** in the Arduino environment. It uses an ultrasonic sensor for distance measurement and dual motors for movement. To explore the power of **FreeRTOS** by implementing multiple real-time tasks such as motor control, obstacle detection, and system monitoring.

---

##  Features

-  Multi-tasking using FreeRTOS on ESP32
-  Motor speed control using PWM and directional pins
-  Obstacle detection with an ultrasonic sensor
-  Shared variables protected by semaphores
-  Debug task prints real-time values to serial monitor

---

##  Requirements

###  Hardware

- ESP32 Dev Module
- Ultrasonic sensor (HC-SR04 or similar)
- L298N motor driver (or dual H-bridge)
- Two DC motors
- Power supply

###  Software & Libraries

Install these libraries via Arduino Library Manager or PlatformIO:

- `Arduino.h` (built-in for ESP32 Arduino core)
- `FreeRTOS.h`, `task.h`, `semphr.h` (included in ESP32 Arduino core)

###  Installation Instructions

#  If using **Arduino IDE**:
1. Install **ESP32 board support** from:
   - `File > Preferences > Additional Boards Manager URLs`  
     Add: `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
2. Go to **Boards Manager** and install **ESP32 by Espressif Systems**
3. Open the `.ino` or `main.cpp` file
4. Select your ESP32 board
5. Connect and upload

# If using **PlatformIO (VS Code)**:

1. Create a new project
2. Set the platform as `Espressif 32`
3. Copy the `main.cpp` into `src/`
4. Add this to `platformio.ini`:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
