# GFLA
# Galvo Fiber Laser Autofocus Control System (V8)

An open-source, ESP32-based autofocus and Z-axis control system for Galvo fiber lasers. 

This system makes it easy to motorize the Z-axis of your fiber laser. It features a built-in setup wizard, the ability to save focal lengths for multiple lenses, material height adjustment, Z-offsets for steel colors, and a dedicated automatic 3D engraving function controlled via a foot pedal and a relay.

## 🚀 Features
* **Screen & Menu:** 128x64 OLED display (SSD1309) with a full menu navigated via an EC11 rotary encoder.
* **Lens Memory:** Save and quickly switch between different lenses (e.g., 70x70, 110x110, 150x150). The system automatically calculates the correct focus height.
* **Material Height:** Input the thickness of your workpiece, and the Z-axis adjusts accordingly.
* **Steel Color Offsets:** Save up to 4 unique Z-offsets to achieve perfect tempering colors on steel. Includes a quick "Off" function to disable offsets.
* **3D Auto Engraving:** Set the depth, minutes, and seconds. The system gradually lowers the Z-axis automatically during engraving, activated and safely monitored via a foot pedal.
* **Quick-Jog Buttons:** 4 physical push-buttons for manual jog control (+10mm, -10mm, +1mm, -1mm).
* **Auto-Homing:** Automatically finds the top position using an endstop switch on startup.

---

## 🛠 Hardware Requirements
* **Microcontroller:** ESP32 (Designed for ESP32-WROOM-DA)
* **Display:** OLED SSD1309 (128x64) I2C
* **Motor Control:** Stepper motor driver (e.g., TMC2160) + Stepper Motor
* **Navigation:** EC11 Rotary Encoder with push-button
* **Safety/Homing:** Endstop switch (Top limit)
* **Manual Control:** 4x Push-buttons (for Jogging)
* **3D Engraving:** Foot pedal and a 3V/5V Relay module

---

## 🔌 Pin Setup (ESP32)

| Component | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **I2C OLED Display** | | |
| SDA | `GPIO 21` | |
| SCL | `GPIO 22` | |
| **EC11 Encoder** | | |
| CLK | `GPIO 32` | INPUT_PULLUP |
| DT | `GPIO 14` | INPUT_PULLUP |
| SW (Button) | `GPIO 13` | INPUT_PULLUP |
| **Stepper Driver** | | |
| STEP | `GPIO 25` | (TMC2160 or similar) |
| DIR | `GPIO 26` | |
| ENABLE | `GPIO 27` | Active Low (`LOW` = Enabled) |
| **Foot Pedal & Relay** | | |
| Pedal | `GPIO 4` | INPUT_PULLUP |
| Relay | `GPIO 23` | Active Low |
| **Endstop** | | |
| Top Endstop | `GPIO 33` | INPUT_PULLUP |
| **Quick Buttons** | | |
| + 10 mm | `GPIO 16` | INPUT_PULLUP |
| - 10 mm | `GPIO 17` | INPUT_PULLUP |
| + 1 mm | `GPIO 18` | INPUT_PULLUP |
| - 1 mm | `GPIO 19` | INPUT_PULLUP |

---

## 💻 Libraries (Arduino IDE)
To compile the code, you need to install the following libraries via the *Library Manager* in the Arduino IDE:
1. `Wire.h` (Built-in)
2. `EEPROM.h` (Built-in to the ESP32 core)
3. `Adafruit GFX Library`
4. `DIYables_OLED_SSD1309`
5. `AccelStepper`

## ⚙️ First-Time Setup
The first time the system is powered on (or if the EEPROM is corrupted/cleared), it will automatically boot into the **Setup Wizard**. Follow the on-screen instructions on the OLED display to perform "Homing" and calibrate the machine's maximum physical height, as well as the focal length for your primary lens. All configurations are permanently saved in the ESP32's EEPROM.
