# PID Beam Balancing System (TM4C123 + MPU6050 + Python GUI)

## 📌 Overview

This project implements a **real-time PID-based beam balancing system** with **live monitoring and visualization**.

The system stabilizes a beam using BLDC motors based on IMU feedback, while a Python-based GUI provides **real-time visualization of angle, RPM, and current**.

---

## 🧩 System Architecture

* **Embedded Layer (TM4C123)**

  * Reads MPU6050 data
  * Computes PID control
  * Generates PWM for ESCs
  * Sends telemetry via UART

* **Monitoring Layer (Python GUI)**

  * Reads serial data
  * Displays artificial horizon
  * Plots real-time graphs
  * Shows system parameters

---

## ⚙️ Hardware Used

* TM4C123G (Tiva C Series)
* MPU6050 (IMU)
* 2 × BLDC Motors with ESCs
* ACS712 Current Sensors
* Beam balancing setup

---

## 💻 Software Components

### 🔹 Embedded Firmware (C)

* PID control loop (~200 Hz)
* Complementary filter for angle estimation
* PWM generation for ESC control
* UART telemetry output
* Current monitoring

---

### 🔹 Python GUI (Real-Time Monitoring)

File: `Grphical_Data.py`

#### Features:

* Serial communication with microcontroller
* Artificial horizon (angle visualization)
* Live plotting of:

  * Roll angle
  * Error
  * Motor RPM
  * Current consumption
* Auto-updating graphs
* Status monitoring dashboard

---

## 📊 GUI Preview Features

* Horizon display (like aircraft attitude indicator)
* Real-time graphs using matplotlib
* Live numeric values (PWM, RPM, Current)

---

## 🔄 Data Flow

1. MPU6050 → Angle measurement
2. PID Controller → Error correction
3. PWM → Motor control
4. UART → Send data to PC
5. Python GUI → Visualization

---

## 📐 PID Parameters

```
KP = 0.38
KI = 0.028
KD = 0.015
```

---

## 🔌 Serial Configuration

* Baud Rate: 9600
* Timeout: 1 sec
* Format: Custom structured data from TM4C123

---

## 🚀 How to Run

### 🔹 Embedded System

1. Open project in Keil / CCS
2. Build and flash to TM4C123

---

### 🔹 Python GUI

#### Step 1: Install Python (if not installed)

https://www.python.org/

#### Step 2: Install dependencies

```
pip install pyserial matplotlib
```

#### Step 3: Run GUI

```
python Grphical_Data.py
```

---

### 🔹 Connect to Device

1. Click **Refresh**
2. Select COM port
3. Click **Connect**
4. Start monitoring live data

---

## 📊 Output Parameters

* Angle (Pitch/Roll)
* Error
* PWM Signal
* Motor RPM
* Current (A)

---

## ⚡ Safety Features

* Current limiting using ACS712
* Fault detection
* ESC protection logic

---

## 🎯 Applications

* Control systems labs
* Self-balancing systems
* Drone stabilization concepts
* Embedded + GUI integration projects

---

## 📌 Future Improvements

* Kalman Filter implementation
* Wireless telemetry (ESP32/Bluetooth)
* Auto PID tuning
* Data logging to CSV
* 3D visualization

---

## 🎥 Demo (Recommended)

Add your video link here:

```
https://your-demo-link
```

---

## 👤 Author

Fida Hussain
Mechatronics Engineer

---

## ⭐ If you like this project

Give it a star on GitHub!
