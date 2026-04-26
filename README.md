# IMSES — Intelligent Multi-Sensor Elbow Sleeve

A wearable health monitoring system that combines physics, bioengineering, and computer science to classify human behaviors and prevent injuries during strength training.

**[View the project website →](https://jiayangb.github.io/imses/)**

## Overview

IMSES is an intelligent multi-sensor elbow sleeve with 5 biosensors and a motor actuator connected to a Seeed XIAO nRF52840 Sense microcontroller. It monitors physiological and mechanical signals in real time, classifies human behaviors using a three-tier decision tree, and provides safety alerts for fatigue and low blood oxygen.

## Files

| File | Description |
|------|-------------|
| `index.html` | Project portfolio website — 8 tabbed sections (Overview, Engineering, Motivation, Science, App, Future, About, Contact) |
| `app.html` | Live monitoring dashboard — connects to the device via Web Bluetooth, real-time sensor display, daily/weekly summaries, personalized feedback. Includes demo mode for exploring without hardware. |
| `demo.html` | Standalone demo app (legacy) — mobile-first dark theme with simulated sensor cycling |
| `ElbowSleeveSystemV1.ino` | Arduino firmware for the XIAO nRF52840 Sense (820 lines C++). Reads all 5 sensors at 50 Hz, runs motor protection state machine, broadcasts 3 BLE packet characteristics. |
| `assets/logo.png` | IMSES project logo |
| `assets/structure-schematic.jpg` | AI-generated exploded view of the sleeve with sensor placement |
| `assets/structure-handdrawn.png` | Hand-drawn system wiring schematic |
| `assets/structure.png` | Updated sensor layout diagram |
| `assets/prototype-worn.jpg` | Prototype photo — sleeve worn on arm |
| `assets/prototype-electronics.jpg` | Prototype photo — inside view showing electronics and wiring |
| `assets/prototype-outer.jpg` | Prototype photo — outer construction and wire routing |
| `assets/ready-player-one-suit.webp` | Ready Player One inspiration image |
| `assets/jiayang-photo.jpg` | Author photo |

## Sensors

| Sensor | Measurement | Hardware |
|--------|-------------|----------|
| EMG | Muscle electrical activity (mV) | EMG sensor module |
| T | Skin temperature (°C) | NTC 10kΩ thermistor |
| ω | Angular velocity (°/s) | IMU LSM6DS3 |
| SpO₂ | Blood oxygen saturation (%) | MAX30102 |
| θ | Elbow flex angle (°) | Flex sensor 2.2" |
| M | Motor actuator | DC motor / servo |

## BLE Architecture

The firmware broadcasts three compact BLE characteristics (≤20 bytes each):

| Characteristic | UUID suffix | Contents |
|----------------|-------------|----------|
| Vitals | `...10011` | Heart rate, SpO₂, temperature, MAX30102 raw, status flags |
| Motion | `...10012` | Flex raw, EMG raw, NTC raw, angle, angular velocity, acceleration, bend speed, motor state |
| IMU | `...10013` | Accel XYZ + Gyro XYZ (raw int16) |

Service UUID: `19B10010-E8F2-537E-4F6C-D104768A1214`

## Tech Stack

- **Hardware**: Seeed XIAO nRF52840 Sense, custom sensor array, motor actuator
- **Firmware**: C++ / Arduino (ArduinoBLE, SparkFun MAX3010x)
- **Dashboard**: HTML / CSS / JavaScript, Web Bluetooth API, Canvas charts
- **Website**: Static HTML, GitHub Pages

## Author

**Jiayang Bai** — High school researcher

Physics · Engineering · Robotics · AI
