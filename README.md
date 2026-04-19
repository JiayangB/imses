# IMSES — Intelligent Multi-Sensor Elbow Sleeve

A wearable health monitoring system that combines physics, bioengineering, and computer science to classify human behaviors and prevent injuries during strength training.

**[View the project website →](https://imses-jiyang.netlify.app/)**

## Overview

IMSES is an intelligent multi-sensor elbow sleeve with 5 biosensors and a motor actuator connected to a Seeed XIAO nRF52840 Sense microcontroller. It monitors physiological and mechanical signals in real time, classifies human behaviors using a three-tier decision tree, and provides safety alerts for fatigue and low blood oxygen.

## What It Measures

| Sensor | Measurement | Hardware |
|--------|-------------|----------|
| EMG | Muscle electrical activity (mV) | EMG sensor module |
| T | Skin temperature (°C) | NTC 10kΩ thermistor |
| ω | Angular velocity (°/s) | IMU LSM6DS3 |
| SpO₂ | Blood oxygen saturation (%) | MAX30102 |
| θ | Elbow flex angle (°) | Flex sensor 2.2" |
| M | Motor actuator | DC motor / servo |

## Behaviors Detected

- 😴 Sleeping
- 🪑 Resting
- 💻 Office / Typing
- 🍜 Eating
- 🚶 Walking
- 🏃 Aerobic Exercise
- 🏋️ Strength Training
- ⚠️ Fatigue Warning

## Safety Alerts

- **Low oxygen**: SpO₂ drops below 93%
- **Fatigue detection**: Angular velocity drops from >100°/s to <20°/s within 3 seconds

## Tech Stack

- **Hardware**: Seeed XIAO nRF52840 Sense, custom sensor array, motor actuator
- **Firmware**: C++ / Arduino
- **App**: React + TypeScript, Web Bluetooth API, Web Serial API
- **Website**: [imses-jiyang.netlify.app](https://imses-jiyang.netlify.app/)

## Author

**Jiayang** — High school researcher

Physics · Engineering · Robotics · AI
