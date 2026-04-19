# IMSES — Intelligent Multi-Sensor Elbow Sleeve

A wearable health monitoring system that combines physics, bioengineering, and computer science to classify human behaviors and prevent injuries during strength training.

**[View the project website](https://jiayangb.github.io/imses/)**

## Overview

IMSES is a soft robotic elbow sleeve embedded with 7 biosensors connected to a Seeed XIAO nRF52840 Sense microcontroller. It monitors physiological and mechanical signals in real time, classifies 8 human behaviors using a three-tier decision tree, and provides safety alerts for fatigue and low blood oxygen.

## What It Measures

| Sensor | Measurement | Hardware |
|--------|-------------|----------|
| T | Skin temperature (°C) | NTC 10kΩ thermistor |
| θ | Elbow flex angle (°) | Flex Sensor 2.2" |
| ω | Angular velocity (°/s) | IMU LSM6DS3 |
| SpO₂ | Blood oxygen (%) | MAX30102 |
| HR | Heart rate (bpm) | MAX30102 |
| G | Skin conductance (µS) | Silver electrodes |
| F | Muscle force (N) | FSR 402 |

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

- **Hardware**: Seeed XIAO nRF52840 Sense, custom sensor array
- **Firmware**: C++ / Arduino
- **App**: React + TypeScript, Web Bluetooth API, Web Serial API
- **Deployment**: GitHub Pages (static, no backend)

## Author

**Jiayang** — High school researcher

Physics · Bioengineering · Computer Science
