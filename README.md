'from pathlib import Path; Path("README.md").write_text("""# UAV Kamikaze — SITL & HIL

Autonomous UAV mission control and flight testing framework developed for fixed-wing unmanned aerial vehicle systems.

This repository contains Software-in-the-Loop (SITL) simulation studies and Hardware-in-the-Loop (HIL) flight testing software developed using ArduPilot, MAVLink and Python.

## Overview

The project focuses on autonomous fixed-wing UAV mission execution, waypoint navigation, flight-mode management, guided flight operations and vision-assisted mission scenarios.

The development workflow combines simulation-based validation with hardware-connected flight testing.

## System Architecture

### SITL — Software-in-the-Loop

The `SITL/` directory contains simulation-oriented mission software.

- Autonomous waypoint navigation
- Route tracking
- Mission state management
- Guided mode transitions
- Gazebo-based simulation
- QR-based mission scenarios
- Autonomous flight sequence testing

### HIL — Hardware-in-the-Loop

The `HIL/` directory contains software used with an NVIDIA Jetson platform and a real ArduPilot-based flight controller.

- Jetson-to-flight-controller communication
- MAVLink-based vehicle control
- Autonomous mission execution
- Waypoint navigation
- Flight-mode management
- Real hardware flight testing
- Mission-state monitoring

## Mission Flow

1. Mission initialization
2. Autonomous takeoff
3. Waypoint-based route tracking
4. Mission state monitoring
5. Guided flight transition
6. Vision-based mission processing
7. Autonomous maneuver execution
8. Mission completion and recovery

## Repository Structure

```text
uav-kamikaze-sitl/
├── HIL/
│   ├── g1.py
│   ├── g2.py
│   ├── g3.py
│   └── g4.py
├── SITL/
│   ├── qr4.py
│   ├── qr5.py
│   └── qr6.py
└── README.md
