# Bereshit 101 - Lunar Landing Simulation

This Python project simulates the landing of the Bereshit spacecraft on the Moon using a PID-controlled thrust system.

## Overview

- The lander uses physics-based modeling to simulate a Moon landing, including gravitational forces, engine thrust, and fuel consumption.
- A **PID controller** dynamically adjusts the engine thrust to match desired vertical speeds throughout descent.
- A **threshold-based altitude-to-vertical-speed mapping** is used to ensure a smooth and controlled landing.

## Key Components

### `Moon` Class
- Defines physical constants like radius, gravity, and orbital speed.
- Computes effective gravity based on horizontal velocity (mimicking orbital dynamics).

### `Bereshit101` Class
- Contains spacecraft parameters (mass, fuel, thrust, etc.).
- Implements PID logic to control descent.
- Dynamically adjusts engine angle to reduce horizontal velocity.
- Logs data for plotting vertical speed, horizontal speed, thrust power, and engine angle over time.

## Simulation Strategy

- The simulation begins with the lander high above the Moon's surface, descending with significant vertical and horizontal velocity.
- At each time step, the code:
  - Updates the desired vertical speed based on current altitude.
  - Computes the PID output to adjust main engine thrust.
  - Reduces horizontal velocity gradually using angle adjustments.
  - Consumes fuel and updates the spacecraft's mass and acceleration.

## PID Tuning

- The best PID values were observed after multiple simulation runs:
  - `P = 0.05`, `I = 0.05`, `D = 0.1`
- Attempts were made to replace the threshold mechanism with a **continuous (logistic-style) function** for vertical speed control.
  - While smoother, the continuous approach was **less fuel-efficient** and did not outperform the simpler threshold-based logic.

## Visualization

At the end of the simulation, the code generates plots for:
- Vertical Speed vs. Time (actual vs. desired)
- Horizontal Speed vs. Time
- Thrust Power (NN) vs. Time
- Engine Angle vs. Time

## 🛰️ Beresheet Spacecraft Crash Report on the Moon

### General Information
- **Mission:** Moon landing by the Beresheet spacecraft, led by Israel Aerospace Industries  
- **Launch Date:** 22.02.2019  
- **Landing Attempt:** 11.04.2019  

### Spacecraft Data
- **Height:** 1.5 m  
- **Base Diameter:** 2 m  
- **Launch Mass:** 585 kg (wet)  
- **Dry Mass:** 150 kg  
- **Main Engine Thrust:** 430 Newtons  
- **Propulsion System:** Single engine with xenon gas thrusters for orientation  
- **Navigation System:** IMU sensors, star trackers, and laser altimeter  

### Sequence of Events Leading to Failure

- Descent began from an altitude of 22 km with the main engine active  
- The primary IMU sensor reported a data error  
- A RESET command was sent from ground control to the IMU sensor  
- The navigation system lost acceleration data, and the flight computer rebooted  
- The main engine shut down due to the navigation system reboot  
- Attempts to restart the engine failed due to a power supply issue  
- At an altitude of 149 meters, horizontal speed was ~947 m/s, vertical speed ~134 m/s  
- **Crash occurred**  

### Failure Analysis

The main issue was the **loss of data from the IMU sensor**. The IMU (Inertial Measurement Unit) is responsible for measuring and reporting orientation, velocity, and gravitational forces using accelerometers and gyroscopes. These measurements are crucial for navigation, guidance, and stabilization.

As seen in telemetry footage, once contact was regained, the **vertical speed began increasing**, despite the spacecraft being in a braking phase. This was due to the IMU failure — the lander could no longer determine its orientation and acceleration accurately, causing it to **accelerate in the wrong direction instead of slowing down**, ultimately resulting in a collision with the Moon’s surface.

---

💡 **Lesson Learned:** Redundancy in critical sensors like the IMU is essential, especially for autonomous spacecraft navigation during landing phases.
