# Lunar Lander Simulation Project

## Overview
This project simulates the landing of a spacecraft (inspired by the Beresheet lander) in 2D on the lunar surface using PID controllers to manage the descent. This is Assignment #0 of the course ```Introduction to New Space``` at Ariel University by Prof. Boaz Ben-Moshe.

## How It Works
The simulation uses separate PID (Proportional-Integral-Derivative) controllers to manage both vertical and horizontal speeds of the spacecraft. By continuously adjusting thrust level and spacecraft orientation, the controllers attempt to follow predefined speed profiles based on altitude to achieve a safe landing.

### Key Components Implemented:
- **Vertical Speed Control**: Manages descent rate to ensure smooth landing
- **Horizontal Speed Control**: Reduces horizontal speed to zero by landing
- **Thrust Vector Control**: Calculates optimal thrust magnitude and direction using basic physics
- **Angle Management**: Controls spacecraft orientation with rate limiting

## Physics Model
The simulation includes:
- Lunar gravity (varies with altitude)
- Thrust forces (main engine + secondary thrusters)
- Fuel consumption based on thrust level
- Basic motion equations

## Running the Simulation
To run the simulation:
```
python3 Bereshit_101.py
```

required dependencies:
- matplotlib

## Results Analysis
The simulation plots four key graphs to visualize the landing.
1. Vertical speed vs. target
2. Horizontal speed vs. target
3. Altitude and spacecraft angle
4. Throttle level (NN)

A successful landing requires:
- Final vertical speed < 2.5 m/s
- Final horizontal speed < 2.5 m/s
- Final angle between 5 degrees and -5 degrees

# Best Landing
![image](https://github.com/user-attachments/assets/be19c59b-c015-4ac4-817f-2f650f54eba1)
Final vertical speed: 1.36 m/s (+ve is down)
Final horizontal speed: 0.32 m/s
Final angle: 3.00 degrees
Remaining fuel: 17.77 kg

We believe a better landing can be achieved with better linear interpolation and PID values.

## Limitations
This is an overly simplified model and has several limitations:
- 2D
- Assumes point-mass dynamics
- Doesn't account for rotational inertia
- No environmental factors (like dust kickup)
- Simplified thruster model

## Future Improvements
Possible future improvements could include:
- Better linear interpolation
- 3D motion simulation
- More realistic thruster configurations
- Landing gear dynamics
- Obstacle avoidance
- Landing at the desired lunar location
