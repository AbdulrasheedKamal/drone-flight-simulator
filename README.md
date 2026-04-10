# drone-flight-simulator
Pure MATLAB quadrator simulation with 3D visualization, cascade PID control and waypoint navigation. No Simulink
# Quadrotor Simulation in Pure MATLAB

A complete 6-DOF quadrotor simulation written entirely in MATLAB. No Simulink, no toolboxes beyond basic MATLAB. Includes a 3D visualisation with patch objects, cascade PID control, waypoint mission planner, and real-time telemetry plots.

![Screenshot](screenshot.png)  <!-- optional: add a screenshot later -->

## Features

- Full nonlinear dynamics (rigid body, aerodynamics, motor dynamics)
- Cascade PID control: position → velocity → attitude → body rates
- Waypoint mission with configurable hold times and yaw angles
- Real-time 3D drone model (arms, rotors, body, landing gear)
- Animated rotor spin and velocity vector arrow
- Live telemetry panels: position, velocity, attitude, motor RPM
- Wind disturbance model
- Smooth reference trajectory generator (prevents derivative kicks)

## Requirements

- MATLAB R2019b or later (earlier versions may work but not tested)
- No additional toolboxes required

## How to Run

1. Clone or download the repository.
2. Open MATLAB and navigate to the folder containing `quadrotor_sim.m`.
3. Run the script:

```matlab
quadrotor_sim
