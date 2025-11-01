# Range Balancing Bot

An autonomous vehicle system that maintains a fixed distance from walls while operating a conveyor mechanism.

## Project Information

- **Course**: PDE4446
- **Authors**:
  - Aman Mishra (M00983641)
  - Arman Shaikh (M01096196)

## Problem Statement

How can an autonomous vehicle accurately maintain a fixed distance from a wall or boundary while its conveyor mechanism is manipulating objects and experiencing manual user interactions?

## Solution Overview

This project implements a closed-loop feedback system using an Arduino microcontroller to precisely regulate the vehicle's lateral position. The system employs a PID (Proportional-Integral-Derivative) controller for stable performance and accurate distance maintenance.

## Components

### Hardware

- **Arduino Board**: Main microcontroller executing the PID algorithm
- **HC-SR04 Ultrasonic Sensor**: Measures wall distance for PID input
- **DC Motors and Wheels**: Provides propulsion and directional control
- **L293D Motor Driver**: Controls motor direction and speed via PWM
- **NEMA Stepper Motor**: Drives the conveyor belt
  - Powered by separate power supply
  - Controlled via A4988 stepper driver
- **Capacitor**: Provides steady current protection
- **Potentiometer**: Manual conveyor belt speed control

### Key Features

- Real-time distance measurement and correction
- User-adjustable conveyor belt speed
- Safety protocols including:
  - Output clamping on PID controller
  - Maximum speed limits
  - Emergency stop on large-magnitude errors

This usually involves an iterative tuning process:Start with Ki and Kd at zero. Increase or decrease Kp until the robot starts to oscillate around the Setpoint.Reduce or increase Kp to about half the value that caused oscillations. Slowly increase or decrease Ki to eliminate any small, persistent steady-state error. Slowly increase or decrease Kd to reduce overshoot and dampen oscillations caused by Kp.

The core solution is a PID controller that dynamically modulates the vehicle's velocity to maintain a fixed Setpoint distance from a wall. An ultrasonic sensor continuously provides the real-time distance measurement, which serves as the Process Variable (PV).

The PID algorithm computes the error signal (e=SP−PV) and generates a corresponding Manipulated Variable by applying proportional (Kp ), integral (Ki), and derivative (Kd) gains. This MV is translated into a Pulse Width Modulation (PWM) signal, which is fed to an L298N H-bridge motor driver.

The driver, in turn, powers the DC propulsion motor, thus forming a system that actively corrects for positional drift. This control loop operates concurrently with a secondary system: a user-adjustable conveyor belt driven by a NEMA stepper motor, whose speed is governed by analog input from a potentiometer.

Critical safety protocols has to be embedded in the software, including output clamping on the PID controller to a maximum speed limit and a fault-detection mechanism that triggers an emergency stop upon sensing, large-magnitude error values.

## Implementation Process

### Phase 1: Robot Chassis Assembly

1. **Base Construction**

   - Build chassis framework
   - Mount DC motors and attach wheels
   - Install core electronics (Arduino, L298N, buck converter, battery)
   - Mount ultrasonic sensor facing tracking wall

2. **Wiring**

   - Connect battery to L298N (VCC and GND)
   - Wire L293D outputs to DC motors
   - Connect control pins to Arduino
   - Configure PWM pins for speed control

3. **Initial Testing**
   - Upload basic movement test sketch
   - Verify forward/backward motion
   - Test motor direction and connections

### Phase 2: Conveyor Integration

1. **Mechanical Assembly**
   - Mount NEMA stepper motor to frame
   - Couple motor shaft to drive roller
   - Install and tension conveyor belt
   - Mount A4988 driver and potentiometer

2. **Electronic Setup**
   - Connect A4988 STEP/DIR pins to Arduino digital pins
   - Wire potentiometer to Arduino analog input
   - Connect stepper motor to A4988 outputs
   - **Important**: Wire 12V power supply to A4988 Vcc/GND

3. **Testing**
   - Write test sketch for basic functionality
   - Implement potentiometer reading (0-1023 range)
   - Map values to stepper motor delay times
   - Verify speed control via potentiometer

### Phase 3: PID Implementation

1. **Control Loop Setup**
   ```cpp
   // Constants
   const float SETPOINT_CM = 12.0;
   
   // PID Gains
   double Kp = 1.0;   // Proportional
   double Ki = 0.15;  // Integral
   double Kd = 0.04;  // Derivative
   ```

2. **PID Algorithm Implementation**
   ```cpp
   // Time Management
   unsigned long currentTime = millis();
   float dt = (currentTime - lastTime) / 1000.0;
   lastTime = currentTime;
   
   // Distance Measurement
   float distance = readUltrasonic();
   
   // Error Calculations
   double error = SETPOINT_CM - distance;
   double p_term = Kp * error;
   
   integral_sum += error * dt;
   double i_term = Ki * integral_sum;
   
   double derivative = (error - last_error) / dt;
   double d_term = Kd * derivative;
   last_error = error;
   
   // Final Output
   float output = p_term + i_term + d_term;
   ```

3. **Safety Implementation**
   - Output clamping: `constrain(output, -MAX_SPEED, MAX_SPEED)`
   - Direction control based on output sign
   - PWM speed application to motors

4. **System Testing**
   - Basic position maintenance
   - Conveyor operation interference testing
   - Safety system verification
     - Maximum speed limits
     - Large-error emergency stop
     - System stability checks

## Results

[Content to be added]

## Challenges and Solutions

### Vehicle Control

- **Challenge**: Free wheel causing directional instability
- **Solution**: Implemented 10% speed increase on right wheel for stability

### Conveyor System

- **Challenge 1**: Belt tension and surface friction issues

  - Initial attempt: Added paper tape layer for increased roughness
  - Final solution: [Content to be added]

- **Challenge 2**: Motor slippage due to worn 3D printed holders
  - Temporary fix: New holders with hot glue reinforcement
  - Long-term solution: [Content to be added]

## Contribution Matrix

| Task               | Aman Mishra | Arman Shaikh |
| ------------------ | ----------- | ------------ |
| [Task Description] | [%]         | [%]          |

## References

[Content to be added]
