# range-balancing-bot
PROJECT TITLE:  RANGE_BALANCING_BOT
AUTHORS: AMAN MISHRA  M
         ARMAN SHAIKH M01096196

PROBLEM STATEMENT:
How can an autonomous vehicle accurately maintain a fixed distance from a wall or boundary while its conveyor mechanism is potentially manipulating objects and users are manually influencing its operation.

SOLUTION:
This project implements a closed-loop feedback system designed around an Arduino microcontroller, to precisely regulate the vehicle's lateral position. The correct values for Kp, Ki and Kd are crucial for stable performance.

LIST OF COMPONENTS:
ARDUINO BOARD is the main microcontroller which executes the PID algorithm and modulates speed and maintains range.

ULTRASONIC SENSOR  HC-SR04 measures the distance to the wall and provides the  INPUT or Desired value  for the PID.

DC MOTOR and WHEELS propels the vehicle robot and provides forward and backward motion

L293D MOTOR DRIVER- interfaces the Arduino's low-power signal  Used for controlling the direction and speed (via PWM) of the DC motor.

NEMA STEPPER MOTOR drives the conveyor belt, It is powered by a separate Power Supply and controlled by the Arduino via a Stepper Motor Driver like the A4988.
Capacitor to send a steady current and prevent from fusing out.

POTENTIOMETER- allows the user to manually set the conveyor belt speed it reads an analog value which the Arduino converts to a speed setting for the stepper motor

This usually involves an iterative tuning process:Start with Ki and Kd at zero. Increase or decrease Kp until the robot starts to oscillate around the Setpoint.Reduce or increase Kp to about half the value that caused oscillations. Slowly increase or decrease Ki to eliminate any small, persistent steady-state error. Slowly increase or decrease Kd to reduce overshoot and dampen oscillations caused by Kp.

The core solution is a PID controller that dynamically modulates the vehicle's velocity to maintain a fixed Setpoint distance from a wall. An ultrasonic sensor continuously provides the real-time distance measurement, which serves as the Process Variable (PV). 

The PID algorithm computes the error signal (e=SP−PV) and generates a corresponding Manipulated Variable  by applying proportional (Kp ), integral (Ki), and derivative (Kd) gains. This MV is translated into a Pulse Width Modulation (PWM) signal, which is fed to an L298N H-bridge motor driver.

 The driver, in turn, powers the DC propulsion motor, thus forming a system that actively corrects for positional drift. This control loop operates concurrently with a secondary system: a user-adjustable conveyor belt driven by a NEMA stepper motor, whose speed is governed by analog input from a potentiometer. 

Critical safety protocols has to be embedded in the software, including output clamping on the PID controller to a maximum speed limit and a fault-detection mechanism that triggers an emergency stop upon sensing, large-magnitude error values.  

PROCESS:
For better understanding divide the process in Phases
PHASE 1: ROBOT CHASIS ASSEMBLY
Build the base robot chassis. Mount the DC motors to the frame and attach the wheels.

MOUNT CORE ELECTRONICS: Securely mount the Arduino, L298N Motor Driver, Buck converter and 12V battery pack to the chassis.

MOUNT SENSOR: Mount the HC-SR04 Ultrasonic Sensor on the side of the chassis. It must face the wall it will be tracking.
Wire the Propulsion System:

Connect the battery to the L298N's power input (VCC and GND).
Connect the L293D's  output to the power connect the L298N motor outputs to the DC motors.

Connect the L293D Input(control) pins (e.g., IN, IN, EN for Both motors) to the Arduino's digital (PWM) pins.
EN will be used for speed control via analogWrite().

TEST PROPULSION: Write a simple test sketch. Upload a program that makes the robot move forward, backward, and stop, just to verify all connections and motor directions are correct.

PHASE 2: Conveyor Belt Integration

MOUNT THE STEPPER MOTOR: Mount the NEMA stepper motor to the frame and couple its shaft to the drive roller. This roller will move the belt. Make sure the tension is proper between rollers. Mount the stepper motor driver (A4988) and the potentiometer.

Connect the A4988's STEP and DIR pins to two digital pins on the Arduino. Connect the potentiometer's output to an Analog pin on the Arduino. Connect the stepper motor's four wires to the A4988's outputs.

IMPORTANT: Connect the separate power supply 12V to the A4988's motor power input Vcc and GND. 

TEST CONVEYOR: Write a test sketch. Read the potentiometer's value and map() the function to convert the 0-1023 value into a delay time for the stepper motor. Verify that you can control the conveyor speed by turning the knob.

PHASE 3: PID Implementation and Tuning

ESTABLISH THE CONTROL LOOP:
Define constants: float SETPOINT_CM = 12.
Define gains:
double Kp = 1.0;
double Ki = 0.15;
double Kd = 0.04;.

Create the PID logic:
distance = readUltrasonic();
unsigned long currentTime = millis(); float dt = (now - lastTime) / 1000.0; lastTime = now;

CALCULATE ERROR: double error = SETPOINT_CM - distance;
CALCULATE PROPORTIONAL TERM: double p_term = Kp * error;
CALCULATE INTEGRAL ERROR: integral_sum += error * dt; float i_term = Ki * integral_sum;
CALCULATE DERIVATIVE TERM: double derivative = (error - last_error) / dt; double d_term = Kd * derivative; last_error = error;

CALCULATE OUTPUT: float output = p_term + i_term + d_term;

SET SAFETY LIMITS:
constrain(output, MAX_SPEED, MAX_SPEED);
Apply to motors: Use the pwm_speed to drive the motors. If pwm_speed is positive, move forward if negative, move backward.
Keep Tuning the Bot based on the conveyor belt surface and the length of conveyor.

Final Test:
Run the robot to maintain its position from the wall.
Test the final system turn on the conveyor belt and the PID loop should now be active enough to correct for these disturbances and maintain its position.

Test your safety systems (max speed and large-error-stop).

RESULT:


REFLECTION: CAR BOT
The journey commenced with building the vehicle assembling its chassis and electronics and wiring which did not create any issue untill the free wheel of the vehicle made the vehicle turn in any direction which made PID tuning more complicated.
we tried to constrain the left and right motion by using long screws but as guided by profesor it is not an appropriate way so we undo the long screws and increase the right wheel speed by 10% which made it work.
  CONVEYOR:
  The connection and the coding part for the conveyor belt was no issue at all and was done in one attempt. The Mechanical Part did not went well where we faced many issues. Issues like the driver roller had an tension issue with the belt which we tried to solve by adding a layer of paper tape and making it rough but after some time that solution did not work.
  Another issue was the slipping of stepper motor from the roller which was because of the old and used 3D printed holders. After printing new holders and applyng hot glue the motor worked for sometime and eventually brokedown. Even the Belt surface had more friction.
  Due to these issue the PID tuning was much difficult.

  CONTRIBUTION MATRIX:

  REFERENCES: