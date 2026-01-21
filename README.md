# CarGo – Autonomous Cargo Delivery Robot

CarGo is an autonomous mobile robot designed to perform cargo pickup and delivery
tasks using a combination of remote input, onboard sensing, and autonomous
navigation.   
Access the demo here: https://drive.google.com/file/d/1wd14m5_EEpL6o4mIZ9spkhfhqQk9Ou74/view?usp=sharing

<img width="282" height="186" alt="Screenshot 2026-01-21 at 3 50 59 PM" src="https://github.com/user-attachments/assets/5b6bdfd0-4b33-42aa-a96e-d02f41cd404e" />

## Overview
This project demonstrates an autonomous cargo delivery system in which a mobile
robot navigates a predefined environment, identifies pickup and drop-off locations,
and executes cargo handling tasks without manual intervention. The system relies
entirely on onboard perception and control, integrating navigation, sensing, and
actuation into a cohesive robotic workflow.

## System Architecture
The CarGo robot is built around an Arduino-based control system that integrates
multiple sensors and actuators to enable autonomous operation. Navigation is
performed using line-following, while onboard perception is used to identify
locations and guide task execution.

The robot follows predefined line paths for navigation and uses onboard sensing
to identify stations and verify task locations. Barcode/QR scanning and color
detection are used to distinguish pickup and drop-off points during operation.

As the robot approaches each potential drop-off location, it repeats barcode/QR
and color verification to ensure alignment with the specified task criteria before
executing cargo placement.

## Operation Workflow
1. The robot navigates autonomously using line-following.
2. Onboard sensors scan barcode/QR markers and color information at stations.
3. Pickup locations are identified and verified using onboard perception.
4. The robot performs cargo pickup using its mechanical handling mechanism.
5. The robot navigates to candidate drop-off locations.
6. Barcode/QR and color verification are repeated to confirm the correct drop-off
   point.
7. Cargo is released once the location is validated.
## Mechanical Design and Circuit Diagram
The robot features a custom-designed chassis that separates power, drive, and
control components to support stable navigation and payload handling. The design
was optimized to accommodate the electronics while maintaining compactness and balance.
A custom forklift mechanism is integrated at the front of the robot to enable
cargo pickup and placement. The mechanism uses a servo-driven rack-and-pinion
style lift to provide controlled vertical motion.
The circuit diagram illustrates the integration of sensors, actuators, and
control electronics used in the CarGo robot.

<img width="371" height="440" alt="Screenshot 2026-01-21 at 3 48 51 PM" src="https://github.com/user-attachments/assets/f246eb07-f571-4dfc-af54-18d8d5566dbf" />   <img width="427" height="197" alt="Screenshot 2026-01-21 at 3 51 39 PM" src="https://github.com/user-attachments/assets/40bded5a-7a51-4a1f-9c40-f3984aaa97ba" />

## Code Structure
- `all_integrated/` contains the integrated Arduino Mega code responsible for
  navigation, sensing, decision-making, and actuation.
- `Remote Code/` contains experimental code that is not required for autonomous
  operation.
