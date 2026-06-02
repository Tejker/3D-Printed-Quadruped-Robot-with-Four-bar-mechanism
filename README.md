# 3D-Printed Quadruped Robot using a 4-Bar Linkage Mechanism

## Overview

This project presents the design, development, and structural validation of a 3D-printed quadruped robot employing a 4-bar parallelogram linkage mechanism for leg actuation. The robot was developed as part of the Design of Machine Elements course at IIT Indore.

The primary objective was to design a mechanically robust quadruped platform capable of stable locomotion while minimizing actuator loading and moving mass. A 4-bar linkage mechanism was used to transmit motion efficiently from torso-mounted servo motors to the leg assembly, reducing leg inertia and improving overall system stability.

The project involved mechanical design, actuator integration, locomotion control, kinematic analysis, force calculations, and finite element validation of critical load-bearing components.

---

## Key Features

- 4-bar linkage based leg mechanism
- Servo-actuated quadruped locomotion
- Crawl gait implementation
- Diagonal-pair gait implementation
- 3D printed PLA structural components
- Kinematic and force analysis
- Stress concentration analysis
- Fatigue life estimation
- ANSYS-based Finite Element Analysis (FEA)

---

## System Architecture

### Mechanical Design

The robot employs a four-bar parallelogram linkage consisting of:

- Ground Link (Robot Chassis)
- Input Link (Servo Driven Thigh Link)
- Coupler Link
- Output Link (Lower Leg)

This configuration offers:

- Reduced moving mass
- Improved mass distribution
- Lower joint torque requirements
- Efficient motion transmission
- Simplified mechanical architecture

---

### Hardware Components

- Arduino Uno
- Arduino Sensor Shield V5
- Servo Motors (Hip and Knee Actuation)
- 9V Power Supply
- 3D Printed PLA Components

The robot uses eight servo motors:

- 4 Hip Servos
- 4 Knee Servos

for generating coordinated quadruped locomotion.

---

## Control Strategy

Two locomotion strategies were implemented:

### Crawl Gait

A statically stable walking pattern where only one leg moves at a time while the remaining three legs maintain ground contact.

Advantages:

- High stability
- Reduced risk of tipping
- Simpler control implementation

### Diagonal Pair Gait

Diagonal leg pairs move simultaneously:

- Left Front + Right Back
- Right Front + Left Back

Advantages:

- Faster locomotion
- Improved gait efficiency
- Better coordination between legs

---

## Engineering Analysis

The quadruped leg assembly was analyzed through:

### Kinematic Analysis

- Motion transmission through the 4-bar mechanism
- Force path determination
- Joint reaction calculations

### Structural Analysis

- Bending stress calculations
- Shear stress calculations
- Stress concentration analysis
- Buckling evaluation

### Fatigue Analysis

- Modified Goodman criterion
- Endurance limit correction
- Fatigue life estimation

### Finite Element Analysis (FEA)

ANSYS simulations were performed to validate:

- Stress distributions
- Deformation characteristics
- Structural integrity under loading

Results confirmed that all critical components operate well within safe limits.

---

## Applications

Potential applications of the developed platform include:

- Autonomous inspection robots
- Search and rescue systems
- Hazardous environment exploration
- Environmental monitoring
- Precision agriculture
- Planetary exploration

---

## Future Improvements

Planned future developments include:

- Vision-based locomotion feedback
- Machine learning assisted gait optimization
- Improved coupler design for higher stiffness
- Enhanced terrain adaptability
- Closed-loop control using sensor feedback

---

## Project Report

The complete project report contains:

- Design methodology
- CAD models
- Kinematic derivations
- Force analysis
- Stress calculations
- Fatigue analysis
- ANSYS FEA results
- Design recommendations
