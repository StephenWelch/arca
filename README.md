## Members
Stephen Welch, Computer Engineering Student (2024)
stephenwelch@vt.edu

## Mentor
MENTOR NAME HERE

## Current Status
IN PROGRESS

### 7-24-24
- Prototype of full assembly design
- Work-in-progress hardware-sim abstraction layer
- Assembly & validation of updated left leg in progress
- Full model exported to URDF & MJCF
- Solved design issues:
  - Knee actuator didn't have enough clearance with sideplate to freely rotate
  - Knee joint was prone to friction with thigh sideplates
  - Thigh joint was prone to friction with lower thigh carriage
  - Hip motor mounting didn't have enough clearance to assemble without flex
  - Foot had no way to add rubber/other material for more friction
  - Width of knee joint made fastening bearings hard
- Current design issues:
  - Most fasteners are M3/M4 nuts & bolts - unknown how this will stand up to vibration
  - Rod end bearing range-of-motion is biggest limiter of robot range-of-motion
  - Backlash
    - Fasteners for 3D printed linkage arms to motor shaft have a lot of play
      - Linkage arm thru-hole tends to compress
        - Currently testing clamp-style design
      - M3 through motor shaft thru-hole is not perfect fit
        - May need to switch to steel pin
      - No off-the-shelf servo arms for motor shaft size
    - Several parts are very thin & flexible in the non-vertical direction in order to reduce size and make assembly easier
    - Mounting for thigh arms is not ideal & may flex
      - Planning on addressing this by integrating into thigh sideplates once finalized
  - Clearance and wire routing
    - Planning on investigating materials to reduce friction between motor wires & PLA parts

<img src="images/status_2-2-24_1.png" width="250">
  
### 2-2-24
- Test fit of (most of) 1st iteration design
- Waiting for electronics & test stand parts
<img src="images/status_2-2-24_1.jpg" width="250">


## Project Overview

A torque-controlled bipedal robot for <$1000. The short term goal is assembling and validating a single leg under load.

## Educational Value Added

- Learning how to CAD complex moving assemblies
- Embedded controls
    - Implement torque control for robot joints
    - Serial interface to high-level controller
- High-level controls
    - ROS 2
    - Implementing [step timing adaptation-based walking controller](https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9082021)
    - Implementing [MPC walking control](https://arxiv.org/pdf/2010.08198.pdf)
    - Impedance control
    - Whole body control

## Tasks

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->

## Design Decisions

### Electronics
- Tethered operation to reduce cost
    - Battery, RPi, Power electronics to be added later
- Teensy commanded over serial by PC running high-level controller
- Robomaster M2006 P36 Brushless Motor
    - 1 Nm torque @ 1:36 reduction
    - 90g motor, 17g ESC
    - Pros:
      - Only brushless motor of its size supporting torque control out-of-the-box
      - [Proven performance](https://www.youtube.com/watch?v=_Sh4kRtmAog)
      - Constant torque at all RPMs
        ![M2006 motor curves](images/m2006_specs.png)
    - Cons:
      - High reduction -> low torque transparency -> less compliance
      - Form factor challenging to package


### Mechanics
![labeled diagram of leg](images/arca.png)
- 3 DoF per-leg
    - Pros:
        - Each actuator removed from the design saves $94
        - Reduces mechanical complexity
    - Cons:
        - Since the ankle isn't actuated, the robot is *not* passively stable - it must actively step to maintain balance
            - This makes control more challenging
        - Since hip yaw isn't actuated, the robot yaw can't be controlled directly while walking
- Actuator arrangement
    - Based on Tello Leg: [Video 1](https://www.youtube.com/watch?v=62lo04Up2vc) [Video 2](https://www.youtube.com/watch?v=mn8tCtYHzHI&t=1s) [Paper](https://arxiv.org/abs/2203.00644)
        - Challenging practical limitation to this design is rotor inertia of QDD actuators, especially at the knee/ankle differential. The P36 is not QDD and is much smaller scale, removing this limitation
    - Minimizes "reflected inertia" - the moment of inertia of each link the actuator has to move
    - Keeps knee actuator in the leg frame, which reduces torque tracking requirements
    - Combined hip/roll axes
        - Rarely actuated at the same time, effectively doubles available torque to 2 Nm
- Avoid gear/belt transmissions where possible
    - Large losses to friction, etc. esp. at this scale
    - Hard to get right, esp. with 3D prints
- Available torque
    - 1 N*m * 0.2248089431 lbf/N \* 100 cm/m = 22.48 lbf\*cm
    - Actuator linkages designed to be 1:1
    - Assuming 90 deg knee bend:
        - Knee can lift 22.48 lbf\*cm / 8 cm thigh length = 2.81 lbf
        - Thigh can lift 22.48 lbf\*cm * 2 motors / 8 cm thigh length = 5.62 lbf
        - Total weight of components is 107 g per actuator * 0.00220462 lbs/g * 3 actuators = 0.707685 lbs

## Design Misc

The main concern is that torque output won't be adequate for dynamic motions (the static case is addressed briefly above).
Possible fixes for this would include:
- Redesigning for larger, higher-torque actuators
- Designing a custom reduction for the M2006 motor
- Adding an effective reduction in the actuator linkages

## Steps for Documenting Your Design Process

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->

## BOM + Component Cost
[BOM Link](https://docs.google.com/spreadsheets/d/1Qe1dRF8I_yPayQ6RHrJmCSCA597qFWhmrA_jtBt1tgA/edit?usp=sharing)

## Timeline

[Timeline Link](timeline.pdf)

## Useful Links

[CAD](https://cad.onshape.com/documents/4743a97557c0a80d1585b0a7/w/2d6986bf76e2e55acb049bf2/e/10cf3b3fefc155c08becdc5d)
[C610 Motor Controller Library](https://github.com/stanfordroboticsclub/DJIC610Controller)
[BOLT Biped Driver](https://github.com/open-dynamic-robot-initiative/bolt)
[BRUCE Paper](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9811790)

## Log

<!-- Your Text Here. You may work with your mentor on this later when they are assigned -->
