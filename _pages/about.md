---
title:  "About"
excerpt: "About"
permalink: /about/
layout: single
author_profile: true
classes: wide
---

## Education

- 2012-03 ~ 2020-02 \| B.E., Mechanical Engineering, Hanyang University, Korea
- 2009-03 ~ 2012-02 \| English/Chinese, Puil Foreign Language High School, Korea

## Experience

- 2022-10 ~ present \| Robotics Software Engineer, Wootzano Ltd.
- 2020-01 ~ 2022-07 \| Robotics Engineer, LG Electronics Inc.

## Project

### Fruit packing robot(2022.10 ~ present)
<br>
<br>
<br>

### Robot operating software(2021-12 ~ 2022.07)

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/robotoperatingsoftware.png" alt="">
- Tool: C#, Prism framework, WPF, Enterprise arhitecture
- Role:
  - Design software architecture
  - Wrap external libraries for custom use cases
  - Abstract different types of devices for modularity and extensiblility
  - Develop user interface
  - Apply to production line

### Robot-vision system(2021-05 ~ 2021.11)

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/robot_visionsystem.png" alt="">
- Robot: UR, LG
- Camera: Intel Realsense, LG
- Tool: C++, Python, OpenCV
- Role:
  - Implement hand-eye calibration algorithm
  - Identify how to evaluate the result of hand-eye calibration
  - Automate calibration process
  - Apply object detection algorithm
    - AI(CNN, YOLO, YOLACT)
    - Image processing(threshold, template matching, etc.)

### Collaborative Robot(2020-03 ~ 2021.06)

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/lgcloirobot.png" alt="">
- Tool: Python, Creo
- Role:
  - Develop software to test actuators and analyse result data
  - Define CMF specification of the robot
  - Design assembling tools to improve productivity
  - Define assembling order for mass production
  - Follow NPI process

### Teleoperation Device(2020-03 ~ 2020-06)

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/teleoperation.png" alt="">

- Controller: Arduino, PC(Python)
- Actuator: Dynamixel servo motor
- Hardware: made by 3d printer
- Tool: Python, Creo, Makerbot 3d printer
- Role:
  - Design robot links which connect 6 actuators in series
  - Design circuit to convert full duplex signal to half duplex??
  - Calculate forward/inverse kinematics of 6-axis robot manipulator
  - Develop python program to visualize motion of the robot

### Drone Manipulator(2019-03 ~ 2019-06)

<img src="{{ site.url }}{{ site.baseurl }}/assets/images/droneManipulator.jpg" alt="">

- Controller: Arduino(ATmega2560), ESC
- Actuator: BLDC motor, Servo motor
- Sensor: IMU sensor, Geomagnetic sensor, Ultrasonic sensor
- Remote Controller: Android application
- Etc: Bluetooth module, Battery
- Tool: MATLAB, MIT App Inventor, Arduino IDE
- Role:
  - Control BLDC motor and servo motor using Arduino board and ESC
  - Read and process signal from IMU, geomagnetic sensor, ultrasonic sensor
  - Develop Android application which communicates with bluetooth module
  - Simulate controller with Matlab/Simulink
  - Write flight control logic on Arduino board