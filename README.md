# Stepper Motor Control Shield with IFX9201 & XMC1300 - Library for Arduino

[![Build Status](https://travis-ci.org/Infineon/Stepper-Motor-Shield-IFX9201-XMC1300.svg?branch=master)](https://travis-ci.org/Infineon/Stepper-Motor-Shield-IFX9201-XMC1300)

<img src="https://github.com/Infineon/Assets/blob/master/Pictures/Stepper-Motor-Control-Shield_with-IFX9201-and-XMC1300.jpg" width="400">

Library of Infineon's [Stepper-Motor-Shield-IFX9201-XMC1300](https://www.infineon.com/cms/de/product/evaluation-boards/kit_xmc1300_ifx9201/) for Arduino.

## Summary
The stepper motor control shield based on Infineon’s h-bridge [IFX9201](https://www.infineon.com/cms/en/product/power/motor-control-ics/intelligent-motor-control-ics/integrated-full-bridge-driver/ifx9201sg/) and [XMC1300 microcontroller](https://www.infineon.com/cms/en/product/microcontroller/32-bit-industrial-microcontroller-based-on-arm-cortex-m/32-bit-xmc1000-industrial-microcontroller-arm-cortex-m0/) is capable of driving the two coils in a stepper motor featuring dual-h-bridge configuration. The implemented integrated IFX9201 h-bridges can be controlled by a STEP-signal via the STEP Pin. Interfacing to a microcontroller is made easy by the integrated XMC1300 microcontroller that holds the peripherals to allow high-speed current control. Micro-stepping of the stepper motor can be achieved using the internal comparators. Operational amplifiers are installed to adapt the motor current sense signal to the microcontroller’s input levels.

This high current stepper motor control boards is compatible with Arduino microcontroller boards and with Infineon’s XMC™ microcontroller kits using the Arduino form factor.

## Key Features and Benefits
* Compatible with Arduino Uno R3
* Capable of high performance current control
* Microstepping degree can be adjusted by software
* Driver circuit with logic level inputs
* Fast and inexpensive prototyping of stepper motor control
* Simple testing of microstepping algorithms
* Diagnose pin to allow hardware feedback during development
* Overtemperature shut down with latch behavior and undervoltage shut down of the power section

## Target Applications:
* Stepper motors up to 5A phase current
* 24V nominal input voltage for the power stage
* Average motor current 3A without additional cooling effort, 5A possible with proper cooling

## Installation
First of all, please download this repository from GitHub by clicking on the field with the .zip file in the [releases](https://github.com/Infineon/Stepper-Motor-Shield-IFX9201-XMC1300/releases) of this repository or directly [here](https://github.com/Infineon/Stepper-Motor-Shield-IFX9201-XMC1300/releases/download/V1.0.0/Stepper-Motor-Shield-IFX9201-XMC1300.zip).

![Download Library](https://raw.githubusercontent.com/infineon/assets/master/Pictures/Releases_Generic.jpg)

To install the Stepper Motor Shield IFX9201 XMC1300 Library in the Arduino IDE, please go now to **Sketch** > **Include Library** > **Add .ZIP Library...** in the Arduino IDE and navigate to the downloaded .ZIP file of this repository release. The library will be installed in your Arduino sketch folder in libraries and you can select as well as include this one to your project under **Sketch** > **Include Library** > **Stepper Motor Shield IFX9201 XMC1300**.

![Install Library](https://raw.githubusercontent.com/infineon/assets/master/Pictures/Library_Install_ZIP.png)

## Getting started
After installing this library in your Arduino IDE, you can do a quick test of your stepper motor, using the example `testStepperMotor.ino`. The default coil current is set to 1A. If your motor is rated for less, please read on below **before** testing the motor. Otherwise it could be damaged.

## Overview

<img src="https://github.com/Infineon/Assets/blob/master/Pictures/ifx9201_stepper_shield_overview.png" width="500">

### Hardware connections

The stepper motor will not be powered through the 5V supply of the Arduino. Please connect an external power source (typ. DC 12-24V) to the PWR_MOT input of your stepper motor shield.

<img src="https://github.com/Infineon/Assets/blob/master/Pictures/ifx9201_stepper_shield_hw_con.png" width="600">

### Configuration
In this section you learn, how to change the configuration settings of your motor shield to fit your target application.

#### Overview
In order to change the configuration of your stepper motor shield, please follow these steps:
1) Open the example sketch `configureStepperMotor.ino` in your Arduino IDE
2) Adapt the parameters according to your preferences, with help of the *Parameters* section below
3) Compile and upload the Arduino sketch to your Arduino
4) Connect the pins as shown in the *Hardware* section below
5) Press the reset button of the Arduino and wait, until the builtin LED of the Arduino lights up constantly
6) The new configuration is now running on the stepper motor shield
7) Remove the serial pin connection from step (4)
8) Flash your actual motor control code to the Arduino, e.g. the example Sketch `testStepperMotor.ino`

#### Parameters
In the example sketch `configureStepperMotor.ino` you find the different configuration parameters:

| Parameter     | Options       | Default |
| :------------ |:--------------|:--------|
| SteppingMode  | IFX9201_STEPPERMOTOR_STEPPINGMODE_FULL,<br>IFX9201_STEPPERMOTOR_STEPPINGMODE_HALF,<br>IFX9201_STEPPERMOTOR_STEPPINGMODE_MICROSTEP | Full step mode
| FreqPWMOut    | PWM output frequency.<br>For higher currenty stay at some kHz,<br>for low current up to 20 kHz is possible.<br><br>*Note: The effective frequency may be different in order to limit the current.* | 3000 \[Hz\] |
| PWMDutyCycleNormFactor | Current limit. Calculated as: *current \[A\] = value / 3333* | 3333 (1A) |
| NumMicrosteps | Microsteps per full step: 8, 12, 16, 20, 24, ... 128. Only valid in microstepping mode. | n/a |
| Store         | IFX9201_STEPPERMOTOR_STEPPINGMODE_DO_NOT_STORE_CONFIG,<br>IFX9201_STEPPERMOTOR_STEPPINGMODE_ STORE_CONFIG.<br><br>**If set**, the non-volatile memory of the onboard microcontroller will be updated with the new values.<br>**If not set**, the configuration will be set to the default parameters after each power cycle.<br><br>*Note: The non-volatile memory has a high, but limited amount of writing cycles, so it is not recommanded to e.g. script a regular re-write of the configuration parameters to the memory.*

#### Hardware connections
In order to write the new parameters to the onboard microntroller, the serial port (UART) of the Arduino is used. Before executing the `configureStepperMotor.ino` sketch, you have to connect the onboard microcontroller serial port to the one of the Arduino as shown here:

<img src="https://github.com/Infineon/Assets/blob/master/Pictures/ifx9201_stepper_shield_serial_con.png" width="400">

**Important note:** The Arduino cannot be flashed via USB, while these pins are connected. Please disconnect them, before trying to flash the Arduino.

Please refer to the [Board User Manual](https://www.infineon.com/dgdl/Infineon-Stepper_Motor_Control_Shield_with_IFX9201SG_XMC1300_for_Arduino-UM-UM-v01_00-EN.pdf?fileId=5546d462617643590161c23fa5120aa1) for additional and more detailed information.

## Remarks

### Heat & Cooling
The two IFX9201 half bridges on the stepper motor shield can become very hot during operation. Do not touch them!
The half bridges have a temperature protection and will not die because of overheating. However, if they overheat, they will disable the output until they cool down. Usually, that's not desired, so lets see what we can do about that:

| Motor coil current | Minimum cooling |
| :----------------- |:----------------|
| < 2A               | none            |
| < 3A               | small passive heatsink e.g. [this](https://www.amazon.de/gp/product/B07JJK92DJ) |
| <= 5A              | additional cooling effort, e.g. active cooling, proper heatsink |

*Note: these statements are empirical values and depend on environment temperature, operation time and supply voltage. Values are based on an environment temperature of 21°C, supply voltage of 12V and operation time ~1h. For long/continous operation proper cooling is always recommended.*

## Board Information, Datasheet and Additional Information
A PDF summarizing the features and layout of the Stepper Motor Shield IFX9201 XMC1300 is stored on the Infineon homepage [here](https://www.infineon.com/dgdl/Infineon-Stepper_Motor_Control_Shield_with_IFX9201SG_XMC1300_for_Arduino-UM-UM-v01_00-EN.pdf?fileId=5546d462617643590161c23fa5120aa1).
The datasheet for the IFX9201 can be found here [IFX9201 Datasheet](https://www.infineon.com/dgdl/Infineon-IFX9201SG-DS-v01_01-EN.pdf?fileId=5546d4624cb7f111014d2e8916795dea).
