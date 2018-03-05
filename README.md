# Stepper-Motor-Shield-IFX9201-XMC1300 Library for Arduino
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

## Usage
Please follow the example sketches in the /examples directory in this library to learn more about the usage of the library.

## Board Information, Datasheet and Additional Information
A PDF summarizing the features and layout of the Stepper Motor Shield IFX9201 XMC1300 is stored on the Infineon homepage [here](https://www.infineon.com/dgdl/Infineon-Stepper_Motor_Control_Shield_with_IFX9201SG_XMC1300_for_Arduino-UM-UM-v01_00-EN.pdf?fileId=5546d462617643590161c23fa5120aa1).
The datasheet for the IFX9201 can be found here [IFX9201 Datasheet](https://www.infineon.com/dgdl/Infineon-IFX9201SG-DS-v01_01-EN.pdf?fileId=5546d4624cb7f111014d2e8916795dea).
