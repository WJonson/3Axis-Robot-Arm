# 3-Axis Robot Arm

This project was made for a Introduction to Microprocessors class at SFSU.

This 3-Axis Robot Arm is a cheap alternative to the expensive robot arms for sale in the current market. Ideally, this arm is intended to practice coding in C with microcontrollers and micro-servos and in it's current form cannot lift heavy objects.


## A Summary of How it Works

Currently, the arm can only move 1 servo at a time and a servo is selected by 4 push-buttons. A potentiometer that is sampled will control the angular position of the servo, the servo of which can only move -90 to 90 degrees. Since analog voltages are used to control the angular position of the servo, the motors had a tendancy of "reseting" their position to the current position of the pot whenever a new servo was selected. To counter this, the user must first move the pot to the previous angular position that the selected servo was at before being choosen. This is a way of "unlocking the servo" and to allow the servos to maintain their set positions. Once "unlocked" the servo will move its position relative to the position on the pot.

Main Project Components:
```
1 T.I. TM4C123 Launchpad MCU
1 Tower Pro 996 Servo
3 Tower Pro 9G Microservo
1 5V,4A DC Wall-wart Power Supply
1 10kOhm Potentiometer
4 SPST Push Buttons
```
## Getting Started

Keil uVision IDE was used to compile and run the C program onto the Launchpad microcontroller. 
```
uVision Project File: 3_axis_arm_final.uvproj
Main Program File: 3_axis_arm_final.c
```
"3_axis_arm_final.c" is the main C file controlling the arm.

## Future Development

* PWM needs tuning to run smoother
* Rebuild arm with 3-D printed material
* Print PCB for new controller design
* Switch potentiometer for rotary encoder
* Add joy-stick controller so motor selection is not needed
* Add Autonomous mode

## Authors

* **Wesley Jonson** - *Initial work* - [WJonson](https://github.com/WJonson)

## License

This project is licensed under the MIT License - see the [LICENSE.md](LICENSE.md) file for details

## Acknowledgments

* **Fortzero** - Created instructable on how to build a 3-Axis Robot Arm
	https://www.instructables.com/id/4-Axis-Robot-Arm-DIY/
