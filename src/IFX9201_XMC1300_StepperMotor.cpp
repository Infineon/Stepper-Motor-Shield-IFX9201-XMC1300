/**
 * IFX9201_XMC1300_StepperMotor.cpp - Library for Arduino of Infineon's stepper motor control shield with 
 * Infineon’s h-bridge IFX9201 and XMC1300 microcontroller.
 *
 * The stepper motor control shield based on Infineon’s h-bridge IFX9201 and XMC1300 microcontroller is 
 * capable of driving the two coils in a stepper motor featuring dual-h-bridge configuration. 
 * The implemented integrated IFX9201 h-bridges can be controlled by a STEP-signal via the STEP Pin. 
 * 
 * Have a look at the application note/reference manual for more information.
 * 
 * Copyright (c) 2018 Infineon Technologies AG
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the 
 * following conditions are met:   
 *                                                                              
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the following 
 * disclaimer.                        
 * 
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following 
 * disclaimer in the documentation and/or other materials provided with the distribution.                       
 * 
 * Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote 
 * products derived from this software without specific prior written permission.                                           
 *                                                                              
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE  
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR  
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   
 */

#include "Arduino.h"

#include "IFX9201_XMC1300_StepperMotor.h"

/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper_motor::Stepper_motor(uint16_t steps, uint16_t pin_dir, uint16_t pin_stp, uint16_t pin_dis)
{
	direction = 0;      // motor direction
	last_step_time = 0; // time stamp in us of the last step taken
	number_of_steps = steps; // total number of steps for this motor
	duty_ratio = 0.5;
	accumulated_steps = 0;

	// set the speed at 1 rpm, same as default speed
	step_delay = 60L * 1000L * 1000L / (number_of_steps * 1);

	// Arduino pins for the motor control connection:
	motor_pin_dir = pin_dir;
	motor_pin_stp = pin_stp;
	motor_pin_dis = pin_dis;
}

void Stepper_motor::begin()
{
	pinInit();

	// set the default speed at 1 rpm
	step_delay = 60L * 1000L * 1000L / (number_of_steps * 1);

	enable();
}

void Stepper_motor::pinInit(void)
{
	// Set all Pins to LOW
	digitalWrite(motor_pin_dir, LOW);
	digitalWrite(motor_pin_stp, LOW);
	digitalWrite(motor_pin_dis, LOW);

	// setup the pins on the micro-controller:
	pinMode(motor_pin_dir, OUTPUT);
	pinMode(motor_pin_stp, OUTPUT);
	pinMode(motor_pin_dis, OUTPUT);
}

bool Stepper_motor::configure(HardwareSerial &bus, IFX9201_STEPPERMOTOR_config_t *config)
{
	bool success = true;
	uint8_t data_to_send[32] = {0};
	data_to_send[17] =	 (uint8_t)config->SteppingpMode;
	data_to_send[18] =	 (uint8_t)(config->FreqPWMOut	>> 24);
	data_to_send[19] =	 (uint8_t)(config->FreqPWMOut 	>> 16);
	data_to_send[20] =	 (uint8_t)(config->FreqPWMOut 	>> 8);
	data_to_send[21] =	 (uint8_t)(config->FreqPWMOut);
	data_to_send[22] =	 (uint8_t)(config->PWMDutyCycleNormFactor	>> 24);
	data_to_send[23] =	 (uint8_t)(config->PWMDutyCycleNormFactor 	>> 16);
	data_to_send[24] =	 (uint8_t)(config->PWMDutyCycleNormFactor 	>> 8);
	data_to_send[25] =	 (uint8_t)(config->PWMDutyCycleNormFactor);
	data_to_send[26] =	 (uint8_t)config->NumMicrosteps;

	data_to_send[27] = (uint8_t)config->Store;
	data_to_send[30] = CONFIG_SERIAL_WRITE_CMD;

	pinInit();

	bus.begin(IFX9201_STEPPERMOTOR_SERIAL_INTERFACE_SPEED);

	bus.write(data_to_send, 32u);

	return success;
}

void Stepper_motor::configRead(HardwareSerial &bus, IFX9201_STEPPERMOTOR_config_t *config)
{
	uint8_t num_received_bytes = 0u;
	uint32_t start_read = millis();
	uint8_t data[32] = {0};
	data[30] = CONFIG_SERIAL_READ_CMD;

	pinInit();

	bus.begin(IFX9201_STEPPERMOTOR_SERIAL_INTERFACE_SPEED);

	bus.write(data, 32u);

	while(num_received_bytes < 32)
	{
		if(bus.available())
		{
			data[num_received_bytes] = bus.read();
			start_read = millis();
			num_received_bytes++;
		}
		if( (millis() - start_read) > 30u )
		{
			break;
		}
	}

	if( (num_received_bytes == 32u) && (config != NULL) )
	{
		config->SteppingpMode = (IFX9201_STEPPERMOTOR_SteppingMode_t)data[17];
		config->FreqPWMOut = ((uint32_t)data[18] << 24u) | \
								((uint32_t)data[19] << 16u) | \
								((uint32_t)data[20] << 8u) | \
								((uint32_t)data[21]);
		config->PWMDutyCycleNormFactor = ((uint32_t)data[22] << 24u) | \
								((uint32_t)data[23] << 16u) | \
								((uint32_t)data[24] << 8u) | \
								((uint32_t)data[25]);
		config->NumMicrosteps = data[26];

		config->Store = (IFX9201_STEPPERMOTOR_StoreConfig_t)data[27];
	}
}

void Stepper_motor::end()
{
	disable();

	// set the speed at 1 rpm, same as default speed
	step_delay = 60L * 1000L * 1000L / (number_of_steps * 1);

	// setup the pins on the micro-controller:
	pinMode(motor_pin_dir, INPUT);
	pinMode(motor_pin_stp, INPUT);
	pinMode(motor_pin_dis, INPUT);
}

void Stepper_motor::enable()
{
	digitalWrite( motor_pin_dis, HIGH);
}

void Stepper_motor::disable()
{
	digitalWrite( motor_pin_dis, LOW);
}

/*
 * Sets the speed in revolutions per minute
 */
void Stepper_motor::setSpeed(float whatSpeed)
{
	step_delay = 60L * 1000L * 1000L / (number_of_steps * whatSpeed);
}

void Stepper_motor::setDutyRatio(float whatRatio)
{
	duty_ratio = whatRatio;
}

void Stepper_motor::move_degree(float number_of_degree)
{
	if(number_of_degree > 0)
	{
		step((int16_t)(number_of_steps * number_of_degree / 360));
	}
	else
	{
		step((int16_t)(number_of_steps * number_of_degree / 360));
	}
}

void Stepper_motor::move_revolution(float number_of_revolution)
{
	step((int16_t)(number_of_steps * number_of_revolution));
}

/*
 * Moves the motor steps_to_move steps.  If the number is negative,
 * the motor moves in the reverse direction.
 */
void Stepper_motor::step(int16_t steps_to_move)
{
	update_accumulated_steps(steps_to_move);

	uint16_t steps_left = abs(steps_to_move);  // how many steps to take

	// determine direction based on whether steps_to_mode is + or -:
	if (steps_to_move > 0) { direction = 1; }
	if (steps_to_move < 0) { direction = 0; }

	digitalWrite(motor_pin_dir, direction);

	// decrement the number of steps, moving one step each time:
	while (steps_left > 0)
	{
		uint32_t now = micros();

		while(now - last_step_time < (duty_ratio * step_delay)) {
			now = micros();
		}

		// get the timeStamp of when the motor half stepped:
		last_step_time = now;

		// output a HIGH half step to STP pin
		stepMotor(HIGH);

		while(now - last_step_time < (duty_ratio * step_delay)) {
			now = micros();
		}

		// get the timeStamp of when the motor half stepped:
		last_step_time = now;

		// output a LOW half step to STP pin
		stepMotor(LOW);

		// decrement the steps left only after a LOW half step:
		steps_left--;
	}
}

/*
 * Moves the motor forward or backwards.
 */
void Stepper_motor::stepMotor(bool high_low)
{
	digitalWrite(motor_pin_stp, high_low);
}

void Stepper_motor::update_accumulated_steps(int16_t steps)
{
	accumulated_steps += steps;

	if(accumulated_steps >= number_of_steps)
	{
		accumulated_steps -= number_of_steps;
	}

	if(accumulated_steps <= -(number_of_steps))
	{
		accumulated_steps += number_of_steps;
	}
}

void Stepper_motor::reset_motor()
{
	step(-accumulated_steps);
}

/*
  version() returns the version of the library:
*/
uint16_t Stepper_motor::version(void)
{
  return 1;
}
