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
	// setup the pins on the micro-controller:
	pinMode(motor_pin_dir, OUTPUT);
	pinMode(motor_pin_stp, OUTPUT);
	pinMode(motor_pin_dis, OUTPUT);

	// set the default speed at 1 rpm
	step_delay = 60L * 1000L * 1000L / (number_of_steps * 1);

	enable();
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
