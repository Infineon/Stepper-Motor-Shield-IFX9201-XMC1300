// ensure this library description is only included once
#ifndef IFX9201_StepperMotor_h
#define IFX9201_StepperMotor_h

#define IFX9201_STEPPERMOTOR_STD_DIS	11
#define IFX9201_STEPPERMOTOR_STD_STP	10
#define IFX9201_STEPPERMOTOR_STD_DIR	9

#define IFX9201_STEPPERMOTOR_SERIAL_INTERFACE_SPEED	19200

#define CONFIG_SERIAL_WRITE_CMD 	165u
#define CONFIG_SERIAL_READ_CMD 		166u

typedef enum IFX9201_STEPPERMOTOR_SteppingMode
{
	IFX9201_STEPPERMOTOR_STEPPINGMODE_FULL = 0x01u,				/**< Fullstep. */
	IFX9201_STEPPERMOTOR_STEPPINGMODE_HALF = 0x02u,				/**< Halfstep. */
	IFX9201_STEPPERMOTOR_STEPPINGMODE_MICROSTEP = 0x04			/**< Microstep. */
} IFX9201_STEPPERMOTOR_SteppingMode_t;

typedef enum IFX9201_STEPPERMOTOR_StoreConfig
{
	IFX9201_STEPPERMOTOR_STEPPINGMODE_DO_NOT_STORE_CONFIG = 0x00u,	/**< Do. */
	IFX9201_STEPPERMOTOR_STEPPINGMODE_STORE_CONFIG = 0x01u			/**< Do not. */
} IFX9201_STEPPERMOTOR_StoreConfig_t;

typedef struct IFX9201_STEPPERMOTOR_config{
	IFX9201_STEPPERMOTOR_SteppingMode_t SteppingpMode;			/**< Steppingp Mode. */
	uint32_t FreqPWMOut;										/**< FreqPWMOut. */
	uint32_t PWMDutyCycleNormFactor;							/**< PWMDutyCycleNormFactor (0 - 10000). */
	uint32_t NumMicrosteps;										/**< Number of Microsteps (SteppingpMode == IFX9201_STEPPERMOTOR_STEPPINGMODE_MICROSTEP) */
	IFX9201_STEPPERMOTOR_StoreConfig_t Store;					/**< Store or do not store config in NVM of Stepper Motor */
}IFX9201_STEPPERMOTOR_config_t;


// library interface description
class Stepper_motor {
  public:
    // constructors:
	Stepper_motor(uint16_t steps, uint16_t motor_pin_dis, uint16_t motor_pin_stp, uint16_t motor_pin_dir);

	// set pins' type as OUTPUT, set default speed and enable the stepper motor
	void begin();

	// Configure the Stepper Motor, HardwareSerial used -> needs to be connected to Debug Interface of Stepper Motor Control Shield
	bool configure(HardwareSerial &bus, IFX9201_STEPPERMOTOR_config_t *config);

	// Read current Config from the Stepper Motor, HardwareSerial used -> needs to be connected to Debug Interface of Stepper Motor Control Shield
	void configRead(HardwareSerial &bus, IFX9201_STEPPERMOTOR_config_t *config);

	// disable the stepper motor, set default speed and set pins' type as INPUT
	void end();

	// enable the stepper motor
	void enable();

	// disable the stepper motor
	void disable();

    // speed setter method:
    void setSpeed(float whatSpeed);

    // duty ratio of the STP signal setter method:
    void setDutyRatio(float whatRatio);

    // mover method:
    void step(int16_t number_of_steps);

    // move a given degree. num_of_degree > 0 means moving clockwise, num_of_degree < 0 means moving clockwise.
    // The degree has to be an integral multiple of ( 360 / number_of_steps )
    void move_degree(float num_of_degree);

    // move a given number of revolutions. num_of_revolution > 0 means moving clockwise, num_of_revolution < 0 means moving counterclockwise
    void move_revolution(float num_of_revolution);

    // update the counter which tracks ((the total number of steps) mod (number_of_steps)) the motor has walked, used for reset
    void update_accumulated_steps(int16_t steps);

    // reset the motor to the position where it was at the beginning
    void reset_motor();

    uint16_t version(void);

  private:
    void pinInit(void);
    void stepMotor(bool high_low);

    int16_t direction;            	// Direction of rotation
    uint32_t step_delay; 			// delay between steps, in ms, based on speed
    uint16_t number_of_steps;      	// total number of steps this motor can take for one revolution
    float duty_ratio;	      		// duty ratio of STP signal, by default it's 0.5, and it seems that it will influence the speed
    int16_t accumulated_steps;	  	// a counter tracking the number of steps that the motor has moved

    // motor pin numbers:
    uint16_t motor_pin_dis;
    uint16_t motor_pin_stp;
    uint16_t motor_pin_dir;

    uint32_t last_step_time; 		// time stamp in us of when the last step was taken
};

#endif

