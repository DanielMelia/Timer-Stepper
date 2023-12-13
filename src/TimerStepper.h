/*
*
* Timer Stepper Motor Driver Library
*
* Created: 25/10/2023
* Author : Daniel Melia
*
*/

#ifndef TimerStepper_h
#define TimerStepper_h

//------------------------------------------ CONTROL MODES -------------------------------------------------------------//
#define STP 0  //STOP mode. Motion not allowed
#define HOM 1
#define POS 2  //POSITION mode: the user inputs the amount of rotation and trajectory parameters (vel, acc, dec...)
#define VEL 3  //VELOCITY mode: or speed control mode. Useful for Joystick control for example. Inputs are velocity and direction
#define TST 4  // TEST

class TimerStepper
{
public:
  void Run_test();
	// SETUP FUNCTIONS
	static void InitialiseTimers();
  void MotorSetup(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage = false, float pitch = 0);
  void LimitSwtichesSetup(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value);
  void DisableMotor(bool disable);
  // RUN MOTOR FUNCTIONS
	void Home(int freq, bool dir);
	void Home_ovr(float position);
	void RotateSteps(int steps, bool dir, int freq, int acc_steps, int dec_steps);
	void RotateAngle(float angle_rad, float vel_rads, float acc_rads2, float dec_rads2);
	void RunLinearStage(float distance_mm, float vel_mms, float acc_mms2, float dec_mms2);
	void RunLinearStage_abs(float abs_trgPos_mm, float vel_mms, float acc_mms2, float dec_mms2);
  void StopMotor();
  void ResetPosition();
	// FEEDBACK FUNCTIONS
	bool IsMoving();
	bool IsHomed();
	long GetStepPosition();
	float GetPosition_mm_rad();
	byte GetCurrentControlMode();
	String PassMotorParametersToString();
	String PassMotorParametersAndIdToString();
  	// INTERRUP SERVICE ROUTINE
	void TimerMotor_Run();
private:
  void TimerTrigger(uint32_t rc);
  // Low-Level functions
	void set_direction(bool dir);
	float convert_AccSteps_2_AccRads2(int freq, int acc_steps);
	int convert_AccRads2_2_AccSteps(float acc_rads2, float vel_rads);
	void Step();

	// Hardware Settings --------------------------------------
	int _pulse_pin, _dir_pin, _ena_pin;		// Stepper Driver Digital Pins: pulse / direction / enabled
	int _ppr;		// Stepper Driver pulses per revolution setting
	double _step_size_rad, _step_size_mm, _pitch;	// Stepper motor step size (rads and/or mm) and screw pitch
	int _home_limit_sw_pin, _far_limit_sw_pin, _home_limit_sw_value, _far_limit_sw_value;	// Limit Switches pin numbers and pin output values
	bool _axis_isLinear;	// FALSE: rotational ; TRUE: linear stage
	int _motor_id;  

  // Position / Direction variables
  long _step_position, _min_step_position, _max_step_position;
  int _dir;

  // Status variables
  volatile bool _moving, _finished;  // Indicates if the motor is moving, and if motion has finished
  volatile bool _pulse_pin_state;
  bool _motor_homed;
  byte _control_mode;

  // Trajectory Parameters ----------------------------------
  volatile unsigned int _count;
  int _total_steps;
  int _acc_end;
	int _dec_start;
  unsigned int _c0;                                  // Initial counter value for the acceleration phase
  volatile unsigned int _rest = 0;
	volatile unsigned int _prev_counter_value;  // Keeps track of the previous counter value
  int _f_trg;                       // Target frequency of the pulse train (motor rotation speed)
	long _max_wf_freq; // = clock_freq / (2 * prescaler);  // the maximum achievable waveform frequency is given by switching twice at clock_freq/prescaler (Hz)

  float _position, _min_position, _max_position;
};

#if defined(ARDUINO_SAM_DUE)
  extern TimerStepper timerSteppers[9];
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  extern TimerStepper timerSteppers[1];
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  extern TimerStepper timerSteppers[4];
#else
  extern TimerStepper timerSteppers[1];  
#endif

#endif