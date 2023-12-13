/*
*
* Timer Stepper Motor Driver Library
*
* Created: 25/10/2023
* Author : Daniel Melia
*
*
* Library to control stepper motors using hardware timers on several Arduino boards.
* Compatible with Arduino Uno, Nano, Mega and Due boards. It uses one hardware interrupt
* to control each motor.
*
* This library contains function to create trapezoidal acceleration/deceleration profiles,
* run steppers in motor-only mode or linear stage mode, position and velocity control and homing.
*
* It also includes functions to configure the motor pins, timers, limit switches or get feedback.
*
* Needs to be used together with another library to handle configuration and control of timer interrupts
* as well as Interrupt Service Routines
* - TimersDue.h on ARDUINO DUE to control a maximum of 9 motors
* - TimersMega.h on ARDUINO MEGA to control a maximum of 4 motors
* - TimersUnoNano.h on ARDUINO UNO/NANO to control a single motor
*
* On the Due board it uses 9 32-bit counters.
* On the mega it uses the 4 16-bit counters (timers 1, 3, 4 and 5)
* On the Uno and Nano uses timer 1 (16-bit)
*
*/

#include "Arduino.h"
#include "TimerStepper.h"
#include "board_list.h"
#include "limits.h"

// Include files to configure and control hardware timers depending on the board used
#if defined(ARDUINO_SAM_DUE)
  #include "due\TimersDue.h"
  #include "due\TimersDue.cpp"
  const int n = 9; 
  #define BOARD "DUE"
#elif defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
  #include "uno_nano\TimersUnoNano.h"
  #include "uno_nano\TimersUnoNano.cpp"
  const int n = 1; 
  #define BOARD "UNO_NANO"
#elif defined(ARDUINO_AVR_MEGA) || defined(ARDUINO_AVR_MEGA2560)
  #include "mega\TimersMega.h"
  #include "mega\TimersMega.cpp"
  const int n = 4; 
  #define BOARD "MEGA"
#else
  const int n = 1; 
  #define BOARD "UNKNOWN"
#endif

TimerStepper timerSteppers[n];

void RunMotor1(){
  timerSteppers[0].TimerMotor_Run();
  }
void RunMotor2(){
  if(n >= 1){
    timerSteppers[1].TimerMotor_Run();
  }}
void RunMotor3(){
  if(n >= 2){
    timerSteppers[2].TimerMotor_Run();
  }}
void RunMotor4(){
  if(n >= 3){
    timerSteppers[3].TimerMotor_Run();
  }}
void RunMotor5(){
  if(n >= 4){
    timerSteppers[4].TimerMotor_Run();
  }}
void RunMotor6(){
  if(n >= 5){
    timerSteppers[5].TimerMotor_Run();
  }}
void RunMotor7(){
  if(n >= 6){
    timerSteppers[6].TimerMotor_Run();
  }}
void RunMotor8(){
  if(n >= 7){
    timerSteppers[7].TimerMotor_Run();
  }}
void RunMotor9(){
  if(n >= 8){
    timerSteppers[8].TimerMotor_Run();
  }}    


// SETUP FUNCTIONS
#pragma region SETUP FUNCTIONS

/** Initialise Motors and Hardware Timers
 * Assign a motor Id to every timerStepper object.
 * Assing a hardware timer to every timer object.
 * Configure timer interrupt subroutiner.
 */
void TimerStepper::InitialiseTimers() {
  int st_count = sizeof(timerSteppers) / sizeof(timerSteppers[0]);  // sizeof operator returns the size of the object in bytes

  for (int i = 0; i < st_count; i++) {
    timerSteppers[i]._motor_id = i;   
  }

  timers[0].init();
  if(BOARD == "DUE"){
    Serial.println("Initializing DUE timers...");
    timers[0].attachInterrupt(RunMotor1);
    timers[1].attachInterrupt(RunMotor2);
    timers[2].attachInterrupt(RunMotor3);
    timers[3].attachInterrupt(RunMotor4);
    timers[4].attachInterrupt(RunMotor5);
    timers[5].attachInterrupt(RunMotor6);
    timers[6].attachInterrupt(RunMotor7);
    timers[7].attachInterrupt(RunMotor8);
    timers[8].attachInterrupt(RunMotor9);
  }
  else if(BOARD == "MEGA"){
    Serial.println("Initializing MEGA timers...");   
    timers[0].attachInterrupt(RunMotor1);
    timers[1].attachInterrupt(RunMotor2);
    timers[2].attachInterrupt(RunMotor3);
    timers[3].attachInterrupt(RunMotor4);  
  }
  else if(BOARD == "UNO_NANO"){
    Serial.println("Initializing UNO_NANO timer...");
    timers[0].attachInterrupt(RunMotor1);
  }  
  for (int i = 0; i < st_count; i++) {
    timerSteppers[i]._max_wf_freq = timers[i]._max_wf_freq;  
  }
  
}

void TimerStepper::TimerTrigger(uint32_t rc)
{

	_count = 0;
	_moving = true;  // Indicate that motion started
  Serial.println("Timer Trigger...");
  timers[_motor_id].TimerTrigger(rc);
 
	//TimerStart(_timer, _channel, _irq, rc);

}
/** Setup motor hardware properties
 * @param pulse_pin Pin connected to the driver's pulse signal
 * @param dir_pin Pin connected to the driver's direction signal
 * @param ena_pin Pin connected to the driver's enable signal
 * @param ppr Pulses per revolution of the motor
 * @param linear_stage TRUE for linear motion (linear stage). FALSE for rotational motion (motor)
 * @param pitch Pitch of the screw in mm (in case it's a linear stage)
 */
void TimerStepper::MotorSetup(int pulse_pin, int dir_pin, int ena_pin, int ppr, bool linear_stage, float pitch) {
	// Configure Microcontroller Pins
	pinMode(pulse_pin, OUTPUT);
	pinMode(dir_pin, OUTPUT);
	pinMode(ena_pin, OUTPUT);
	_pulse_pin = pulse_pin;
	_dir_pin = dir_pin;
	_ena_pin = ena_pin;

	DisableMotor(false);

	// Configure Axis
	_axis_isLinear = linear_stage;
	_ppr = ppr;
	_step_size_rad = (2 * PI) / ppr;
	if (_axis_isLinear) {
		_pitch = pitch;
		_step_size_mm = _pitch / ppr;
	}

	// By default set limit switches pins to -1 to indicate they are not configured
	_home_limit_sw_pin = -1;
	_far_limit_sw_pin = -1;
	
	// Initialise Position Variables
	_step_position = 0;
	_min_step_position = LONG_MIN;
	_max_step_position = LONG_MAX;

}
/** Setup travel limits and switchces hardware parameter
 * @param home_ls_pin Pin connected to the home limit switch
 * @param far_ls_pin Pin connected to the far limit switch
 * @param min_pos_value Minimum position value
 * @param max_pos_value Maximum travel value
 */
void TimerStepper::LimitSwtichesSetup(int home_ls_pin, int far_ls_pin, float min_pos_value, float max_pos_value) {
	// Configure Microcontroller Pins
	pinMode(home_ls_pin, INPUT_PULLUP);
	pinMode(far_ls_pin, INPUT_PULLUP);
	_home_limit_sw_pin = home_ls_pin;
	_far_limit_sw_pin = far_ls_pin;

	_min_position = min_pos_value;
	_max_position = max_pos_value;

	if (!_axis_isLinear) {	// Rotational axis
		_max_step_position = ((_max_position - _min_position) / _step_size_rad);
	}
	else {	// Linear axis
		_max_step_position = ((_max_position - _min_position) / _step_size_mm);
	}

}
/** Setup motor enabled/disabled state
 * Controls the driver's enable pin to enable/disable motor control by hardware
 * @param disable If TRUE the motor control is disabled. Set to FALSE to enable
 */
void TimerStepper::DisableMotor(bool disable) {
	if (disable) {
		digitalWrite(_ena_pin, HIGH);
	}
	else {
		digitalWrite(_ena_pin, LOW);
	}
}

#pragma endregion

// LOW-LEVEL FUNCTIONS
#pragma region LOW-LEVEL FUNCTIONS
/** Set motor direction
 * Controls the driver's direction pin to rtate motor CW / CCW
 * @param dir Set TRUE or FALSE to control direction of the motor
 */
void TimerStepper::set_direction(bool dir) {
	if (dir) {
		digitalWrite(_dir_pin, HIGH);
		_dir = 1;
	}
	else {
		digitalWrite(_dir_pin, LOW);
		_dir = -1;
	}
}
/** Calculate acceleration in radians per second square to reach a final velocity in a given number of steps
 * @param freq target frequency at which the step pulses are sent when final velocity is reached
 * @param acc_steps Number of steps to reach final speed
 * @return acceleration in radians per second square
 */
float TimerStepper::convert_AccSteps_2_AccRads2(int freq, int acc_steps) {
	float v_0 = (float(freq) / 10) * _step_size_rad; // Calculate initial, final and average velocities
	float v_f = freq * _step_size_rad;
	float v_avg = (v_f + v_0) / 2;
	float travel = acc_steps * _step_size_rad;       // Travel (rad) at the end of acceleration phase
	float time = travel / v_avg;                // Time taken for the motor to reach the end of the acceleration phase
	float acc = (v_f - v_0) / time;             // Acceleration value (rad/s2)

	return acc;

}
/** Calculate number of steps needed to reach a final velocity with a given acceleration in radians per second square
 * @param acc target acceleration in radians per second square
 * @param vel target velocity in mm per second
 * @return acceleration in radians per second square
 */
int TimerStepper::convert_AccRads2_2_AccSteps(float acc_rads2, float vel_rads) {
	// NOTE: try setting v_0 = 0 and see the difference in results
	float v_0 = vel_rads / 10;
	float v_avg = (vel_rads + v_0) / 2;
	float time = (vel_rads - v_0) / acc_rads2;
	float travel = time * v_avg;
	int acc_steps = travel / _step_size_rad;

	return acc_steps;
}
/** Send pulse to the driver to perform a step
 * Toggle driver's pulse pin and keeps track of pin state and step count
 */
void TimerStepper::Step() {
	if (_pulse_pin_state) {
		digitalWrite(_pulse_pin, LOW);
		_pulse_pin_state = LOW;
	}
	else {
		digitalWrite(_pulse_pin, HIGH);
		_count++;
		_step_position += _dir;
		_pulse_pin_state = HIGH;
	}
}
#pragma endregion

// RUN MOTOR FUNCTIONS
#pragma region RUN MOTOR FUNCTIONS
/** Start motor homing procedure
 * @param freq step frequency 
 * @param dir target velocity in mm per second
 */
void TimerStepper::Home(int freq, bool dir) {

	if (_far_limit_sw_pin == -1 && _home_limit_sw_pin == -1){ return; }

	set_direction(dir);
	_f_trg = freq;
	
	// Initialise the Timer
	_control_mode = HOM;
	unsigned int compare_value = (unsigned int)(_max_wf_freq / (long)_f_trg);

	_motor_homed = false;
	//_motor_homed_phase_1 = false;
	//_H_homing_counter = 0;
	//_F_homing_counter = 0;
	//_H_F_sensor_triggered = false;

	TimerTrigger(compare_value);

}
/** Override motor homing procedure
 * Setup initial position of the motor without performing homing routine
 * @param position initial motor position
 */
void TimerStepper::Home_ovr(float position) {
	if (position > _max_position) { position = _max_position; }
	if (position < _min_position) { position = _min_position; }
	_position = position;
	_step_position = long((_position / (_max_position - _min_position)) * (float)_max_position);
	_min_step_position = 0;
	//_max_step_position = _max_step_number;
	_motor_homed = true;
}
/** Rotate the motor a given number of steps
 * @param steps number of steps to rotate
 * @param dir direction of movement (CW / CCW)
 * @param freq frequency of the steps when target speed is reached
 * @param acc_steps number of steps to reach target speed
 * @param dec_steps number of steps to decelarate from target speed to stop
 */
void TimerStepper::RotateSteps(int steps, bool dir, int freq, int acc_steps, int dec_steps) {
  // Input arguments Check
	if (acc_steps < 0) {acc_steps = 0;}
	else if (acc_steps > steps) {acc_steps = steps;}
	if (dec_steps < 0) {dec_steps = 0;}
	else if (dec_steps > steps) {dec_steps = steps;}

	if (acc_steps + dec_steps > steps) {
		acc_steps = int(steps / 2);
		dec_steps = int(steps / 2);
	}

	// Update trapezoidal profile parameters
	_total_steps = steps;
	_acc_end = acc_steps;
	_dec_start = steps - dec_steps;
	_f_trg = freq;

	// Set motor direction
	set_direction(dir);

	// Calculate first timer/counter value
  
	if (acc_steps != 0) {
		float acc = convert_AccSteps_2_AccRads2(freq, acc_steps);
    Serial.print("acc_mms2: "); Serial.println(acc);
		_c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * _step_size_rad / acc));
	}
	else {
		_c0 = _max_wf_freq / _f_trg;
	}

	// Initialise the Timer
  _control_mode = POS;
  Serial.println("Setup parameters:");
  Serial.print("freq: "); Serial.println(freq);
  Serial.print("acc_steps: "); Serial.println(acc_steps);
  Serial.print("dec_steps: "); Serial.println(dec_steps);

  
  Serial.print("max_freq: "); Serial.println(_max_wf_freq);
  Serial.print("step_size_rad: "); Serial.println(_step_size_rad);

  Serial.print("CM: "); Serial.println(_control_mode);
  Serial.print("c0: "); Serial.println(_c0);
	
	TimerTrigger(_c0);
}
/** Rotate the motor a given angle in radians
 * @param angle_rad angle to rotate in radians
 * @param vel_rads target velocity in radians per second
 * @param acc_rads2 target acceleration in radians per second square
 * @param dec_rads2 target deceleration in radians per second square
 */
void TimerStepper::RotateAngle(float angle_rad, float vel_rads, float acc_rads2, float dec_rads2) {
  // Input arguments Check
	if (vel_rads <= 0 || acc_rads2 < 0 || dec_rads2 < 0) {return;}

	// Calculate trapezoidal profile parameters
	_total_steps = round(abs(angle_rad) / _step_size_rad);  // Calculate number of steps to reach target angle
	_f_trg = (int)(vel_rads / _step_size_rad);                    // Calculate targer step frequency

	long acc_steps = (vel_rads * vel_rads) / (2 * _step_size_rad * acc_rads2);  // Number of steps in acceleration mode
	long acc_lim = (_total_steps * dec_rads2) / (acc_rads2 + dec_rads2);      // Maximum number of steps allowed

	// If acceleration limit is reached before maximum speed
	if (acc_lim <= acc_steps) {
		_acc_end = acc_lim;          // Calculate steps to acceleration end
		_dec_start = acc_lim + 1;    // Calculate steps to deceleration start
	}
	// If maximum speed is reached before acceleration limit
	else {
		long decel_val = -acc_steps * acc_rads2 / dec_rads2;
		_acc_end = acc_steps;                    // Calculate steps to acceleration end
		_dec_start = _total_steps + decel_val;    // Calculate steps to deceleration start
	}

	if (acc_rads2 == 0) { _acc_end = 0; }
	if (dec_rads2 == 0) { _dec_start = _total_steps; }

	// Calculate first timer/counter 	
	if (acc_rads2 != 0) {_c0 = (int)(0.676 * _max_wf_freq * sqrt(2 * _step_size_rad / acc_rads2));}
	else {_c0 = _max_wf_freq / _f_trg;}
	// Set motor direction
	bool dir;
	if (angle_rad >= 0) {dir = true;}
	else {dir = false;}
	set_direction(dir);
	// Initialise Timer
	_control_mode = POS;
	TimerTrigger(_c0);
}
/** Move the motor in linear stage mode a given distance
 * The motor needs to be configured as linear stage when using the MotorSetup function
 * @param distance_mm distance to move in millimeters
 * @param vel_mms target velocity in millimeters per second
 * @param acc_mms2 target acceleration in millimeters per second square
 * @param dec_mms2 target deceleration in millimeters per second square
 */
void TimerStepper::RunLinearStage(float distance_mm, float vel_mms, float acc_mms2, float dec_mms2) {
	if (_pitch == 0) { return; }
	// Calculate the amount of angular rotation in radians to cover the linear distance
	float angle = 2 * PI * (distance_mm / _pitch);
	// Run motor
	float vel_rads = vel_mms * 2 * PI / _pitch;
	float acc_rads2 = acc_mms2 * 2 * PI / _pitch;
	float dec_rads2 = dec_mms2 * 2 * PI / _pitch;

	RotateAngle(angle, vel_rads, acc_rads2, dec_rads2);
}
/** Move the motor in linear stage mode to a target position
 * The motor needs to be configured as linear stage when using the MotorSetup function
 * @param abs_trgPos_mm targer position in millimeters in absolute axis coordinates
 * @param vel_mms target velocity in millimeters per second
 * @param acc_mms2 target acceleration in millimeters per second square
 * @param dec_mms2 target deceleration in millimeters per second square
 */
void TimerStepper::RunLinearStage_abs(float abs_trgPos_mm, float vel_mms, float acc_mms2, float dec_mms2) {

	float _curr_pos = GetPosition_mm_rad();

	if (abs_trgPos_mm > _max_position || abs_trgPos_mm < _min_position) { return; }
	if (_curr_pos > _max_position || _curr_pos < _min_position) { return; }
	
	float _dist = abs_trgPos_mm - _curr_pos;

	RunLinearStage(_dist, vel_mms, acc_mms2, dec_mms2);

}
	
/** Stop motor immediately
 * Stop the timer interrupt that drives the motor without any deceleration
 */
void TimerStepper::StopMotor() {

	if (_pulse_pin_state == HIGH) {
		Step();
		_pulse_pin_state = LOW;
	}
	_control_mode = STP;
	_moving = false;
	_finished = true;
	timers[_motor_id].TimerStop();
}
/** Reset motor position
 * Sets the motor status as not homed and initialises position with a negative number
 */
void TimerStepper::ResetPosition() {
	_motor_homed = false;
	_position = -999;
}

#pragma endregion

// FEEDBACK FUNCTIONS
#pragma region FEEDBACK FUNCTIONS
/** Get the motor's moving status
 * @return TRUE if it's moving. FALSE if stopped
 */
bool TimerStepper::IsMoving() {
	return _moving;
}
/** Get the motor's homing status
 * @return TRUE if it's homed. FALSE if not homed
 */
bool TimerStepper::IsHomed() {
	return _motor_homed;
}
/** Get the motor's current control mode
 * @return wheter the motor is stopped, homing, moving in position control mode, or moving in velocity control mode
 */
byte TimerStepper::GetCurrentControlMode() {
	return _control_mode;
}
/** Get the motor's current step count
 * @return current number of steps
 */
long TimerStepper::GetStepPosition() {
	return _step_position;
}
/** Get the motor's current position in millimeters or radians
 * Will return position in millimeters if the motor is a linear axis, or in radians if it's not
 * @return current motor position in millimeters or radians
 */
float TimerStepper::GetPosition_mm_rad() {
	if (!_motor_homed) { return -999; }
	if (!_axis_isLinear) {
		_position = (float)(_step_position * _step_size_rad);
	}
	else {
		_position = (float)(_step_position * _step_size_mm);
	}
	
	return _position;

}
/** Get the motor's parameters information
 * Return motor parameters: pilse pin, direction pin, enable pin, limit switches pins, pulses per revolution, screw pitch and maximum travel
 * @return string with motor parameters separated by semicolons
 */
String TimerStepper::PassMotorParametersToString() {
	String tempStr = String(_pulse_pin) + ";" + String(_dir_pin) + ";" + String(_ena_pin) + ";" + String(_home_limit_sw_pin) + ";" + String(_far_limit_sw_pin) + ";" +
		String(_ppr) + ";" + String(_pitch, 1) + ";" + String(_max_position, 1) + ";";
	return tempStr;
}

String TimerStepper::PassMotorParametersAndIdToString() {
	String tempStr = String(_motor_id) + ";" + String(_pulse_pin) + ";" + String(_dir_pin) + ";" + String(_ena_pin) + ";" + String(_home_limit_sw_pin) + ";" + 
		String(_far_limit_sw_pin) + ";" + String(_ppr) + ";" + String(_pitch, 1) + ";" + String(_max_position, 1) + ";";		
	return tempStr;
}

#pragma endregion

void TimerStepper::TimerMotor_Run() {
	if (_motor_homed) {
		if ((_step_position > _max_step_position && _dir == 1) || (_step_position < _min_step_position && _dir == -1)) {
			// if (!_vm_stop) {
			// 	StopSpeedMode(_ppr/5);
			// }
		}
	}
	switch (_control_mode) {
		// If Motor in STOP MODE:
    case STP:
      if (_pulse_pin_state == HIGH) {
        Step();
      }
      _moving = false;
      _finished = true;
      _control_mode = STP;
      timers[_motor_id].TimerStop();
      break;
    case HOM:
		  _far_limit_sw_value = digitalRead(_far_limit_sw_pin);
		  _home_limit_sw_value = digitalRead(_home_limit_sw_pin);
      break;

    case POS:
      //Serial.println(_count);
      if (_count < _total_steps) {
        // First Step
        if (_count == 0 && _pulse_pin_state == LOW) {
          unsigned int compare_value = (int)_c0;
          timers[_motor_id].TimerUpdate(compare_value);
          _prev_counter_value = compare_value;
          _rest = 0;	
        }
        // Acceleration Phase
        else if (_count < _acc_end && _pulse_pin_state == LOW) {
          unsigned int compare_value = _prev_counter_value - ((2 * _prev_counter_value + _rest) / (4 * _count + 1));
          timers[_motor_id].TimerUpdate(compare_value);
          _prev_counter_value = compare_value;
          //% = compound remainder: calculate the remainder when one integer is divided by another
          if (_count != (_acc_end - 1)) {
            _rest = (2 * _prev_counter_value + _rest) % (4 * _count + 1);
          }
          else {
            _rest = 0;
          }	
        }
        // Constant Velocity Phase
        else if (_count < _dec_start && _dec_start != _acc_end + 1 && _pulse_pin_state == LOW) {
          unsigned int compare_value = _max_wf_freq / _f_trg;
          timers[_motor_id].TimerUpdate(compare_value);
          _prev_counter_value = compare_value;	
        }
        // Deceleration Phase
        else {
					int num = 2 * _prev_counter_value + _rest;
					int den = 4 * (_count - _total_steps) + 1;
					unsigned int compare_value = _prev_counter_value - (num / den);
					timers[_motor_id].TimerUpdate(compare_value);
					_prev_counter_value = compare_value;
					if (_count != (_total_steps - 1)) {
						_rest = num % den;
					}
					else {
						_rest = 0;
					}
        }
        Step();  // Toggle step pin

      }
      // STOP
      else {       
        if (_pulse_pin_state == HIGH) { Step(); }
        _control_mode = STP;
        _moving = false;
        _finished = true;
        timers[_motor_id].TimerStop();
      }
      break;
  }
}