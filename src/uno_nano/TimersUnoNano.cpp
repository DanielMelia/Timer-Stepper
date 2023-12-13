#include "Arduino.h"
#include "TimersUnoNano.h"

// ----------------------------------------- MEGA TIMER / COUNTERS -----------------------------------------------------//
// Timer 0: 8-bit. Used for timer functions delay(), millis(), micros()...
// Timer 1: 16-bit. 
// Timer 2: 8-bit. Used for libraries like tone()

// 16-bit max counter value: 65536
// Prescaler:	Min Time (ms):	Max Time (ms):
//    1: 		0.0000625		4.096
//    8:		0.0005			32.768
//   64:		0.004			262.144
//  256:		0.016			1048.576
// 1024:		0.064			4194.304

// //------------------------------------------ CONTROL MODES -------------------------------------------------------------//
// #define STP 0  //STOP mode. Motion not allowed
// #define HOM 1
// #define POS 2  //POSITION mode: the user inputs the amount of rotation and trajectory parameters (vel, acc, dec...)
// #define VEL 3  //VELOCITY mode: or speed control mode. Useful for Joystick control for example. Inputs are velocity and direction

// Microcontroller parameters
#define clock_freq 16000000  // Microprocesor speed in Hz (Most Arduinos have 16MHz. Arduino Due has 84MHz)

//Timer On/Off macros (Timer1 compare interrupt)
#define TIMER1_INTERRUPTS_ON TIMSK1 |= (1 << OCIE1A);    // Set bit. Start Output Compare A Match Interrupt
#define TIMER1_INTERRUPTS_OFF TIMSK1 &= ~(1 << OCIE1A);  // Clear bit. Stop Output Compare A Match Interrupt

TimersUnoNano timers[1];

bool TimersUnoNano::_isInit = false;

void TimersUnoNano::init() {
	if(! _isInit){
		Serial.println("Initializing UNO_NANO timer...");
		TimersUnoNano::InitialiseTimers();
	}
}

void TimersUnoNano::InitialiseTimers() {
    Serial.println("Initialising timer 0...");
	timers[0].ConfigureTimer(1, 64);
}

#pragma region Timer Start / Update / Stop Generic Functions

void TimersUnoNano::TimerStart(uint32_t rc) {

}
void TimersUnoNano::TimerUpdate(uint32_t rc) {
	if (_timer_number == 1) {
		OCR1A = rc;
	}
}
void TimersUnoNano::TimerStop() {
	if (_timer_number == 1) {
		TIMER1_INTERRUPTS_OFF
	}
}

#pragma endregion

void TimersUnoNano::ConfigureTimer(int timerNumber, int ps) {

	if(timerNumber != 1){return;}
	if(ps != 1 && ps != 8 && ps != 64 && ps != 256 && ps != 1024){return;}

	_timer_number = timerNumber;
	_prescaler = ps;
	_max_wf_freq = clock_freq / (2 * _prescaler);

	noInterrupts();

	//Setup Timer1 Interrupt
	TCCR1A = 0;              //Reset entire TCCR1A register (Timer/Counter control register)
	TCCR1B = 0;              //Reset entire TCCR1B register
	TCNT1 = 0;               // Reset Timer/Counter value to 0
	OCR1A = 1000;            // Load compare value (Timer value at which the interrupt will execute)
	TCCR1B |= (1 << WGM12);  // CTC mode

	if (ps == 1) {
		TCCR1B |= (1 << CS10);  //Prescaler: CSx2(0), CSx1(0), CSx0(1) --> 1
	}
	else if (ps == 8) {
		TCCR1B |= (1 << CS11);  //Prescaler: CSx2(0), CSx1(1), CSx0(0) --> 8
	}	
	else if (ps == 64) {
		TCCR1B |= ((1 << CS11) | (1 << CS10));  //Prescaler: CSx2(0), CSx1(1), CSx0(1) --> 64
	}		
	else if (ps == 256) {
		TCCR1B |= (1 << CS12);  //Prescaler: CSx2(1), CSx1(0), CSx0(0) --> 256
	}
	else if (ps == 1024) {
		TCCR1B |= ((1 << CS12) | (1 << CS10));  //Prescaler: CSx2(1), CSx1(0), CSx0(1) --> 1024
	}

	interrupts();

}

void TimersUnoNano::TimerTrigger(uint32_t rc)
{
	TimerUpdate(rc);

	if (_timer_number == 1) {
		TCNT1 = 0;
		TIMER1_INTERRUPTS_ON
	}

}

// void (*TimersUnoNano::isrCallback)() = TimersUnoNano::isrDefaultUnused;
// void TimersUnoNano::isrDefaultUnused()
// {
// }

// ISR Handler Functions
#pragma region ISR Handler Functions

ISR(TIMER1_COMPA_vect)
{
	timers[0].isrCallback();
}

#pragma endregion