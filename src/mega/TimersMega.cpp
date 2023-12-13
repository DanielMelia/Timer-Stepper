#include "Arduino.h"
#include "TimersMega.h"

// ----------------------------------------- MEGA TIMER / COUNTERS -----------------------------------------------------//
// Timer 0: 8-bit. Used for timer functions delay(), millis(), micros()...
// Timer 1: 16-bit. 
// Timer 2: 8-bit. Used for libraries like tone()
// Timer 3: 16-bit
// Timer 4: 16-bit
// Timer 5: 16-bit

// 16-bit max counter value: 65536
// Prescaler:	Min Time (ms):	Max Time (ms):
//    1: 		  0.0000625		       4.096
//    8:		  0.0005			      32.768
//   64:		  0.004			       262.144
//  256:		  0.016			      1048.576
// 1024:		  0.064			      4194.304

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
#define TIMER3_INTERRUPTS_ON TIMSK3 |= (1 << OCIE3A);    // Set bit. Start Output Compare A Match Interrupt
#define TIMER3_INTERRUPTS_OFF TIMSK3 &= ~(1 << OCIE3A);  // Clear bit. Stop Output Compare A Match Interrupt
#define TIMER4_INTERRUPTS_ON TIMSK4 |= (1 << OCIE4A);    // Set bit. Start Output Compare A Match Interrupt
#define TIMER4_INTERRUPTS_OFF TIMSK4 &= ~(1 << OCIE4A);  // Clear bit. Stop Output Compare A Match Interrupt
#define TIMER5_INTERRUPTS_ON TIMSK5 |= (1 << OCIE5A);    // Set bit. Start Output Compare A Match Interrupt
#define TIMER5_INTERRUPTS_OFF TIMSK5 &= ~(1 << OCIE5A);  // Clear bit. Stop Output Compare A Match Interrupt

TimersMega timers[4];

bool TimersMega::_isInit = false;

void TimersMega::init() {
	if(! _isInit){
		Serial.println("Initializing MEGA timers...");
		TimersMega::InitialiseTimers();
	}
}

void TimersMega::InitialiseTimers() {

	int st_count = sizeof(timers) / sizeof(timers[0]);  // sizeof operator returns the size of the object in bytes

	for (int i = 0; i < st_count; i++) {
		if (i == 0) {
      Serial.println("Initialising timer 0...");
			timers[i].ConfigureTimer(1, 64);			
		}
		else if (i == 1) {
      Serial.println("Initialising timer 1...");
			timers[i].ConfigureTimer(3, 64);
		}
		else if (i == 2) {
      Serial.println("Initialising timer 2...");
			timers[i].ConfigureTimer(4, 64);
		}
		else if (i == 3) {
			timers[i].ConfigureTimer(5, 64);
		}
	}

	_isInit = true;

}

#pragma region Timer Start / Update / Stop Generic Functions

void TimersMega::TimerStart(uint32_t rc) {

}
void TimersMega::TimerUpdate(uint32_t rc) {
	if (_timer_number == 1) {
		OCR1A = rc;
	}
	else if (_timer_number == 3) {
		OCR3A = rc;
	}
	else if (_timer_number == 4) {
		OCR4A = rc;
	}
	else if (_timer_number == 5) {
		OCR5A = rc;
	}
}
void TimersMega::TimerStop() {
	if (_timer_number == 1) {
		TIMER1_INTERRUPTS_OFF
	}
	else if (_timer_number == 3) {
		TIMER3_INTERRUPTS_OFF
	}
	else if (_timer_number == 4) {
		TIMER4_INTERRUPTS_OFF
	}
	else if (_timer_number == 5) {
		TIMER5_INTERRUPTS_OFF
	}
}

#pragma endregion

void TimersMega::ConfigureTimer(int timerNumber, int ps) {
	if(timerNumber != 1 && timerNumber != 3 && timerNumber != 4 && timerNumber != 5){return;}
	if(ps != 1 && ps != 8 && ps != 64 && ps != 256 && ps != 1024){return;}

	_timer_number = timerNumber;
	_prescaler = ps;
	_max_wf_freq = clock_freq / (2 * _prescaler);

	noInterrupts();

	if(_timer_number == 1){
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
	}
	else if(_timer_number == 3){
		//Setup Timer3 Interrupt
		TCCR3A = 0;              //Reset entire TCCR1A register (Timer/Counter control register)
		TCCR3B = 0;              //Reset entire TCCR1B register
		TCNT3 = 0;               // Reset Timer/Counter value to 0
		OCR3A = 1000;            // Load compare value (Timer value at which the interrupt will execute)
		TCCR3B |= (1 << WGM32);  // CTC mode

		if (ps == 1) {
			TCCR3B |= (1 << CS30);  //Prescaler: CSx2(0), CSx1(0), CSx0(1) --> 1
		}
		else if (ps == 8) {
			TCCR3B |= (1 << CS31);  //Prescaler: CSx2(0), CSx1(1), CSx0(0) --> 8
		}	
		else if (ps == 64) {
			TCCR3B |= ((1 << CS31) | (1 << CS30));  //Prescaler: CSx2(0), CSx1(1), CSx0(1) --> 64
		}		
		else if (ps == 256) {
			TCCR3B |= (1 << CS32);  //Prescaler: CSx2(1), CSx1(0), CSx0(0) --> 256
		}
		else if (ps == 1024) {
			TCCR3B |= ((1 << CS32) | (1 << CS30));  //Prescaler: CSx2(1), CSx1(0), CSx0(1) --> 1024
		}
	}
	else if(_timer_number == 4){
		//Setup Timer1 Interrupt
		TCCR4A = 0;              //Reset entire TCCR1A register (Timer/Counter control register)
		TCCR4B = 0;              //Reset entire TCCR1B register
		TCNT4 = 0;               // Reset Timer/Counter value to 0
		OCR4A = 1000;            // Load compare value (Timer value at which the interrupt will execute)
		TCCR4B |= (1 << WGM42);  // CTC mode

		if (ps == 1) {
			TCCR4B |= (1 << CS40);  //Prescaler: CSx2(0), CSx1(0), CSx0(1) --> 1
		}
		else if (ps == 8) {
			TCCR4B |= (1 << CS41);  //Prescaler: CSx2(0), CSx1(1), CSx0(0) --> 8
		}	
		else if (ps == 64) {
			TCCR4B |= ((1 << CS41) | (1 << CS40));  //Prescaler: CSx2(0), CSx1(1), CSx0(1) --> 64
		}		
		else if (ps == 256) {
			TCCR4B |= (1 << CS42);  //Prescaler: CSx2(1), CSx1(0), CSx0(0) --> 256
		}
		else if (ps == 1024) {
			TCCR4B |= ((1 << CS42) | (1 << CS40));  //Prescaler: CSx2(1), CSx1(0), CSx0(1) --> 1024
		}		
	}
	else if(_timer_number == 5){
		//Setup Timer1 Interrupt
		TCCR5A = 0;              //Reset entire TCCR1A register (Timer/Counter control register)
		TCCR5B = 0;              //Reset entire TCCR1B register
		TCNT5 = 0;               // Reset Timer/Counter value to 0
		OCR5A = 1000;            // Load compare value (Timer value at which the interrupt will execute)
		TCCR5B |= (1 << WGM52);  // CTC mode

		if (ps == 1) {
			TCCR5B |= (1 << CS50);  //Prescaler: CSx2(0), CSx1(0), CSx0(1) --> 1
		}
		else if (ps == 8) {
			TCCR5B |= (1 << CS51);  //Prescaler: CSx2(0), CSx1(1), CSx0(0) --> 8
		}	
		else if (ps == 64) {
			TCCR5B |= ((1 << CS51) | (1 << CS50));  //Prescaler: CSx2(0), CSx1(1), CSx0(1) --> 64
		}		
		else if (ps == 256) {
			TCCR5B |= (1 << CS52);  //Prescaler: CSx2(1), CSx1(0), CSx0(0) --> 256
		}
		else if (ps == 1024) {
			TCCR5B |= ((1 << CS52) | (1 << CS50));  //Prescaler: CSx2(1), CSx1(0), CSx0(1) --> 1024
		}		
	}	
	interrupts();

}

void TimersMega::TimerTrigger(uint32_t rc){

	TimerUpdate(rc);

	if (_timer_number == 1) {
		TCNT1 = 0;
		TIMER1_INTERRUPTS_ON
	}
	else if (_timer_number == 3) {
		TCNT3 = 0;
		TIMER3_INTERRUPTS_ON
	}
	else if (_timer_number == 4) {
		TCNT4 = 0;
		TIMER4_INTERRUPTS_ON
	}
	else if (_timer_number == 5) {
		TCNT5 = 0;
		TIMER5_INTERRUPTS_ON
	}

}

// ISR Handler Functions
#pragma region ISR Handler Functions

ISR(TIMER1_COMPA_vect)
{
	timers[0].isrCallback();
}

ISR(TIMER3_COMPA_vect)
{
	timers[1].isrCallback();

}

ISR(TIMER4_COMPA_vect)
{
	timers[2].isrCallback();
}

ISR(TIMER5_COMPA_vect)
{
	timers[3].isrCallback();

}

#pragma endregion