#include "Arduino.h"
#include "TimersDue.h"

TimersDue timers[9];

bool TimersDue::_isInit = false;

void TimersDue::init() {
	if(! _isInit){
		Serial.println("Initializing DUE timers...");
		TimersDue::InitialiseTimers();
	}
	
}

void TimersDue::InitialiseTimers() {

	int st_count = sizeof(timers) / sizeof(timers[0]);  // sizeof operator returns the size of the object in bytes
	for (int i = 0; i < st_count; i++) {
		if (i == 0) {
      Serial.println("Initialising timer 0...");
			timers[i].ConfigureTimer(TC0, 0, TC0_IRQn);			
		}
		else if (i == 1) {
      Serial.println("Initialising timer 1...");
			timers[i].ConfigureTimer(TC0, 1, TC1_IRQn);
		}
		else if (i == 2) {
      Serial.println("Initialising timer 2...");
			timers[i].ConfigureTimer(TC0, 2, TC2_IRQn);
		}
		else if (i == 3) {
			timers[i].ConfigureTimer(TC1, 0, TC3_IRQn);
		}
		else if (i == 4) {
			timers[i].ConfigureTimer(TC1, 1, TC4_IRQn);
		}
		else if (i == 5) {
			timers[i].ConfigureTimer(TC1, 2, TC5_IRQn);
		}
		else if (i == 6) {
			timers[i].ConfigureTimer(TC2, 0, TC6_IRQn);
		}
		else if (i == 7) {
			timers[i].ConfigureTimer(TC2, 1, TC7_IRQn);
		}
		else if (i == 8) {
			timers[i].ConfigureTimer(TC2, 2, TC8_IRQn);
		}
	}
	_isInit = true;
}

#pragma region Timer Start / Update / Stop Generic Functions

void TimersDue::TimerStart(uint32_t rc) {
  Serial.println("Inside timer start...");
	// Tell the Power Management Controller to disable the write protection of the (Timer/Counter) registers:
	pmc_set_writeprotect(false);
	// Enable clock for the timer
	pmc_enable_periph_clk(_irq);

	// Set up the Timer in waveform mode which creates a PWM in UP mode with automatic trigger on RC Compare
	// and sets it up with the determined internal clock as clock input.

	// A 32-bit unsigned integer has a range of 0 - 4294967295
	//PRESCALERS:
	// - TIMER_CLOCK1 - MCK/2   - 42MHz
	// - TIMER_CLOCK2 - MCK/8   - 10.5MHz		--> 0.09us to 409 seconds timer period
	// - TIMER_CLOCK3 - MCK/32  - 2.652MHz
	// - TIMER_CLOCK4 - MCK/128 - 656.25KHz
	// - TIMER_CLOCK5 - SLCK    - 32KHz
	// *MCK - Master Clock (84MHz)
	TC_Configure(_timer, _channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2);
	//freq --> desired blinking frequency (in Hz)
	//uint32_t rc = VARIANT_MCK / _prescaler / freq;
	//Creates a wafeform that goes high at RA and low at RC
	//TC_SetRA(tc, channel, rc >> 1); // 50% duty cycle square wave
	TC_SetRC(_timer, _channel, rc);
	TC_Start(_timer, _channel);
	//interrups occurs only when counter reaches RC:
	_timer->TC_CHANNEL[_channel].TC_IER = TC_IER_CPCS;
	_timer->TC_CHANNEL[_channel].TC_IDR = ~TC_IER_CPCS;

	NVIC_EnableIRQ(_irq);
}
void TimersDue::TimerUpdate(uint32_t rc) {
	//uint32_t rc = VARIANT_MCK / _prescaler / freq;
	TC_SetRC(_timer, _channel, rc);
	TC_Start(_timer, _channel);
	NVIC_EnableIRQ(_irq);
}
void TimersDue::TimerStop() {
	NVIC_DisableIRQ(_irq);
	TC_Stop(_timer, _channel);
}

#pragma endregion

void TimersDue::ConfigureTimer(Tc* tc, uint32_t channel, IRQn_Type irq) {
	_timer = tc;
	_channel = channel;
	_irq = irq;

	_prescaler = 8;
	_max_wf_freq = VARIANT_MCK / (2 * _prescaler);
  //Serial.print("Timer confing done! ");Serial.print(String(irq));Serial.print(" - ");Serial.print(channel);
}

void TimersDue::TimerTrigger(uint32_t rc)
{

	//_count = 0;
	//_moving = true;  // Indicate that motion started
  Serial.println("Inside timer trigger...");
	TimerStart(rc);

}

//void (*TimersDue::isrCallback)() = TimersDue::isrDefaultUnused;
// void TimersDue::isrDefaultUnused()
// {
// }

// ISR Handler Functions
#pragma region ISR Handler Functions
void TC0_Handler()
{
  //Serial.println("I");
	TC_GetStatus(TC0, 0);
	timers[0].isrCallback();
}
void TC1_Handler()
{
	TC_GetStatus(TC0, 1);
	timers[1].isrCallback();
}
void TC2_Handler()
{
	TC_GetStatus(TC0, 2);
	//timers_due[2].callback();
}
void TC3_Handler()
{
	TC_GetStatus(TC1, 0);
	//timers_due[3].callback();
}
void TC4_Handler()
{
	TC_GetStatus(TC1, 1);
	//timers_due[4].callback();
}
void TC5_Handler()
{
	TC_GetStatus(TC1, 2);
	//timers_due[5].callback();
}
void TC6_Handler()
{
	TC_GetStatus(TC2, 0);
	//timers_due[6].callback();
}
void TC7_Handler()
{
	TC_GetStatus(TC2, 1);
	//timers_due[7].callback();
}
void TC8_Handler()
{
	TC_GetStatus(TC2, 2);
	//timers_due[8].callback();
}
#pragma endregion