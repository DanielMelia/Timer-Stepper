#ifndef TimersDue_h
#define TimersDue_h

class TimersDue
{
public:
	static void InitialiseTimers();
	void init();
	void ConfigureTimer(Tc* tc, uint32_t channel, IRQn_Type irq);
	void TimerTrigger(uint32_t rc);

	void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	//TIMSK |= _BV(OCIE1A);
	}
	void attachInterrupt(void (*isr)(), int motor_id) __attribute__((always_inline)) {
	//if(microseconds > 0) setPeriod(microseconds);
	attachInterrupt(isr);
	}

	void (*isrCallback)();        // pointer to the callback function
	//void isrDefaultUnused();

	// Timer Start / Update / Stop / Trigger Functions
	void TimerStart(uint32_t rc);
	void TimerUpdate(uint32_t rc);
	void TimerStop();

	long _max_wf_freq; // = clock_freq / (2 * prescaler);  // the maximum achievable waveform frequency is given by switching twice at clock_freq/prescaler (Hz)

private:
	Tc *_timer;
	uint32_t _channel;
	IRQn_Type _irq;
	int _prescaler;

	static bool _isInit;

	//long _max_wf_freq; // = clock_freq / (2 * prescaler);  // the maximum achievable waveform frequency is given by switching twice at clock_freq/prescaler (Hz)
};

extern TimersDue timers[9];

#endif