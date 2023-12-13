#ifndef TimersMega_h
#define TimersMega_h

class TimersMega
{
public:
	static void InitialiseTimers();
	void init();
	void ConfigureTimer(int timerNumber, int ps);
	void TimerTrigger(uint32_t rc);

	void attachInterrupt(void (*isr)()) __attribute__((always_inline)) {
	isrCallback = isr;
	}
	void attachInterrupt(void (*isr)(), int motor_id) __attribute__((always_inline)) {
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
	unsigned int _timer_number;
	int _prescaler;

	static bool _isInit;

	//long _max_wf_freq; // = clock_freq / (2 * prescaler);  // the maximum achievable waveform frequency is given by switching twice at clock_freq/prescaler (Hz)
};

extern TimersMega timers[4];

#endif