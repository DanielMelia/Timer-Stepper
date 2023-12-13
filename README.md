# Timer-Stepper
Library to control stepper motors using hardware timers on several Arduino boards.
Compatible with Arduino Uno, Nano, Mega and Due boards. It uses one hardware interrupt
to control each motor.

This library contains function to create trapezoidal acceleration/deceleration profiles,
run steppers in motor-only mode or linear stage mode, position and velocity control and homing.

It also includes functions to configure the motor pins, timers, limit switches or get feedback.

Needs to be used together with another library to handle configuration and control of timer interrupts
as well as Interrupt Service Routines
 - TimersDue.h on ARDUINO DUE to control a maximum of 9 motors
 - TimersMega.h on ARDUINO MEGA to control a maximum of 4 motors
 - TimersUnoNano.h on ARDUINO UNO/NANO to control a single motor

On the Due board it uses 9 32-bit counters.
On the mega it uses the 4 16-bit counters (timers 1, 3, 4 and 5)
On the Uno and Nano uses timer 1 (16-bit)
