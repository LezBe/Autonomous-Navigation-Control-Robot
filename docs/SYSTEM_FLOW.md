# Embedded System Flow

The original system-flow diagram maps the major execution paths of the final integration firmware.

## Startup Sequence

The main routine:

1. Enables the FPU and lazy stacking.
2. Sets the system clock to 40 MHz.
3. Initializes UART0 and UART1.
4. Initializes PWM, the right IR ADC, the front IR ADC, and the 50 ms timer.
5. Initializes the line sensor and places the motors in an idle state.
6. Creates TI-RTOS Hwi objects for timer, ADC, and UART events.
7. Creates the PID/right-turn Swi.
8. Constructs the background ping-pong-buffer Task.
9. Starts the RTOS scheduler and waits for interrupt-driven activity.

## Command Path

Three-character commands received through the UART interfaces are passed to the command lookup routine. Supported behaviors include:

- stop
- U-turn
- forward
- right turn

These commands invoke the corresponding motor and timer functions.

## Sensor / Control Path

The timer initiates ADC activity. The front and right sensor interrupt handlers use the resulting distance measurements to choose between obstacle handling, forward motion, right-turn behavior, and the wall-following control routine.

## Line / Logging Path

The line sensor drives the event sequence used for:

- starting data logging after the first thin line
- stopping / flushing logging after the second thin line
- stopping the robot at the thick line
- triggering the final LED flashing behavior

The ping-pong buffer allows data collection and UART transmission to be separated from the main real-time control path.
