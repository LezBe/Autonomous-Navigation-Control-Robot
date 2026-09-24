# Technical Notes

This document highlights implementation details from the final integration firmware.

## Controller Timing

```c
#define SAMPLE_PERIOD_MS 50
```

This corresponds to a **20 Hz** control update rate.

## Wall Distance

Right-side IR sensor:

```text
PE1 / AIN2
```

Front IR sensor:

```text
PE2 / AIN1
```

ADC resolution is 12 bits with a 3.3 V reference.

The right-side sensor reading is converted to distance with:

```c
#define DIST_CM_FVOLT(v) ((5.0685f*(v)*(v)) - (23.329f*(v)) + 31.152f)
```

Target distance:

```c
#define DIST_TARGET_CM 10.0f
```

## Control Gains

```c
Kp = 1.0f;
Ki = 0.0f;
Kd = 0.0f;
```

Although the firmware implements proportional, integral, and derivative terms, the final tuned configuration effectively uses proportional control.

## Motor PWM

```c
#define PWM_FREQ_HZ 20000
#define BASE_DUTY_PC 92.0f
#define DUTY_MIN_PC  25.0f
#define DUTY_MAX_PC  99.0f
```

Direction pins:

```text
Left motor:  PD2
Right motor: PD3
```

## Communications

```text
UART0: 115200 baud (USB)
UART1: 9600 baud (Bluetooth)
```

The Bluetooth interface receives three-character commands and transmits logged wall-following error data.

## Logging

```c
int16_t PingPongBuf[2][20];
```

The ping-pong organization allows one buffer to collect samples while another completed buffer can be transmitted.

## RTOS Organization

- **Hwi:** ADC, timer, and UART interrupt handlers
- **Swi:** deferred wall-following / right-turn computation
- **Task:** background logging / transmission support

## Line-State Behavior

```text
Before first line
Logging
After second line
Thick-line / shutdown
```

Thin-line detections control logging state. A thick-line event stops the motors and starts a 60-second LED flashing sequence.

## Hardware Identified in the Final Report

- EK-TM4C123GXL Tiva C
- HC-05 Bluetooth module
- Sharp GP2Y0A41SK0F distance sensors
- DRV8835 motor driver
- Pololu SPDT rocker switch
- Pololu S7V7F5 regulator
- Pololu QTR-1RC reflective sensor

## Portfolio Note

Generated Code Composer Studio build files and caches are intentionally excluded from this cleaned portfolio repository.
