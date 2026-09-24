# Autonomous Navigation & Control Robot

Autonomous wall-following robot developed on a **TI TM4C123GXL (Tiva C) microcontroller** using **TI-RTOS**, infrared distance sensing, PWM motor control, UART/Bluetooth communication, and real-time event handling.

## Project Overview

This project was developed as a team-based embedded systems integration project. The robot autonomously follows a wall while maintaining a target distance, responds to front obstacles, detects course markings, logs control error data, and performs a controlled shutdown sequence.

My work focused on system integration, embedded control, event handling, and data collection.

### Key Results

- Maintained approximately **±1 cm wall-tracking accuracy**
- Executed the control loop every **50 ms**
- Used **20 kHz PWM** for left/right motor speed control
- Logged wall-following error through a **ping-pong buffer**
- Used line events to start/stop logging and trigger the final shutdown sequence
- Coordinated control and logging using **TI-RTOS Hwi, Swi, and Task objects**

## My Contributions

- Integrated system-level event handling using TI-RTOS
- Implemented the wall-following control logic using IR distance feedback
- Controlled left and right motors through PWM duty-cycle adjustment
- Integrated ADC acquisition for front and right-side IR sensors
- Implemented UART communication for USB and Bluetooth interfaces
- Managed data-logging timing using a ping-pong buffering scheme
- Integrated line-sensor events with LED indicators and robot state transitions
- Implemented the one-minute flashing shutdown routine after the final course event
- Assisted with system integration and debugging of the final robot

## Hardware

| Component | Function |
|---|---|
| TI TM4C123GXL / Tiva C | Main embedded controller |
| Sharp GP2Y0A41SK0F distance sensors | Front and right wall-distance sensing |
| DRV8835 motor driver + DC motors | Differential-drive motion |
| Pololu QTR-1RC reflective sensor | Detects black course markings |
| On-board RGB LED | Visual state indication |
| HC-05 Bluetooth module | Wireless UART commands and data logging |
| Pololu S7V7F5 regulator | Stable regulated supply |
| Buzzer | Audible feedback |

## Embedded Architecture

```text
50 ms Timer
    |
    +--> Front IR ADC
    |
    +--> Right IR ADC
    |
    +--> TI-RTOS Swi
            |
            +--> Wall-follow control
            +--> Right-turn logic
                    |
                    v
             Left / Right PWM
                    |
                    v
                  Motors
```

The periodic timer also participates in line-event timing, LED behavior, data-buffer transmission, and the shutdown state machine.

## Wall-Following Control

The right-side IR sensor is sampled through the TM4C123's ADC and converted from voltage to an estimated distance using a calibration curve.

```c
distance_cm = (5.0685 * V * V) - (23.329 * V) + 31.152;
```

The target wall distance is **10 cm**.

The firmware contains a PID-capable control framework:

```text
error = measured_distance - target_distance

adjust =
    Kp * error
  + Ki * integral(error)
  + Kd * derivative(error)
```

The final tuning stored in the project was:

```text
Kp = 1.0
Ki = 0.0
Kd = 0.0
```

Therefore, the final configuration operated as **proportional control within a PID-capable implementation**.

The controller adjusts the motor commands around a nominal **92% duty cycle**, constrained to approximately **25%–99%**.

## Real-Time Event Handling

TI-RTOS separates time-critical acquisition from higher-level control work:

- **Hardware interrupt (Hwi):** periodic timer, ADC completion, UART
- **Software interrupt (Swi):** wall-following / right-turn computation
- **Task:** background ping-pong-buffer transmission

The main control timer runs every **50 ms**, providing a 20 Hz control update rate.

## Course / Line Events

| Event | System response |
|---|---|
| First thin line | Enables error-data logging and indicates the event |
| Second thin line | Stops logging and flushes remaining data |
| Thick line | Stops the motors and begins the final shutdown sequence |
| Shutdown sequence | Red LED flashes for approximately one minute |

## Data Logging

Wall-following error samples are stored in a **2 × 20 ping-pong buffer**. While one buffer is being filled, another completed buffer can be transmitted through Bluetooth UART.

Logged frames are labeled:

```text
Team 16:
```

## Communication

- **UART0 / USB:** 115200 baud
- **UART1 / Bluetooth:** 9600 baud
- Three-character commands trigger forward motion, right turns, U-turns, and stopping.

## Documentation

Documentation summaries derived from the original final report and embedded code/process diagram are included in this repository:

- [Final Report Summary](docs/PROJECT_REPORT_SUMMARY.md)
- [Embedded System Flow](docs/SYSTEM_FLOW.md)

The original project documentation covers component specifications, pin assignments, APIs, RTOS organization, Bluetooth/USB command interfaces, the circuit diagram, and final integration behavior.

## Repository Structure

```text
.
├── README.md
├── TECHNICAL_NOTES.md
├── .gitignore
├── docs/
│   ├── PROJECT_REPORT_SUMMARY.md
│   └── SYSTEM_FLOW.md
└── firmware/
    └── Integration_Team16/
        ├── empty_min.c
        ├── empty_min.cfg
        ├── Board.h
        ├── EK_TM4C123GXL.cmd
        ├── .ccsproject
        ├── .project
        └── targetConfigs/
```

## Development Environment

- **Language:** C
- **MCU:** TI TM4C123GH6PM / EK-TM4C123GXL LaunchPad
- **IDE:** Code Composer Studio
- **RTOS:** TI-RTOS
- **Libraries:** TI TivaWare / DriverLib

## Skills Demonstrated

Embedded C · TI-RTOS · Real-Time Systems · Closed-Loop Control · PWM · ADC · UART · Interrupts · Sensor Integration · Motor Control · Data Logging · Hardware/Software Integration · Embedded Debugging

## Notes

Generated Code Composer Studio build outputs and IDE cache files are intentionally excluded from the portfolio repository.
