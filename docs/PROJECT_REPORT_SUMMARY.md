# Final Report Summary

This file summarizes the original ECE 4437 Team 16 final report that accompanied the robot integration project.

## System Purpose

The final integration combined UART and Bluetooth communication, PWM motor control, distance sensing, wall-following control, line detection, and TI-RTOS scheduling into an autonomous robot capable of navigating a maze without continuous user control.

## Hardware Identified in the Final Report

- EK-TM4C123GXL Tiva C microcontroller
- HC-05 Bluetooth module
- Two Sharp GP2Y0A41SK0F distance sensors
- DRV8835 motor driver
- Pololu SPDT rocker switch
- Pololu S7V7F5 voltage regulator
- Pololu QTR-1RC reflective sensor

## Pin Assignments

- PB0: UART1 RX
- PB1: UART1 TX
- PB6: buzzer
- PC4: left line sensor
- PC5: right line sensor
- PE1: right-side distance sensor
- PE2: front distance sensor
- PF1: red LED
- PF2: blue LED
- PF3: green LED

## Software / RTOS Organization

The final report describes TI-RTOS objects used to separate time-critical and background work:

- Hwi objects for UART0, UART1, ADC0SS2, ADC0SS3, and Timer1A
- A Swi for PID / right-turn processing
- A background Task for ping-pong-buffer transmission
- BIOS_start() to launch the RTOS scheduler

## Integration Behavior

- Timer1A provides the recurring control timing.
- The front sensor is used for obstacle / U-turn decisions.
- The right sensor provides wall-distance information.
- Wall-following correction adjusts motor PWM duty cycles around the target distance.
- The first thin line starts PID-error logging.
- The second thin line stops logging and flushes remaining samples.
- A thick line stops the motors and starts the final red-LED flashing sequence.
- Bluetooth and USB UART interfaces support command input and debugging / output.

## Original Documentation

The original submitted report also contains API notes, hardware/software procedures, Bluetooth and Tera Term screenshots, a circuit diagram, and the general code/process flow diagram.
