//*****************************************************************************
//
// uart_echo.c - Example for reading data from and writing data to the UART in
// an interrupt driven fashion.
//
// Copyright (c) 2012-2016 Texas Instruments Incorporated. All rights reserved.
// Software License Agreement
//
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
// This is part of revision 2.1.3.156 of the EK-TM4C123GXL Firmware Package.
//
//*****************************************************************************
// Notes (Midterm 1):
//
// UART Notes:
// https://www.analog.com/en/resources/analog-dialogue/articles/uart-a-hardware-communication-protocol.html
//
// Refer to W2C2 slides for Bluetooth and BLE.
//
// Volatile keyword is a type qualifier that tells the compiler that the value
// of a variable may change at any time, in ways that are not detectable by the
// compiler itself. This prevents the compiler from performing certain
// optimizations that might otherwise lead to incorrect behavior.
//
// Refer to W5C1 slides for ADC
// ADC SAR: https://circuitdigest.com/article/how-does-successive-approximation-sar-adc-work-and-where-is-it-best-used
//
// Notes (Midterm 2):
//
// PID:
// * Kp: adds higher or lower oscillatory behavior to error. Can settle to wrong value (introduces offset)
// * Ki: accumulates errors to remind system of original settling point (removes offset)
// * Kd: Can decrease settling time (time it takes to reach ideal value).
// * Algorithm: Increase Kp until oscillatory, then half its value. Increase Ki to eliminate steady-state error,
//   and then increase Kd to decrease settling time. (All parameters are zero initially).
// * Video: https://www.youtube.com/watch?v=tFVAaUcOm4I&t=29s
// * Only look at PID code for exam 2
//
// Ping-Pong Buffer:
// * One buffer (ping) receives first, once filled it starts to transmit. At the same time, the other buffer (pong) starts
//   to receive. The two flip back and forth between these two states.
// * Definition: uint8_t PingPongBuf [2][20];
// * Context: Can be used with Bluetooth for data logging (showing the error samples) purposes.
//
//
// Questions:
// * PWMOutputState()?
// * PH pin? (for direction pins on motors)
// * Pre-emption for tasks?
// * Semaphores?
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "string.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
// TI-RTOS / SYS BIOS includes
#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/hal/Hwi.h>
#include "driverlib/rom_map.h"
#include "driverlib/fpu.h"
#include "ctype.h"
// Use only the LEFT line sensor for edge or line detection
#define LINE_SENSOR_MASK (LINE_SENSOR_LEFT_PIN) // PC4 only
// Line and race state machine phases
enum { LP_BEFORE1 = 0, LP_LOGGING = 1, LP_AFTER2 = 2, LP_THICK = 3 };
// Current line phase state
volatile uint8_t g_linePhase = LP_BEFORE1;
// 50 ms ticks while line is held low (active low sensors)
volatile uint16_t g_lineHoldTicks = 0; // how long a line is held low in ticks
volatile uint16_t g_flashTicks = 0;    // used for 60 second flash timing, 60s / 50ms = 1200
// Red LED flasher for final 60 seconds (60s / 0.05s = 1200 ticks)
volatile uint8_t g_redOn = 0;
// Thresholds in units of 50 ms ticks
#define THIN_MIN_TICKS 0.5  // about 50 to 100 ms
#define THIN_MAX_TICKS 1.85
#define THICK_MIN_TICKS 2   // about 150 ms means thick line
// Logging decimation flag, toggles every 50 ms
volatile uint8_t g_logDecim = 0; // 0 or 1 toggles each 50 ms
// PID timer definitions
#define PID_TIMER_PERIPH SYSCTL_PERIPH_TIMER1
#define PID_TIMER_BASE   TIMER1_BASE
#define PID_TIMER_INT    INT_TIMER1A
// Global flag to enable or disable data logging
volatile bool g_logEnable = false; // false means no Bluetooth logger output
// Flag to enable or disable line safety logic
volatile bool g_lineSafetyEnable = true;
// Number of consecutive hits needed to declare a line trip
#define LINE_TRIP_COUNT 3 // need 3 consecutive hits (about 150 ms at 50 ms each)
volatile uint8_t g_lineTrip = 0; // consecutive trip count
// COMMUNICATION SETTINGS
// UART baud rates
#define USB_BAUD 115200 // Fast speed for USB cable to computer
#define BT_BAUD  9600   // Slower speed for Bluetooth
// LED INDICATOR PINS (Port F)
// These LEDs show how well the robot is following the wall
#define LED_RED   GPIO_PIN_1 // Pin PF1 - Lights when too far from target
#define LED_BLUE  GPIO_PIN_2 // Pin PF2 - Currently unused
#define LED_GREEN GPIO_PIN_3 // Pin PF3 - Lights when distance is perfect
#define LED_ALL   (LED_RED | LED_BLUE | LED_GREEN) // All three LEDs combined
// MOTOR PWM CONFIGURATION
// PWM frequency for motor control, 20 kHz
// This makes the motors run smoothly and usually keeps the sound high enough to be less annoying
#define PWM_FREQ_HZ 20000
// MOTOR DIRECTION CONTROL PINS (Port D)
// These pins control whether motors spin forward or backward
#define LEFT_DIR_PORT_BASE  GPIO_PORTD_BASE // Port D base address
#define RIGHT_DIR_PORT_BASE GPIO_PORTD_BASE // Port D base address
#define LEFT_DIR_PIN        GPIO_PIN_2      // PD2 connects to left motor driver PH pin
#define RIGHT_DIR_PIN       GPIO_PIN_3      // PD3 connects to right motor driver PH pin
// When pin is HIGH (3.3V): motor spins forward
// When pin is LOW (0V): motor spins backward
// LINE SENSORS (Port C)
// Digital line sensors to detect table edge or line.
// Using PC4 = left line sensor, PC5 = right line sensor.
#define LINE_SENSOR_PORT_BASE   GPIO_PORTC_BASE
#define LINE_SENSOR_LEFT_PIN    GPIO_PIN_4
#define LINE_SENSOR_RIGHT_PIN   GPIO_PIN_5
// INFRARED DISTANCE SENSOR CONFIGURATION (Port E)
// Right side IR sensor that measures distance to the wall
#define RIGHT_SENSOR_PORT_BASE  GPIO_PORTE_BASE // Port E base address
#define RIGHT_SENSOR_PIN        GPIO_PIN_1      // PE1 physical pin
#define RIGHT_SENSOR_ADC_CH     ADC_CTL_CH2     // Analog channel 2 (AIN2)
// This sensor outputs 0 to 3.3V voltage that changes as objects move closer or farther
// FRONT SENSOR CONFIGURATION
#define FRONT_SENSOR_PORT_BASE  GPIO_PORTE_BASE // Port E base
#define FRONT_SENSOR_PIN        GPIO_PIN_2      // PE2 physical pin
#define FRONT_SENSOR_ADC_CH     ADC_CTL_CH1     // Analog channel 1 (AIN1)
// ADC SETTINGS
// The ADC converts analog voltage (0 to 3.3V) to digital numbers
#define VREF      3.3f      // Maximum voltage the ADC can measure
#define ADC_FULL  4096.0f   // 12 bit ADC means 2^12 = 4096 values (0 to 4095)
// Example: 1.65V would convert to approximately 2048 count
// SENSOR CALIBRATION FORMULA
// This formula converts sensor voltage to approximate distance in centimeters
// It uses a quadratic equation that approximates the sensor behavior
#define DIST_CM_FVOLT(v) ( (5.0685f*(v)*(v)) - (23.329f*(v)) + 31.152f )
// Input: voltage v from 0 to 3.3 volts
// Output: distance in centimeters
// Example: 2.5V gives around 10 cm distance
// PID CONTROL TIMING
// How often the PID controller runs in milliseconds
#define SAMPLE_PERIOD_MS 50
// 50 ms is 0.05 seconds which is 20 updates per second
// PID TUNING PARAMETERS
// The target distance the robot tries to maintain from the wall
#define DIST_TARGET_CM 10.0f // target distance in cm
// PID gains control how aggressively the robot corrects errors
// Kp (Proportional): Reacts to current error
// Example: I am 2 cm too far right now, so I need to turn this much
// Higher Kp gives stronger immediate response, but can overshoot
float Kp = 1.0f;
// Ki (Integral): Fixes persistent errors over time
// It accumulates error over time to remove steady state offset
// Higher Ki removes offsets faster, but can cause oscillation
// float Ki = 0.10f;
float Ki = 0.0f;
// Kd (Derivative): Responds to how fast the error is changing
// It predicts future error and helps smooth the response
// Higher Kd reduces overshoot and can make motion smoother
float Kd = 0.0f;
// MOTOR SPEED LIMITS
#define BASE_DUTY_PC 92.0f // Normal cruising speed in percent
#define DUTY_MIN_PC  25.0f // Minimum allowed duty to avoid stalling
#define DUTY_MAX_PC  99.0f // Maximum allowed duty to protect motors
// PID adjustments happen around BASE_DUTY_PC
// Example: If adjustment is +3 percent:
// Left motor: 92 - 3 = 89 percent
// Right motor: 92 + 3 = 95 percent
// GLOBAL VARIABLES
// Command buffer for receiving 3 character commands via Bluetooth
uint8_t g_cmdBuf[4]; // Stores 3 characters plus null terminator if needed
uint8_t g_cmdLen = 0; // Current number of characters received
// PWM motor control variables
uint32_t g_pwmLoad = 0;           // PWM timer period value (calculated at startup)
volatile float g_leftDuty = 0.0f; // Left motor duty cycle in percent
volatile float g_rightDuty = 0.0f;// Right motor duty cycle in percent
// PID CONTROLLER STATE
volatile float g_errPrev = 0.0f; // Previous distance error for derivative term
volatile float g_errInt = 0.0f;  // Accumulated error over time for integral term
// These must persist between PID updates to calculate rate of change
// SENSOR READINGS
volatile uint32_t g_adcRaw = 0; // Raw ADC value (0 to 4095)
volatile float g_volts = 0.0f;  // Converted voltage in volts
volatile float g_distCm = 0.0f; // Converted distance in centimeters
// FLAGS
bool uturnFlag = false; // Flag for u turn behavior
// Ping pong buffer for error logging
#define PINGPONG_BUF_ROWS 2
#define PINGPONG_BUF_COLS 20
int16_t PingPongBuf[PINGPONG_BUF_ROWS][PINGPONG_BUF_COLS];
volatile uint8_t ppIndex = 0;           // index into active buffer
volatile uint8_t ppActive = 0;          // 0 for ping, 1 for pong
volatile bool ppReady[PINGPONG_BUF_ROWS] = { false, false }; // buffer ready flags
// TI RTOS objects
Swi_Handle swiPID;            // Swi that runs PID and right turn logic
Task_Struct loggerTaskStruct; // Task structure for sending ping pong data
Char loggerTaskStack[1024];   // Stack space for logger task
// FUNCTION DECLARATIONS
// Interrupt handlers (used as Hwi in TI RTOS)
void UART0IntHandler(void);   // USB receives data
void UART1IntHandler(void);   // Bluetooth receives data
void ADC0SS3_Handler(void);   // ADC finishes right sensor reading
void ADC0SS2_Handler(void);   // ADC finishes front sensor reading
void Timer1A_Handler(void);   // Timer expires every 50 ms
// Swi and Task functions
void swiPIDFxn(UArg arg0, UArg arg1);           // Swi to handle PID work
void loggerTaskFxn(UArg arg0, UArg arg1);       // Task to send ping pong data
// Communication functions
void UART0_Send(uint8_t *buf, uint32_t n);      // Send data to USB
void UART1_Send(uint8_t *buf, uint32_t n);      // Send data to Bluetooth
void SEND_BOTH(const char *s);                  // Send string to both USB and Bluetooth
// Motor control functions
void PWM_Init(void);                            // Set up PWM hardware
void Motors_SetDuty(float leftPc, float rightPc);// Set motor speeds in percent
void Motors_SetForward(void);                   // Set both motors forward
void Motors_SetUTurn(void);                     // Set motors into opposite directions
void Motors_AllStop(void);                      // Emergency stop with 0 percent duty
void Motors_EnableOutputs(bool on);             // Enable or disable PWM outputs
// PID
void PID_Function(float g_distCm);              // PID function using current distance
// Sensor and timing setup
void ADC_Init_RightSensor(void);                // Configure right IR sensor ADC
void ADC_Init_FrontSensor(void);                // Configure front sensor ADC
void Timer1A_Init_50ms(void);                   // Configure 50 ms PID timer
// Command processing
void do_cmd_lookup_and_run(uint8_t *g_cmdBuf);
void forward(void);                             // Start forward motion and PID
void right(void);                               // Start right turn motion
void uturn(void);                               // Start u turn motion
void stop(void);                                // Stop motion and PID
// Ping pong helper functions
bool flag1 = 1;
void DataLog_Append100ms(float err);
void PingPong_SendReadyBuffers(void);
void DataLog_FlushPartial(void);

// pingpong work please variables
volatile bool Full = false;

// Notes (Hz)
#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_G5 784
// Simple structure for a musical note
typedef struct { uint16_t freqHz; uint16_t ms; } Note;

#define NOTE_C4 262
#define NOTE_D4 294
#define NOTE_E4 330
#define NOTE_F4 349
#define NOTE_G4 392
#define NOTE_A4 440
#define NOTE_B4 494
#define NOTE_C5 523
#define NOTE_D5 587
#define NOTE_E5 659
#define NOTE_G5 784
// extra for Nokia style melody
#define NOTE_CS4 277
#define NOTE_FS4 370
#define NOTE_GS4 415
#define NOTE_CS5 554
// Nokia tune note sequence table
static const Note NOKIA_TUNE[] = {
    // Phrase 1
    {NOTE_E5,  120}, {NOTE_D5,  120}, {NOTE_FS4, 240}, {NOTE_GS4, 240}, {0, 120},
    {NOTE_CS5, 120}, {NOTE_B4,  120}, {NOTE_D4,  240}, {NOTE_E4,  240}, {0, 120},
    {NOTE_B4,  120}, {NOTE_A4,  120}, {NOTE_CS4, 240}, {NOTE_E4,  240}, {0, 120},
    {NOTE_A4,  360}, {0, 120},
    // Phrase 2 (repeat higher)
    {NOTE_E5,  120}, {NOTE_D5,  120}, {NOTE_FS4, 240}, {NOTE_GS4, 240}, {0, 120},
    {NOTE_CS5, 120}, {NOTE_B4,  120}, {NOTE_D4,  240}, {NOTE_E4,  240}, {0, 120},
    {NOTE_B4,  120}, {NOTE_A4,  120}, {NOTE_CS4, 240}, {NOTE_E4,  240}, {0, 120},
    {NOTE_A4,  360}, {0, 300},
};
#define NOKIA_TUNE_LEN (sizeof(NOKIA_TUNE)/sizeof(NOKIA_TUNE[0]))
// Globals for the buzzer player
static const Note *g_song = NULL;
static uint32_t g_songLen = 0;
static uint32_t g_songIdx = 0;
static bool g_songLoop = false;
// Forward declarations for buzzer control
void Buzzer_Init(void);
void Buzzer_PlaySong(const Note *song, uint32_t len, bool loop);
void Buzzer_Stop(void);
static void Buzzer_SetFrequency(uint32_t freqHz);
void Timer3A_BeepScheduler(void);
// Structure to hold command information
typedef struct {
    char code[4];          // 3 letter command and null terminator
    void (*function)(void);// pointer to function that runs the command
} cmd_entry;
// Table of all available commands
cmd_entry CMD_TABLE[] = {
    { "FWD", forward },
    { "RGT", right },
    { "UTU", uturn },
    { "STP", stop }
};
// UTILITY FUNCTIONS
// CLAMP: Forces a value to stay within minimum and maximum bounds
// This prevents motors from getting dangerous values
// Example: fclamp(150, 25, 99) returns 99, cannot exceed max
// fclamp(10, 25, 99) returns 25, cannot go below min
float fclamp(float x, float lo, float hi) {
    if (x < lo) {
        return lo; // Too low, return minimum
    }
    if (x > hi) {
        return hi; // Too high, return maximum
    }
    return x; // Within range, return unchanged
}
// Append one sample to the ping pong log every 100 ms
void DataLog_Append100ms(float err)
{
    // Only log when logging is enabled
    if (!g_logEnable) return;
    // Decimate 50 ms down to 100 ms (log every other PID tick)
    g_logDecim ^= 1;
    if (g_logDecim == 0) return;
    //SEND_BOTH("Err Data Logged\r\n");
    // Convert error to fixed point tenths, for example 4.3 becomes 43
    int16_t sample = (int16_t)(err * 10.0f);
    // Store into current ping or pong buffer
    PingPongBuf[ppActive][ppIndex++] = sample;
    // If this buffer is full, mark it ready and flip to the other one
    if (ppIndex >= PINGPONG_BUF_COLS) {
        ppReady[ppActive] = true;      // this buffer can now be sent
        ppActive ^= 1;                 // switch between 0 and 1
        ppIndex = 0;                   // reset index for new buffer
        // Debug message to show that a full buffer happened
        //SEND_BOTH("buffer full\r\n");
        //PingPong_SendReadyBuffers();
        Full = true;
    }
}
// Send ready ping pong buffers over Bluetooth as formatted lines
void PingPong_SendReadyBuffers(void)
{
	if (!Full) return;

	Full = false;
    char line[160];
    uint8_t buf;
    for (buf = 0; buf < PINGPONG_BUF_ROWS; ++buf) {
        if (!ppReady[buf]) continue;
        // Start line with team label
        int n = snprintf(line, sizeof(line), "Team 16:");
        uint8_t i;
        // Append all samples for this buffer
        for (i = 0; i < PINGPONG_BUF_COLS; ++i) {
            n += snprintf(line + n, sizeof(line) - n, " %d", PingPongBuf[buf][i]);
            PingPongBuf[buf][i] = 0;
        }
        // End line with carriage return and newline
        n += snprintf(line + n, sizeof(line) - n, " \r\n");
        // Send line over Bluetooth
        UART1_Send((uint8_t*)line, (uint32_t)n);
        // Mark this buffer as consumed
        ppReady[buf] = false;
    }
    // Sleep a short time to avoid hogging the CPU
    //Task_sleep(10);
}
}
// Flush any partial buffer on logging stop
// Called in Timer1A_Handler when phase changes
void DataLog_FlushPartial(void)
{
    if (ppIndex == 0) return;  // Nothing in buffer
    // Pad with zeros to full size
    uint8_t i;
    for (i = ppIndex; i < PINGPONG_BUF_COLS; ++i) {
        PingPongBuf[ppActive][i] = 0;
    }
    // Mark partial buffer as ready to send
    //SEND_BOTH("Toilet Time\r\n");
    ppReady[ppActive] = true;
    Full= true;
    PingPong_SendReadyBuffers();
    // Flip active buffer
    ppActive ^= 1;
    ppIndex = 0;
}
// Background task that keeps sending any ready ping pong buffers
void loggerTaskFxn(UArg arg0, UArg arg1)
{
    for (;;) {
        PingPong_SendReadyBuffers();
    }
}
// COMMUNICATION FUNCTIONS
// Send a string to BOTH USB and Bluetooth at the same time
void SEND_BOTH(const char *s) {
    uint32_t n = (uint32_t)strlen(s); // Count characters in string
    UART0_Send((uint8_t*)s, n); // Send to USB
    UART1_Send((uint8_t*)s, n); // Send to Bluetooth
}
// Send data buffer to UART0 (USB) one byte at a time
void UART0_Send(uint8_t *buf, uint32_t n) {
    while (n--) {
        UARTCharPut(UART0_BASE, *buf++); // Send byte, then move pointer to next byte
    }
}
// Send data buffer to UART1 (Bluetooth) one byte at a time
void UART1_Send(uint8_t *buf, uint32_t n) {
    while (n--) {
        UARTCharPut(UART1_BASE, *buf++); // Send byte, then move pointer to next byte
    }
}
// PWM AND MOTOR CONTROL FUNCTIONS
// Initialize PWM hardware for motor speed control
// PWM rapidly switches power on and off to control speed
// Example: 75 percent duty cycle means power is on 75 percent of the time
void PWM_Init(void)
{
    // STEP 1: Enable power to PWM and GPIO peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0); // PWM Module 0
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0)); // Wait until ready
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // Port B for PWM pins
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); // Port D for direction pins
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    // STEP 2: Set PWM clock speed
    // System runs at 40 MHz, divide by 64 gives 625 kHz PWM clock
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // STEP 3: Configure PWM output pins PB4 and PB5
    GPIOPinConfigure(GPIO_PB4_M0PWM2); // PB4 is M0PWM2 for left motor speed
    GPIOPinConfigure(GPIO_PB5_M0PWM3); // PB5 is M0PWM3 for right motor speed
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    // STEP 4: Configure direction control pins PD2 and PD3
    GPIOPinTypeGPIOOutput(LEFT_DIR_PORT_BASE, LEFT_DIR_PIN);   // PD2 left direction
    GPIOPinTypeGPIOOutput(RIGHT_DIR_PORT_BASE, RIGHT_DIR_PIN); // PD3 right direction
    // STEP 5: Configure PWM generator
    // PWM Module 0, Generator 1 controls outputs 2 and 3
    // Down counting mode
    PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    // STEP 6: Calculate PWM period for 20 kHz frequency
    // PWM clock = 625000 Hz
    // Desired frequency = 20000 Hz
    // Period = (625000 / 20000) - 1 = about 30 timer ticks
    uint32_t pwmClk = SysCtlClockGet() / 64; // Get PWM clock speed
    g_pwmLoad = (pwmClk / PWM_FREQ_HZ) - 1; // Calculate period
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, g_pwmLoad); // Set the period
    // STEP 7: Start with motors at 0 percent duty cycle
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // Left motor 0
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0); // Right motor 0
    // STEP 8: Keep PWM outputs disabled until "FWD" command
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, false);
    // STEP 9: Enable the PWM generator, starts internal counting
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);
}
// Enable or disable PWM signal outputs to motors
// When disabled, motors receive no signal and coast
// When enabled, motors run at the set PWM duty
void Motors_EnableOutputs(bool on)
{
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, on);
}
// Set motor duty cycles for speed control
// leftPc: left motor speed percentage (0 to 100)
// rightPc: right motor speed percentage (0 to 100)
// This is where PID adjustments are finally applied to hardware
void Motors_SetDuty(float leftPc, float rightPc)
{
    // Clamp values to safe operating range
    g_leftDuty = fclamp(leftPc, DUTY_MIN_PC, DUTY_MAX_PC);
    g_rightDuty = fclamp(rightPc, DUTY_MIN_PC, DUTY_MAX_PC);
    // Convert percentage to PWM timer ticks
    uint32_t leftTicks  = (uint32_t)((float)g_pwmLoad * g_leftDuty  / 100.0f);
    uint32_t rightTicks = (uint32_t)((float)g_pwmLoad * g_rightDuty / 100.0f);
    // Write pulse widths to PWM hardware registers
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, leftTicks);  // Left motor
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, rightTicks); // Right motor
}
// Set both motors to spin in forward direction
// This only sets direction, not speed
void Motors_SetForward(void)
{
    // Set both direction pins high for forward motion
    GPIOPinWrite(LEFT_DIR_PORT_BASE, LEFT_DIR_PIN, LEFT_DIR_PIN);   // Left forward
    GPIOPinWrite(RIGHT_DIR_PORT_BASE, RIGHT_DIR_PIN, RIGHT_DIR_PIN); // Right forward
}
// Configure motors for a u turn by making them spin in opposite directions
void Motors_SetUTurn(void)
{
    // Left motor forward, right motor backward
    GPIOPinWrite(LEFT_DIR_PORT_BASE, LEFT_DIR_PIN, LEFT_DIR_PIN); // Left forward
    GPIOPinWrite(RIGHT_DIR_PORT_BASE, RIGHT_DIR_PIN, 0);          // Right backward
}
// Immediately set PWM duty to 0 percent on both motors
// Motors will coast to a stop since power is cut
void Motors_AllStop(void)
{
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 0); // Left motor 0 percent
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 0); // Right motor 0 percent
}
// PID Function, runs one PID update using the current distance
void PID_Function(float g_distCm){

    // Calculate distance error
    // Positive error means robot is too far from wall
    // Negative error means robot is too close to wall
    float err = (g_distCm - DIST_TARGET_CM);
    // Get absolute error, how far off we are ignoring direction
    float aerr = fabsf(err);
    (void)aerr; // currently not used but kept for possible debug
    // Sample time in seconds for this PID update
    float Ts = (float)SAMPLE_PERIOD_MS / 1000.0f;
    // INTEGRAL TERM
    // Accumulate error over time, adds up past error
    // This removes steady state offset from target distance
    g_errInt += err * Ts;
    // Clamp integral to prevent it from growing too large
    g_errInt = fclamp(g_errInt, -50.0f, 50.0f);
    // DERIVATIVE TERM
    // Rate of change of error, how fast error is growing or shrinking
    float derr = (err - g_errPrev) / Ts;
    g_errPrev = err; // store current error for next time
    // PID formula output
    // adjust = Kp * error + Ki * integral + Kd * derivative
    float adjust = (Kp*err) + (Ki*g_errInt) + (Kd*derr);
    // Log the current error sample into ping pong buffer for data logging
    DataLog_Append100ms(err);
    // STEP 4: Apply adjustment to motor speeds
    // When error is positive we are too far from wall, so turn toward wall
    // When error is negative we are too close to wall, so turn away
    float left  = BASE_DUTY_PC - adjust;
    float right = BASE_DUTY_PC + adjust;
    // Update motor speeds
    Motors_SetDuty(left, right);
}
// ADC SENSOR INITIALIZATION
// Configure ADC to read the right side IR distance sensor
// ADC is Analog to Digital Converter: converts voltage to digital value
void ADC_Init_RightSensor(void)
{
    // STEP 1: Enable and configure Port E for analog input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Configure PE1 as analog input
    GPIOPinTypeADC(RIGHT_SENSOR_PORT_BASE, RIGHT_SENSOR_PIN);
    // STEP 2: Setup LEDs on Port F
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_ALL);
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0); // Start with LEDs off
    // STEP 3: Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    // STEP 4: Configure Sample Sequencer 3 for a single sample
    ADCSequenceDisable(ADC0_BASE, 3); // Disable before configuring
    // Processor trigger means software starts conversion
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_PROCESSOR, 0);
    // STEP 5: Configure the single sampling step
    // Step 0:
    // - Read from AIN2 channel (PE1 pin)
    // - Generate interrupt when done (ADC_CTL_IE)
    // - End the sequence (ADC_CTL_END)
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0,
                             RIGHT_SENSOR_ADC_CH | ADC_CTL_IE | ADC_CTL_END);
    // STEP 6: Enable sequencer and interrupts
    ADCSequenceEnable(ADC0_BASE, 3); // Turn on sample sequencer
    ADCIntClear(ADC0_BASE, 3);       // Clear any old interrupts
    ADCIntEnable(ADC0_BASE, 3);      // Enable ADC interrupts
    // NVIC enable will be handled by TI RTOS Hwi_create
}
// Configure ADC to read the front IR distance sensor
void ADC_Init_FrontSensor(void)
{
    // STEP 1: Enable and configure Port E for analog input
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    // Configure PE2 as analog input (front sensor)
    GPIOPinTypeADC(FRONT_SENSOR_PORT_BASE, FRONT_SENSOR_PIN);
    // STEP 2: Enable ADC0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));
    // STEP 3: Configure Sample Sequencer 2
    ADCSequenceDisable(ADC0_BASE, 2); // Disable before configuring
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    // STEP 4: Configure the single sampling step
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0,
                             FRONT_SENSOR_ADC_CH | ADC_CTL_IE | ADC_CTL_END);
    // STEP 5: Enable sequencer and its interrupt
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);
    ADCIntEnable(ADC0_BASE, 2);
    // NVIC enable will be handled by TI RTOS Hwi_create
}
// TIMER INITIALIZATION FOR PID CONTROL
// Configure Timer1A to trigger every 50 milliseconds
// This controls the rate of PID updates and sensor reading
void Timer1A_Init_50ms(void)
{
    // Timer starts off and will be enabled when "FWD" command is received
    // STEP 1: Enable Timer1 peripheral
    SysCtlPeripheralEnable(PID_TIMER_PERIPH);
    while(!SysCtlPeripheralReady(PID_TIMER_PERIPH));
    // STEP 2: Configure as periodic timer, repeats automatically
    TimerConfigure(PID_TIMER_BASE, TIMER_CFG_PERIODIC);
    // STEP 3: Calculate how many clock ticks correspond to 50 ms
    // System clock = 40 MHz
    // 1 ms = 40000 ticks
    // 50 ms = 2,000,000 ticks
    uint32_t ticks = (SysCtlClockGet() / 1000) * SAMPLE_PERIOD_MS;
    // STEP 4: Load the timer period, subtract 1 because timer counts down from N to 0
    TimerLoadSet(PID_TIMER_BASE, TIMER_A, ticks - 1);
    // STEP 5: Enable timeout interrupt for this timer
    TimerIntEnable(PID_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    // NVIC enable will be handled by TI RTOS Hwi_create
}
// INTERRUPT HANDLERS (used as Hwi under TI RTOS)
// UART0 interrupt handler
// Simply echoes back any received characters over USB
void UART0IntHandler(void)
{
    // Read and clear interrupt status flags
    uint32_t status = UARTIntStatus(UART0_BASE, true);
    UARTIntClear(UART0_BASE, status);
    // Echo all available characters back to sender
    while(UARTCharsAvail(UART0_BASE)) {
        // Read character and immediately send it back
        UARTCharPutNonBlocking(
            UART0_BASE,
            UARTCharGetNonBlocking(UART0_BASE));
    }
}
// UART1 interrupt handler
// Called automatically when Bluetooth module receives data
// Processes 3 character commands like "FWD" and "STP"
void UART1IntHandler(void)
{
    // Read and clear interrupt status
    uint32_t status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, status);
    // Process all available characters
    uint32_t c = UARTCharGetNonBlocking(UART1_BASE);

    while(UARTCharsAvail(UART1_BASE))
    {
        // Read one character from Bluetooth
        uint32_t c = UARTCharGetNonBlocking(UART1_BASE);
        // Echo character to BOTH Bluetooth and USB
        UARTCharPutNonBlocking(UART1_BASE, c);
        UARTCharPutNonBlocking(UART0_BASE, c);
        // Add character to command buffer
        g_cmdBuf[g_cmdLen++] = (uint8_t)c;
        // When 3 characters received, we have a complete command
        if (g_cmdLen == 3) {
            do_cmd_lookup_and_run(g_cmdBuf); // Execute the command
            g_cmdLen = 0; // Reset for next command
        }
    }
}
// TIMER Hwi
// Called automatically every 50 milliseconds when timer expires
// Checks line sensor and triggers ADC conversions
void Timer1A_Handler(void)
{
    // Clear timer interrupt
    TimerIntClear(PID_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    //check ppBuff
    PingPong_SendReadyBuffers();

    // Read ONLY the left line sensor (active low input with pull up)

    const uint8_t mask = LINE_SENSOR_MASK; // PC4
    const bool low = ((GPIOPinRead(LINE_SENSOR_PORT_BASE, mask) & mask) != mask);
    // Handle thick line flashing phase
    if (g_linePhase == LP_THICK) {
        if (g_flashTicks) {
            g_flashTicks--;
            g_redOn ^= 1;
            GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, g_redOn ? LED_RED : 0);
            Buzzer_Stop();
            if (!g_flashTicks) { // done flashing, power down motors
                GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0);
                TimerDisable(PID_TIMER_BASE, TIMER_A);
            }
        }
        return;
    }
    // If line sensor is not low, we are on a line, turn LEDs on and count hold time
    if(!low){
        GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_ALL);
        g_lineHoldTicks++;
    }
    // Measure how long the left line is held (active low)
    if (g_lineHoldTicks && low) {
        uint16_t held = g_lineHoldTicks;
        g_lineHoldTicks = 0;
        // Thick line detection: long hold means thick line, stop robot and start flashing
        if (held >= THICK_MIN_TICKS) {
            Motors_AllStop();
            Motors_EnableOutputs(false);
            g_linePhase = LP_THICK;
            char* msg0 = "Thick";
            //UART1_Send((uint8_t*)msg0, (uint32_t)strlen(msg0));
            g_flashTicks = 1200; // 60 seconds at 50 ms
            g_redOn = 0;
            return;
        }
        if (low) {
            GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL,0);
        }
        // Thin line detection
        if (held >= THIN_MIN_TICKS && held <= THIN_MAX_TICKS) {
            // First thin line, start logging
            if (g_linePhase == LP_BEFORE1) {
                g_linePhase = LP_LOGGING;
                g_logEnable = true;
                g_logDecim = 0;
                GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_GREEN);
            }
            // Second thin line, stop logging
            else if (g_linePhase == LP_LOGGING) {
                g_linePhase = LP_BEFORE1;
                g_logEnable = false;
                DataLog_FlushPartial();
                GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, LED_BLUE);
            }
        }
    }
    // Normal 50 ms control loop, keeps running until thick phase
    // Trigger front sensor; it will chain to right sensor and then Swi PID
    ADCProcessorTrigger(ADC0_BASE, 2);
}
// ADC INTERRUPTS
// ADC SAMPLE SEQUENCER 3 (right wall sensor)
// Short handler: converts raw value, computes distance, then posts Swi for PID work
void ADC0SS3_Handler(void)
{
    // Clear ADC interrupt flag
    ADCIntClear(ADC0_BASE, 3);
    // Read the converted value from ADC (0 to 4095)
    ADCSequenceDataGet(ADC0_BASE, 3, (uint32_t*)&g_adcRaw);
    // Convert ADC number to voltage (0 to 3.3V)
    g_volts = (float)g_adcRaw * (VREF / ADC_FULL);
    // Convert voltage to distance using calibration curve
    g_distCm = DIST_CM_FVOLT(g_volts);
    // Defer PID and right turn logic into a Swi (software interrupt)
    Swi_post(swiPID);
}
// ADC SAMPLE SEQUENCER 2 (front sensor)
void ADC0SS2_Handler(void)
{
    // Clear ADC interrupt flag
    ADCIntClear(ADC0_BASE, 2);
    // Read the converted value from ADC (0 to 4095)
    ADCSequenceDataGet(ADC0_BASE, 2, (uint32_t*)&g_adcRaw);
    // STEP 1: Convert raw ADC value to distance
    // Convert ADC number to voltage (0 to 3.3V)
    g_volts = (float)g_adcRaw * (VREF / ADC_FULL);
    // Convert voltage to distance using same calibration curve
    g_distCm = DIST_CM_FVOLT(g_volts);
    // Front distance sensor and right distance sensor interrupt logic
    if(g_distCm < 6.0f)
    {
        // Very close to obstacle, stop and set u turn flag
        Motors_AllStop();
        uturnFlag = true;
        uturn();
    }
    else if((g_distCm < 14.0f) && uturnFlag)
    {
        // Still close and already in u turn mode, keep turning
        uturn();
    }
    else if(uturnFlag)
    {
        // Far enough from obstacle, resume forward motion and clear flag
        forward();
        uturnFlag = false;
    }
    else
    {
        // Once front logic is done, trigger right sensor to run PID Swi
        ADCProcessorTrigger(ADC0_BASE, 3); // Right distance sensor triggered
    }
}
// Swi that runs PID and right turn decision (called from ADC0SS3_Handler)
void swiPIDFxn(UArg arg0, UArg arg1)
{
    // If we are too far from wall, do a right correction turn
    if(g_distCm > 12.0f)
    {
        right();
    }
    else
    {
        // Otherwise call PID function for fine adjustment
        PID_Function(g_distCm);
    }
}
// COMMAND PROCESSING
// Execute 3 character commands received via Bluetooth
void do_cmd_lookup_and_run(uint8_t *g_cmdBuf)
{
    // Make sure buffer has a null terminator for string compare safety
    g_cmdBuf[3] = '\0';
    uint32_t i = 0;
    uint32_t flag = 0;
    // Search table for matching command string
    for(i = 0; i < 4; i++){
        if((strncmp((char *)g_cmdBuf, CMD_TABLE[i].code, 3)) == 0){
            // If found, call associated function
            CMD_TABLE[i].function();
            flag = 1;
            break;
        }
    }
    // If no command matched, send message back
    if(flag == 0){
        SEND_BOTH("\r\nNot a valid command!");
    }
}
// Forward command: start moving forward and enable PID
void forward(void)
{
    // Configure motors for forward motion
    Motors_SetForward(); // Set direction pins for forward
    // Set initial speed to base cruising speed
    Motors_SetDuty(BASE_DUTY_PC, BASE_DUTY_PC);
    // Enable PWM outputs to motors
    Motors_EnableOutputs(true);
    // Enable logging and reset log decimation flag
    //g_logEnable = true;
    g_logDecim = 0;
    // Play Nokia tune only once at first forward command
    if(flag1){
        Buzzer_PlaySong(NOKIA_TUNE, NOKIA_TUNE_LEN, true);
        flag1 = 0;
    }
    // Start PID timer so control loop runs every 50 ms
    TimerEnable(PID_TIMER_BASE, TIMER_A);
}
// Right command: simple right turn using duty imbalance
void right(void)
{
    // Configure motors for forward motion
    Motors_SetForward(); // Direction still forward
    // Right turn by slowing left motor and keeping right at base speed
    Motors_SetDuty(36.0f, BASE_DUTY_PC);
    // Enable PWM outputs to motors
    Motors_EnableOutputs(true);
}
// U turn command: rotate in place by driving motors in opposite directions
void uturn(void)
{
    // Configure motors for opposite directions
    Motors_SetUTurn();
    // Both motors run at same duty to spin in place
    Motors_SetDuty(92.0f, 92.0f);
    // Enable PWM outputs
    Motors_EnableOutputs(true);
}
// Stop command: stop motors and disable PID and logging
void stop(void)
{
    // 1. Stop PID timer so no more control updates
    TimerDisable(PID_TIMER_BASE, TIMER_A);
    // 2. Cut power to motors, set PWM to 0 percent
    Motors_AllStop();
    // 3. Disable PWM outputs
    Motors_EnableOutputs(false);
    // 4. Turn off all LEDs
    GPIOPinWrite(GPIO_PORTF_BASE, LED_ALL, 0);
    // Stop logging and flush any partial buffer
    g_logEnable = false;
    DataLog_FlushPartial();
}
// BUZZER AND MUSIC FUNCTIONS
// Buzzer initialization for PB6 as Timer0A PWM output and Timer3A as scheduler
void Buzzer_Init(void)
{
    // Enable peripherals for GPIOB, Timer0, and Timer3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER3);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER3));
    // PB6 as T0CCP0 (timer controlled pin for PWM output)
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
    // Timer0A in PWM mode (we set period and duty in Buzzer_SetFrequency)
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR | TIMER_CFG_A_PWM);
    TimerControlLevel(TIMER0_BASE, TIMER_A, false); // non inverted PWM
    // Timer3A one shot used as note scheduler
    TimerConfigure(TIMER3_BASE, TIMER_CFG_ONE_SHOT);
    TimerControlStall(TIMER3_BASE, TIMER_A, true);
    TimerIntDisable(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    // Create Hwi for Timer3A
    Hwi_Params hp;
    Hwi_Params_init(&hp);
    hp.priority = 1; // lower priority than main control interrupts
    Hwi_create(INT_TIMER3A, (Hwi_FuncPtr)Timer3A_BeepScheduler, &hp, NULL);
    Hwi_enableInterrupt(INT_TIMER3A);
    TimerIntEnable(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
}
// Start playing a song by giving note array, length, and loop flag
void Buzzer_PlaySong(const Note *song, uint32_t len, bool loop)
{
    g_song = song;
    g_songLen = len;
    g_songIdx = 0;
    g_songLoop = loop;
    if (!g_song || !g_songLen) {
        Buzzer_Stop();
        return;
    }
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER3_BASE, TIMER_TIMA_TIMEOUT); // ensure enabled
    // Start first note immediately
    Timer3A_BeepScheduler();
}
// Stop playback and silence the buzzer
void Buzzer_Stop(void)
{
    TimerDisable(TIMER3_BASE, TIMER_A);
    Buzzer_SetFrequency(0);
    g_song = NULL;
    g_songLen = 0;
    g_songIdx = 0;
    g_songLoop = false;
}
// Set buzzer frequency using Timer0A PWM
static void Buzzer_SetFrequency(uint32_t f)
{
    // If frequency is zero, we silence the buzzer
    if (f == 0) {
        TimerDisable(TIMER0_BASE, TIMER_A);
        GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_6);
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_6, 0);
        return;
    }
    // Put PB6 back under timer control
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);
    // Transpose up by one octave to help piezo audibility
    f <<= 1;
    // Compute period based on system clock
    uint32_t sys = SysCtlClockGet();
    if (f < 20) f = 20; // avoid very low frequencies
    uint32_t period = sys / f;
    if (period < 8) period = 8;
    if (period > 0xFFFFFF) period = 0xFFFFFF;
    // Split into low 16 bits and high 8 bits for prescaler
    uint32_t loadLow = period & 0xFFFF;
    uint32_t loadPre = (period >> 16) & 0xFF;
    // 50 percent duty for square wave
    uint32_t half = period / 2;
    uint32_t matchLow = half & 0xFFFF;
    uint32_t matchPre = (half >> 16) & 0xFF;
    // Program Timer0A with new period and duty
    TimerDisable(TIMER0_BASE, TIMER_A);
    TimerLoadSet(TIMER0_BASE, TIMER_A, loadLow);
    TimerPrescaleSet(TIMER0_BASE, TIMER_A, loadPre);
    TimerMatchSet(TIMER0_BASE, TIMER_A, matchLow);
    TimerPrescaleMatchSet(TIMER0_BASE, TIMER_A, matchPre);
    TimerEnable(TIMER0_BASE, TIMER_A);
}
// Timer3A ISR: advances through notes, sets tone, and arms next timeout
void Timer3A_BeepScheduler(void)
{
    // Acknowledge interrupt
    TimerIntClear(TIMER3_BASE, TIMER_TIMA_TIMEOUT);
    // If no song loaded, stop
    if (!g_song || g_songLen == 0) {
        Buzzer_Stop();
        return;
    }
    // Get current note
    const Note n = g_song[g_songIdx];
    // Program tone for this note
    Buzzer_SetFrequency(n.freqHz);
    // Arm next timeout in milliseconds for this note
    // Convert ms to ticks: (SysClk/1000) * ms, minus 1 for timer
    uint32_t ticks = (SysCtlClockGet() / 1000U) * (n.ms ? n.ms : 1U);
    TimerLoadSet(TIMER3_BASE, TIMER_A, ticks - 1);
    TimerEnable(TIMER3_BASE, TIMER_A);
    // Advance index for next note
    g_songIdx++;
    if (g_songIdx >= g_songLen) {
        if (g_songLoop) {
            g_songIdx = 0;
        } else {
            // Last note, schedule silence next time
            g_song = NULL;
            g_songLen = 0;
        }
    }
}
// MAIN FUNCTION
int main(void)
{
    // Required because PID uses float calculations in interrupt handlers
    FPUEnable();
    FPULazyStackingEnable();
    // Set system clock to 40 MHz using PLL and 16 MHz crystal
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL |
                       SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
    // Initialize UART0 (USB)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    // Configure PA0 and PA1 as UART pins
    GPIOPinConfigure(GPIO_PA0_U0RX); // PA0 receive
    GPIOPinConfigure(GPIO_PA1_U0TX); // PA1 transmit
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Initialize UART1 (Bluetooth)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
    // Configure PB0 and PB1 as UART pins
    GPIOPinConfigure(GPIO_PB0_U1RX); // PB0 receive from Bluetooth
    GPIOPinConfigure(GPIO_PB1_U1TX); // PB1 transmit to Bluetooth
    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Configure UART parameters (baud rate and data format)
    // 8 data bits, no parity, 1 stop bit
    // UART0: 115200 baud
    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), USB_BAUD,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);
    // UART1: 9600 baud
    UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), BT_BAUD,
                            UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                            UART_CONFIG_PAR_NONE);
    // Clear stale interrupt state before enabling sources
    UARTIntClear(UART1_BASE, UARTIntStatus(UART1_BASE, false));
    UARTIntClear(UART0_BASE, UARTIntStatus(UART0_BASE, false));
    // Now enable RX and receive timeout interrupts
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
    // Initialize robot subsystems
    PWM_Init();             // Setup PWM for motor speed control
    ADC_Init_RightSensor(); // Setup ADC for right IR distance sensor
    ADC_Init_FrontSensor(); // Setup ADC for front sensor
    Timer1A_Init_50ms();    // Setup timer for 50 ms PID updates
    // Line sensor GPIO init (digital inputs with pull ups)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    GPIOPinTypeGPIOInput(LINE_SENSOR_PORT_BASE, LINE_SENSOR_MASK); // PC4 only
    GPIOPadConfigSet(LINE_SENSOR_PORT_BASE, LINE_SENSOR_MASK, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    // Ensure motors are idle at startup
    Motors_SetDuty(0.0f, 0.0f);  // Set duty to 0 percent
    Motors_EnableOutputs(false); // Disable PWM outputs
    // Motors will not move until "FWD" command is received
    // Create Hwis under TI RTOS for timer, ADC, and UART ISR functions

        Hwi_Params hwiParams;
        Hwi_Params_init(&hwiParams);
        // Timer 1A handler priority
        hwiParams.priority = 6;
        Hwi_create(PID_TIMER_INT, Timer1A_Handler, &hwiParams, NULL);
        // ADC sequence 2 handler priority
        hwiParams.priority = 3;
        Hwi_create(INT_ADC0SS2, ADC0SS2_Handler, &hwiParams, NULL);
        // ADC sequence 3 handler priority
        hwiParams.priority = 3;
        Hwi_create(INT_ADC0SS3, ADC0SS3_Handler, &hwiParams, NULL);
        // UART0 handler
        Hwi_create(INT_UART0, UART0IntHandler, &hwiParams, NULL);
        // UART1 handler
        Hwi_create(INT_UART1, UART1IntHandler, &hwiParams, NULL);
        // Enable all these interrupts in NVIC
        Hwi_enableInterrupt(PID_TIMER_INT);
        Hwi_enableInterrupt(INT_ADC0SS2);
        Hwi_enableInterrupt(INT_ADC0SS3);
        Hwi_enableInterrupt(INT_UART0);
        Hwi_enableInterrupt(INT_UART1);

    // Create Swi for PID and right turn processing

        Swi_Params swiParams;
        Swi_Params_init(&swiParams);
        swiPID = Swi_create(swiPIDFxn, &swiParams, NULL);

    // Create background Task for ping pong Bluetooth logging

        Task_Params taskParams;
        Task_Params_init(&taskParams);
        taskParams.stack = loggerTaskStack;
        taskParams.stackSize = sizeof(loggerTaskStack);
        Task_construct(&loggerTaskStruct, loggerTaskFxn, &taskParams, NULL);

    // Terminal messages (currently commented out)
    const char *msg0 = "\033[2JType 3-char cmds: FWD/RGT/UTU/STP\r\n";
    const char *msg1 = "BT ready. FWD uses right wall PID at 50ms.\r\n";
    (void)msg0;
    (void)msg1;
    // UART0_Send((uint8_t*)msg0, (uint32_t)strlen(msg0)); // Send to USB
    // UART0_Send((uint8_t*)msg1, (uint32_t)strlen(msg1)); // Send to USB
    // UART1_Send((uint8_t*)msg1, (uint32_t)strlen(msg1)); // Send to Bluetooth
    // Initialize buzzer hardware
    Buzzer_Init();
    // Buzzer_Stop();
    // Start TI RTOS scheduler. From here on, Tasks, Hwis, and Swis run everything.
    BIOS_start();
    return 0;
}


