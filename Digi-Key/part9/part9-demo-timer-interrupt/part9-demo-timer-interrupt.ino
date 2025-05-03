/**
 * ESP32 Timer Interrupt Demo
 * 
 * Blink LED with hardware timer interrupt.
 * 
 * Date: February 3, 2021
 * Author: Shawn Hymel
 * License: 0BSD
 * Edited for the new Arduino-ESP32 version: v3.2.0
 */

#include "esp32-hal-timer.h"

// Settings
static const uint32_t timer_freq = 1000000; // 1MHz frequency (1 count = 1μs)
static const uint64_t timer_max_count = 1000000;

// Pins
static const int led_pin = 2;

// Globals
static hw_timer_t *timer = NULL;
volatile bool led_state = false; // Track LED ON/OFF

//*****************************************************************************
// Interrupt Service Routines (ISRs)

// This function executes when timer reaches max (and resets)
// In the new ESP32 v3.2.0 API, this may be handled internally by the timerAttachInterrupt function.
void onTimer() {
  // Toggle LED
  led_state = !led_state;         // Toggle LED state
  digitalWrite(led_pin, led_state); // Apply new LED state
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // put your setup code here, to run once:
  // Configure LED pin
  pinMode(led_pin, OUTPUT);

  // Create and start timer (frequence)
  timer = timerBegin(timer_freq);
  if (timer == NULL) return;

  // Attach the interrupt handler (timer, function)
  timerAttachInterrupt(timer, &onTimer);

  // At what count should ISR trigger (timer, alarm_value, autoreload, reload_count)
  // Set alarm to trigger once per second (1000000 microseconds) with autoreload
  timerAlarm(timer, timer_max_count, true, 0);
}

void loop() {
  // put your main code here, to run repeatedly:

}
