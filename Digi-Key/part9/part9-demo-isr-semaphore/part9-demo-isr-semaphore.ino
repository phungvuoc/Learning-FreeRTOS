/**
 * ESP32 ISR Semaphore Demo
 * 
 * Read ADC values in ISR at 1 Hz and defer printing them out in a task.
 * 
 * Date: February 3, 2021
 * Author: Shawn Hymel
 * License: 0BSD
 * Edited for Arduino ESP32 v3.2.0
 */

// You'll likely need this on vanilla FreeRTOS
//#include <semphr.h>

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
// static const uint16_t timer_divider = 80; // old version: Count at 1 MHz
static const uint32_t timer_freq = 1e6; // 1 MHz frequency
static const uint64_t timer_max_count = 1e6; // 1e6 * 1 MHz = 1 s

// Pins
static const int adc_pin = A0;

// Globals
static hw_timer_t *timer = NULL;
static volatile uint16_t val;
static SemaphoreHandle_t bin_sem = NULL;

//*****************************************************************************
// Interrupt Service Routines (ISRs)

// This function executes when timer reaches max (and resets)
// Interrupt handler
void IRAM_ATTR onTimer() {
  // To detect if a higher-priority task is unblocked in ISR
  BaseType_t task_woken = pdFALSE;

  // Perform action (read from ADC)
  val = analogRead(adc_pin);

  // Give semaphore to tell task that new value is ready
  xSemaphoreGiveFromISR(bin_sem, &task_woken);

  // Exit from ISR (Vanilla FreeRTOS)
  //portYIELD_FROM_ISR(task_woken);

  // Exit from ISR (ESP-IDF)
  if (task_woken) {
    portYIELD_FROM_ISR();
  }
}

//*****************************************************************************
// Tasks

// Wait for semaphore and print out ADC value when received
// Deffered Task
void printValues(void *parameters) {

  // Loop forever, wait for semaphore, and print value
  while (1) {
    xSemaphoreTake(bin_sem, portMAX_DELAY);
    Serial.println(val); 
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {

  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS ISR Buffer Demo---");

  // Create semaphore before it is used (in task or ISR)
  bin_sem = xSemaphoreCreateBinary();

  // Force reboot if we can't create the semaphore
  if (bin_sem == NULL) {
    Serial.println("Could not create semaphore");
    ESP.restart();
  }

  // Start task to print out results (higher priority!)
  xTaskCreatePinnedToCore(printValues,
                          "Print values",
                          1024,
                          NULL,
                          2,              // Higher priority for the deferred task
                          NULL,
                          app_cpu);

  // Create and start timer (frequence)
  timer = timerBegin(timer_freq);
  if (timer == NULL) return;

  // Attach the interrupt handler (timer, function)
  timerAttachInterrupt(timer, &onTimer);

  // At what count should ISR trigger (timer, alarm_value, autoreload, reload_count)
  timerAlarm(timer, timer_max_count, true, 0);
}

void loop() {
  // Do nothing, forever
}