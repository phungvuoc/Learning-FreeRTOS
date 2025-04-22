// Part 8 Challenge - Software timer
/* Turn on LED when entering serial commands. Turn it off if serial is inactive
 * for 5 seconds.
 */

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Globals
static const int led_pin = 2;
static BaseType_t xLightOn = pdFALSE;
static TimerHandle_t one_shot_timer = NULL;
static const TickType_t time_delay_tick = 5000 / portTICK_PERIOD_MS;

//*****************************************************************************
// Tasks

static void serialInputTask(void *parameters) {
  char c;

  while (1) {
    // Has an input in Serial?
    if (Serial.available() > 0) {
      c = Serial.read();
      if (xLightOn == pdFALSE) {
        // The LED was off, so turn it on
        Serial.println("Turning on...");
        xLightOn = pdTRUE;
        digitalWrite(led_pin, HIGH);
      } else {
        // The LED was already on, so reset the software timer
        Serial.println("Reset timer...");
        xTimerReset(one_shot_timer, time_delay_tick);
      }
    }
  }
}

static void lightTimerCallBack(TimerHandle_t xTimer) {
  Serial.println("Turning off...");
  // Turn off the LED
  xLightOn = pdFALSE;
  digitalWrite(led_pin, LOW);
}

//*****************************************************************************
// MAIN

void setup() {
  // put your setup code here, to run once:
  // Set up serial
  Serial.begin(115200);

  // Configure LED pin
  pinMode(led_pin, OUTPUT);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Timer Challenge---");

  // Create a one-shot timer
  one_shot_timer = xTimerCreate("One-shot timer", 
                                time_delay_tick, 
                                pdFALSE, 
                                (void *)0, 
                                lightTimerCallBack);

  if (one_shot_timer == NULL) {
    Serial.println("Could not create one of the timers");
  } else {
    // Create task
    xTaskCreatePinnedToCore(serialInputTask, 
                            "Serial input task", 
                            1024, 
                            NULL, 
                            1, 
                            NULL, 
                            app_cpu);
  }

  // Delete self task to show that timers will work with no user tasks
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:

}
