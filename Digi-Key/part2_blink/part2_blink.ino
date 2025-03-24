// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Pins
static const int led_pin = 2;

// LED Rates
static const int rate_1 = 500; // [ms]
static const int rate_2 = 333; // [ms]

// Out task: blink an LED
void toggleLED_1(void *parameter) {
  while(1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(rate_1 / portTICK_PERIOD_MS); // hardware tick: normally 1 for 1 ms
    digitalWrite(led_pin, LOW);
    vTaskDelay(rate_1 / portTICK_PERIOD_MS); // 500 ms
  }
}

void toggleLED_2(void *parameter) {
  while(1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(rate_2 / portTICK_PERIOD_MS); // hardware tick: normally 1 for 1 ms
    digitalWrite(led_pin, LOW);
    vTaskDelay(rate_2 / portTICK_PERIOD_MS); // 500 ms
  }
}

void setup() {
  // put your setup code here, to run once:
  // configure pin
  pinMode(led_pin, OUTPUT);

  // Task to run forever
  xTaskCreatePinnedToCore(    // Use xTaskCreate() in Vanilla FreeRTOS
    toggleLED_1,              // Function to be called
    "Toggle 1",               // Name of task
    1024,                     // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,                     // Parameter to pass to function
    1,                        // Task priortiy (0 to configMax_PRIORITIES - 1)
    NULL,                     // Task handle, to be used in another task
    app_cpu);                 // CPU core -> Run on one core for demo purposes (ESP32 only)

  xTaskCreatePinnedToCore(    // Use xTaskCreate() in Vanilla FreeRTOS
    toggleLED_2,              // Function to be called
    "Toggle 2",               // Name of task
    1024,                     // Stack size (bytes in ESP32, words in FreeRTOS)
    NULL,                     // Parameter to pass to function
    1,                        // Task priortiy (0 to configMax_PRIORITIES - 2)
    NULL,                     // Task handle, to be used in another task
    app_cpu);                 // CPU core -> Run on one core for demo purposes (ESP32 only)

 // If this was Vanilla FreeRTOS, call vTaskStartScheduler() in main after setting up tasks
}

void loop() {
  // put your main code here, to run repeatedly:

}
