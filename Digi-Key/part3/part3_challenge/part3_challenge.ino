/* Using FreeRTOS, create two separate tasks. 
 * One listens for an integer over UART (from the Serial Monitor) 
 * and sets a variable when it sees an integer. 
 * The other task blinks the onboard LED (or other connected LED) 
 * at a rate specified by that integer. 
 * In effect, you want to create a multi-threaded system 
 * that allows for the user interface to run concurrently 
 * with the control task (the blinking LED).
 */

// use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t APP_CPU = 0;
#else
static const BaseType_t APP_CPU = 1;
#endif

// LED pin
static const int LED_PIN = 2;

// Declare a global variable to hold the blink interval
int blink_interval = 500;  // Default value in milliseconds

//**********************************************
// Tasks

// task: listen for input from UART (Serial Monitor)
void listenForInput(void *parameter) {
  Serial.println("Enter a blink interval in milliseconds:");

  while (1) {
    if (Serial.available() > 0) {
      int input_value = Serial.parseInt();  // Read the integer from the serial monitor
      if (input_value > 0) {
        blink_interval = input_value;
        Serial.print("Updated Blink interval: ");
        Serial.println(blink_interval);
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay to avoid busy loop
  }
}

// task: Blink LED at rate set by global variable
void toggleLED(void *parameter) {
  while (1) {
    digitalWrite(LED_PIN, HIGH);
    vTaskDelay(blink_interval / portTICK_PERIOD_MS);
    digitalWrite(LED_PIN, LOW);
    vTaskDelay(blink_interval / portTICK_PERIOD_MS);
  }
}

// *******************************************
// Main (runs as its own task with priority 1 on core 1)
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // configure pin
  pinMode(LED_PIN, OUTPUT);

  // task to run forever
  xTaskCreatePinnedToCore(toggleLED, "Toggle LED", 1024, NULL, 1, NULL, APP_CPU);

  // task to run once with higher priority
  xTaskCreatePinnedToCore(listenForInput, "UART task", 1024, NULL, 2, NULL, APP_CPU);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
}
