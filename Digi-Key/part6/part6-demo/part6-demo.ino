// Part 6 demo
/* Increment a shared global variable with mutex protection.
 **/

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Globals
static int shared_var = 0;
static SemaphoreHandle_t mutex;

//*****************************************************************************
// Tasks

// Increment shared variable 
void incTask(void *parameter) {
  int local_var;

  while (1) {
    if (xSemaphoreTake(mutex, 0) == pdTRUE) {
      // Critical section (poor demonstration of "shared_var++")
      local_var = shared_var;
      local_var++;
      vTaskDelay(random(100,500) / portTICK_PERIOD_MS);
      shared_var = local_var;

      // Print out new shared variable
      Serial.print(pcTaskGetName(NULL)); Serial.print(" : ");
      Serial.println(shared_var);

      // Give mutex after critical section
      xSemaphoreGive(mutex);

      // Give the other task a chance to run
      taskYIELD();
    }
  }
}


//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // put your setup code here, to run once:
  // Hack to kinda get randomness
  randomSeed(analogRead(0));

  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Race Condition Demo---");

  // Create the mutex
  mutex = xSemaphoreCreateMutex();

  // Check the semaphore was created successfully before creating the tasks
  if (mutex != NULL) {
    xTaskCreatePinnedToCore(incTask, "IncTask 1", 1024, NULL, 1, NULL, app_cpu);
    xTaskCreatePinnedToCore(incTask, "IncTask 2", 1024, NULL, 1, NULL, app_cpu);

    vTaskDelete(NULL);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
