// Part 4 demo: Memory management
// use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Task: Perform some mundane task
void testTask(void *parameter) {
  while (1) {
    int a = 1;  // 4 bytes
    int b[100]; // 400 bytes

    // Do something with array so it's not optimized out by the compiler
    for (int i = 0; i < 100; i++) {
      b[i] = a + 1;
    }
    //Serial.println(b[0]);

    // Print out remaining stack memory (words)
    Serial.print("High water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));

    // Print out number of free heap memory bytes before malloc
    Serial.print("Heap before malloc (bytes): ");
    Serial.println(xPortGetFreeHeapSize());

    int *ptr = (int*)pvPortMalloc(1024 * sizeof(int));
    // One way to prevent heap overflow is to check the malloc output
    if (ptr == NULL) {
      Serial.println("Not enough heap.");
      vPortFree(NULL);
    } else {
      // Do something with the memory so it's not optimized out by the compiler
      for (int i = 0; i < 1024; i++) {
        ptr[i] = 3;
      }
    }

    // Print out number of free heap memory bytes after malloc
    Serial.print("Heap after malloc (bytes): "); 
    Serial.println(xPortGetFreeHeapSize());

    // Free up our allocated memory
    vPortFree(ptr);

    // Wait for a while
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial ouput)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Memory Demo---");

  // Start the only other task
  xTaskCreatePinnedToCore(testTask,
                          "Test Task",
                          1500,         // Stack depth [bytes] -> Local variables created during function calls within a task are pushed to the task’s local stack. 
                                        // Because of this, it’s important to calculate the predicted stack usage of a task ahead of time and include that 
                                        // as the stack size parameter in xTaskCreate().
                          NULL,
                          1,
                          NULL,
                          app_cpu);
  
  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // Execution should never get here
}
