// use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
static const uint8_t buf_len = 255;

// Globals
static char *msg_ptr = NULL;
static volatile uint8_t msg_flag = 0;

//*****************************************************************************
// Tasks

// Task A: Listens for input from Serial Monitor (UART)
// On newline char ('\n') stores all chars up to that point in heap memory
// Notifies TAsk B of new message
void listenForInput(void *parameter) {
  char c;
  char buf[buf_len];
  uint8_t idx = 0;

  // Clear whole buffer
  memset(buf, 0, buf_len);

  while (1) {
    if (Serial.available() > 0) {
      c = Serial.read();

      // Store received character to buffer if not over buffer limit
      if (idx < buf_len - 1) {
        buf[idx] = c;
        idx++;
      }

      // Create a message buffer for print task
      if (c == '\n') {
        // The last character in the string is '\n', so we need to replace
        // it with '\0' to make it null-terminated
        buf[idx-1] = '\0';

        // Try to allocate memory and copy over message. If message buffer is
        // still in use, ignore the entire message.
        if (msg_flag == 0) {
          msg_ptr = (char *)pvPortMalloc(idx * sizeof(char));

          // If malloc returns 0 (out of memory), throw an error and reset
          // configASSERT(msg_ptr);
          if (msg_ptr == NULL) {
            Serial.println("Not enough heap.");
            return;
          }

          // copy message
          memcpy(msg_ptr, buf, idx);

          // Notify other task that message is ready
          msg_flag = 1;
        }

        // Reset receive buffer and index counter
        memset(buf, 0, buf_len);
        idx = 0;
      }
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay to avoid busy loop
  }
}


// Task B: Prints message found in heap memory to Serial Monitor
// Waits for notification from Task A
// Frees heap memory
void printMessage(void *parameter) {
  while(1) {
    // Wait for flag to be set and print message
    if (msg_flag == 1) {
      Serial.println(msg_ptr);

      // Show amount of free heap memory
      Serial.print("Free heap (bytes): ");
      Serial.println(xPortGetFreeHeapSize());

      // Free buffer and heap memory and reset flag
      vPortFree(msg_ptr);
      msg_ptr = NULL;
      msg_flag = 0;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay to avoid busy loop
  }
}

//*****************************************************************************
// main
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial ouput)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Memory Challenge---");

  xTaskCreatePinnedToCore(listenForInput,
                          "Task A",
                          2048,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  xTaskCreatePinnedToCore(printMessage,
                          "Task B",
                          2048,
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
