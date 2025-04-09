// Part 5 challenge: Queue
/* Use FreeRTOS to create two tasks and two queues.
 * 
 * Task A should print any new messages it receives from Queue 2. 
 * Additionally, it should read any Serial input from the user and echo back 
 * this input to the serial input. If the user enters “delay” followed by a space and a number, 
 * it should send that number to Queue 1.
 *
 * Task B should read any messages from Queue 1. 
 * If it contains a number, it should update its delay rate to that number (milliseconds). 
 * It should also blink an LED at a rate specified by that delay. 
 * Additionally, every time the LED blinks 100 times, it should send the string “Blinked” to Queue 2. 
 * You can also optionally send the number of times the LED blinked (e.g. 100) as part of struct 
 * that encapsulates the string and this number.
*/

// Use only core 1 
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
static const uint8_t buf_len = 255;     // Size of buffer to look for command
static const char command[] = "delay "; // Command to change delay time (Note the space!)
static const int delay_queue_len = 5;   // Size of delay_queue
static const int msg_queue_len = 5;     // Size of msg_queue
static const uint8_t blink_max = 100;   // Num times to blink before message

// LED Pin
static const int led_pin = 2;

// Message struct: used to wrap strings
typedef struct Message {
  char body[20];
  int count;
} Message_t;

// Globals
static QueueHandle_t delay_queue; // Queue 1
static QueueHandle_t msg_queue; // Queue 2

//*****************************************************************************
// Tasks

/* Task A:
 * Prints any new messages from Queue 2
 * Reads serial input from user
 * Echo input back to serial terminal
 *      If "delay xxx", sends xxx (number) to Queue 1
 */
void readAndPrintMessages(void *parameters) {
  // For delay queue
  char c;
  char buf[buf_len];
  uint8_t idx = 0;
  uint8_t cmd_len = strlen(command);
  int led_delay;

  // For msg queue
  Message_t received_msg;
  
  // Clear the whole buffer
  memset(buf, 0, buf_len);

  while (1) {
    // Prints any new message from Queue 2 (msg_queue)
    if (xQueueReceive(msg_queue, (void *)&received_msg, 0) == pdTRUE) {
      Serial.print(received_msg.body);
      Serial.println(received_msg.count);
    }

    // Read command from terminal and send delay time to Queue 1 (delay_queue)
    if (Serial.available() > 0) {
      // Read new character from terminal
      c = Serial.read();

      // Store received character to buffer if not over buffer limit
      if (idx < buf_len - 1) {
        buf[idx] = c;
        idx++;
      }

      // check input on 'enter'
      if ((c == '\n') || (c == '\r')) {
        // Print newline to terminal
        Serial.print("\r\n");

        // Check if the first 6 characters are "delay "
        if (memcmp(buf, command, cmd_len) == 0) {
          // Convert last part to positive integer (negative int crashes)
          char* tail = buf + cmd_len;
          led_delay = atoi(tail);
          led_delay = abs(led_delay);

          // Send integer to other task via queue
          if (xQueueSend(delay_queue, (void *)&led_delay, 10) != pdTRUE) {
            Serial.println("Delay queue is full!");
          }
        }

        // Reset receive buffer and index counter
        memset(buf, 0, buf_len);
        idx = 0;
      } /*else {
        Serial.print(c);
      }*/
    }
  }
}

/* Task B:
 * Updates t with any values from Queue 1
 * Blinks LED with t delay
 * Ever time LED blinks 100 times, send "Blinked" string to Queue 2
 * (optional: also send number of times blinked)
*/
void blinkLED(void *parameters) {
  // For delay queue
  int blink_interval = 500;

  // For msg queue
  Message_t msg;
  int counter = 0;

  while (1) {
    // See if there's a message in the queue (do not block)
    if (xQueueReceive(delay_queue, (void *)&blink_interval, 0) == pdTRUE) {
      // Best practice: use only one task to manage serial
      strcpy(msg.body, "Input delay: ");
      msg.count = blink_interval;
      xQueueSend(msg_queue, (void *)&msg, 10);
      // reset counter
      counter = 0;
    }

    // Blink
    digitalWrite(led_pin, HIGH);
    vTaskDelay(blink_interval / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(blink_interval / portTICK_PERIOD_MS);

    counter++;
    if (counter >= blink_max) {
      strcpy(msg.body, "Blinked: ");
      msg.count = counter;
      xQueueSend(msg_queue, (void *)&msg, 10);
    }
  }
}

//*****************************************************************************
// Main

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // configure pin
  pinMode(led_pin, OUTPUT);

  // Wait a moment to start (so we don't miss Serial ouput)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Queue Challenge---");

  // Create queues
  delay_queue = xQueueCreate(delay_queue_len, sizeof(int));
  msg_queue = xQueueCreate(msg_queue_len, sizeof(Message_t));

  // Start tasks
  xTaskCreatePinnedToCore(readAndPrintMessages, 
                          "read and print", 
                          2048,
                          NULL,
                          1,
                          NULL,
                          app_cpu);
  xTaskCreatePinnedToCore(blinkLED, 
                          "blink LED", 
                          2048,
                          NULL,
                          1,
                          NULL,
                          app_cpu);

  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void loop() {
  // put your main code here, to run repeatedly:
  // should never reach here
}
