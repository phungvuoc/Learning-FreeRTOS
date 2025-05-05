/**
 * Part 12 - challenge - multicore
 Specifically, the timer ISR (for sampling the ADC) and the task that computes
  the average of 10 samples should run on Core 0. The user interface task 
  (the one that handles echoing characters to the serial terminal and 
  responding to the “avg” command) should run on Core 1.

 From Part 9 - challenge:
 You should implement a hardware timer in the ESP32 (here is a good article 
 showing how to do that) that samples from an ADC pin once every 100 ms. This 
 sampled data should be copied to a double buffer (you could also use a 
 circular buffer). Whenever one of the buffers is full, the ISR should notify 
 Task A.

 Task A, when it receives notification from the ISR, should wake up and compute
  the average of the previously collected 10 samples. Note that during this 
  calculation time, the ISR may trigger again. This is where a double (or 
  circular) buffer will help: you can process one buffer while the other is 
  filling up.

 When Task A is finished, it should update a global floating point variable 
 that contains the newly computed average. Do not assume that writing to this 
 floating point variable will take a single instruction cycle! You will need to
  protect that action as we saw in the queue episode.

 Task B should echo any characters received over the serial port back to the 
 same serial port. If the command “avg” is entered, it should display whatever
 is in the global average variable.
 */

// You'll likely need this on vanilla FreeRTOS
//#include <semphr.h>

// Use both core
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Settings
static const char command[] = "avg";              // Command
// static const uint16_t timer_divider = 8;       // old version: Divide 80 MHz by this
static const uint32_t timer_freq = 10e6;          // 10 MHz frequency
static const uint64_t timer_max_count = 1e6;      // Timer counts to this value
static const uint32_t cli_delay = 20;             // ms delay
enum { BUF_LEN = 10 };      // Number of elements in sample buffer
enum { MSG_LEN = 100 };     // Max characters in message body
enum { MSG_QUEUE_LEN = 5 }; // Number of slots in message queue
enum { CMD_BUF_LEN = 255};  // Number of characters in command buffer

// Pins
static const int adc_pin = A0;

// Message struct to wrap strings for queue
typedef struct Message {
  char body[MSG_LEN];
} Message;

// Globals
static hw_timer_t *timer = NULL;
static TaskHandle_t processing_task = NULL;
static SemaphoreHandle_t sem_done_reading = NULL;
static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t msg_queue;
static volatile uint16_t buf_0[BUF_LEN];      // One buffer in the pair
static volatile uint16_t buf_1[BUF_LEN];      // The other buffer in the pair
static volatile uint16_t *write_to = buf_0;   // Double buffer write pointer
static volatile uint16_t *read_from = buf_1;  // Double buffer read pointer
static volatile uint8_t buf_overrun = 0;      // Double buffer overrun flag
static float adc_avg;

//*****************************************************************************
// Functions that can be called from anywhere (in this file)

// Swap the write_to and read_from pointers in the double buffer
// Only ISR calls this at the moment, so no need to make it thread-safe
void IRAM_ATTR swap() {
  volatile uint16_t *temp_ptr = write_to;
  write_to = read_from;
  read_from = temp_ptr;
}

//*****************************************************************************
// Interrupt Service Routines (ISRs)

// This function executes when timer reaches max (and resets)
/* Interrupt handler:
 * Read from AD0, write write_to buffer
 * If the buffer is full:
 *  - Wait until the buffer is free via semaphore and set overrun
 *  - Swap write_to and read_from buffer
 *  - Notifiy the processing task (calcAverage) to wake up
 */
void IRAM_ATTR onTimer() {
  // To check how many slots in the write_to buffer are filled
  static uint16_t idx = 0; // static keyword to reatin its value between function calls
  // To detect if a higher-priority task is unblocked in ISR
  BaseType_t task_woken = pdFALSE;
  
  // If buffer is not overrun, read ADC to next buffer element. If buffer is
  // overrun, drop the sample.
  if ((idx < BUF_LEN) && (buf_overrun == 0)) {
    write_to[idx] = analogRead(adc_pin);
    idx++;
  }

  // Check if the buffer is full
  if (idx >= BUF_LEN) {

    // If reading is not done (take = pdFALSE), set overrun flag. We don't need to set this
    // as a critical section, as nothing can interrupt and change either value.
    // This TakeFromISR doesn't need time_delay because this onTimer() function is called,
    // only when the ISR occurs and it will check the semaphore at that time.
    if (xSemaphoreTakeFromISR(sem_done_reading, &task_woken) == pdFALSE) {
      buf_overrun = 1;
    }

    // Only swap buffers and notify task if overrun flag is cleared
    if (buf_overrun == 0) {

      // Reset index and swap buffer pointers
      idx = 0;
      volatile uint16_t *temp_ptr = write_to;
      write_to = read_from;
      read_from = temp_ptr;
  
      // A task notification works like a binary semaphore but is faster
      vTaskNotifyGiveFromISR(processing_task, &task_woken);
    }
  }
  
  // Exit from ISR (Vanilla FreeRTOS)
  //portYIELD_FROM_ISR(task_woken);

  // Exit from ISR (ESP-IDF)
  if (task_woken) {
    portYIELD_FROM_ISR();
  }
}

//*****************************************************************************
// Tasks

// Task B: Serial terminal task
void doCLI(void *parameters) {

  Message rcv_msg;
  char c;
  char cmd_buf[CMD_BUF_LEN];
  uint8_t idx = 0;
  uint8_t cmd_len = strlen(command);

  // Clear whole buffer
  memset(cmd_buf, 0, CMD_BUF_LEN);

  // Loop forever
  while (1) {

    // Look for any error messages that need to be printed
    if (xQueueReceive(msg_queue, (void *)&rcv_msg, 0) == pdTRUE) {
      Serial.println(rcv_msg.body);
    }

    // Read characters from serial
    if (Serial.available() > 0) {
      c = Serial.read();

      // Store received character to buffer if not over buffer limit
      if (idx < CMD_BUF_LEN - 1) {
        cmd_buf[idx] = c;
        idx++;
      }

      // Print newline and check input on 'enter'
      if ((c == '\n') || (c == '\r')) {
        
        // Print newline to terminal
        Serial.print("\r\n");

        // Print average value if command given is "avg"
        cmd_buf[idx - 1] = '\0';
        if (strcmp(cmd_buf, command) == 0) {
          Serial.print("Average: ");
          Serial.println(adc_avg);
        }

        // Reset receive buffer and index counter
        memset(cmd_buf, 0, CMD_BUF_LEN);
        idx = 0;

      // Otherwise, echo character back to serial terminal
      } else {
        Serial.print(c);
      }
    }

    // Don't hog the CPU. Yield to other tasks for a while
    vTaskDelay(cli_delay / portTICK_PERIOD_MS);
  }
}

// Task A: Wait for semaphore and calculate average of ADC values
void calcAverage(void *parameters) {

  Message msg;
  float avg;

  // %%% move creating timer here so it runs in core 0
  // Create and start timer (frequence)
  timer = timerBegin(timer_freq);
  if (timer == NULL) return;
  // Attach the interrupt handler (timer, function)
  timerAttachInterrupt(timer, &onTimer);
  // At what count should ISR trigger (timer, alarm_value, autoreload, reload_count)
  timerAlarm(timer, timer_max_count, true, 0);

  // Loop forever, wait for semaphore, and print value
  while (1) {

    // Wait for notification from ISR (similar to binary semaphore)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Calculate average (as floating point value)
    avg = 0.0;
    for (int i = 0; i < BUF_LEN; i++) {
      avg += (float)read_from[i];
      //vTaskDelay(105 / portTICK_PERIOD_MS); // Uncomment to test overrun flag
    }
    avg /= (float)BUF_LEN;

    // Updating the shared float may or may not take multiple isntructions, so
    // we protect it with a mutex or critical section. The ESP-IDF critical
    // section is the easiest for this application.
    portENTER_CRITICAL(&spinlock);
    adc_avg = avg;
    portEXIT_CRITICAL(&spinlock);

    // If we took too long to process, buffer writing will have overrun. So,
    // we send a message to be printed out to the serial terminal.
    if (buf_overrun == 1) {
      strcpy(msg.body, "Error: Buffer overrun. Samples have been dropped.");
      xQueueSend(msg_queue, (void *)&msg, 10);
    }

    // Clearing the overrun flag and giving the "done reading" semaphore must
    // be done together without being interrupted.
    portENTER_CRITICAL(&spinlock);
    buf_overrun = 0;
    xSemaphoreGive(sem_done_reading);
    portEXIT_CRITICAL(&spinlock);
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
  Serial.println("---FreeRTOS Sample and Process Demo---");

  // Create semaphore before it is used (in task or ISR)
  sem_done_reading = xSemaphoreCreateBinary();

  // Force reboot if we can't create the semaphore
  if (sem_done_reading == NULL) {
    Serial.println("Could not create one or more semaphores");
    ESP.restart();
  }

  // We want the done reading semaphore to initialize to 1
  xSemaphoreGive(sem_done_reading);

  // Create message queue before it is used
  msg_queue = xQueueCreate(MSG_QUEUE_LEN, sizeof(Message));

  // Start task to handle command line interface events. Let's set it at a
  // higher priority but only run it once every 20 ms.
  xTaskCreatePinnedToCore(doCLI,
                          "Do CLI",
                          2048,
                          NULL,
                          2,
                          NULL,
                          app_cpu);

  // Start task to calculate average. Save handle for use with notifications.
  xTaskCreatePinnedToCore(calcAverage,
                          "Calculate average",
                          1024,
                          NULL,
                          1,
                          &processing_task,
                          pro_cpu);

  // Delete "setup and loop" task
  vTaskDelete(NULL);

}

void loop() {
  // Do nothing, forever
}