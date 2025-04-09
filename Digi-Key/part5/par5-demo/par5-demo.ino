// Part 5 demo: Queue
// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
static const uint8_t msg_queue_len = 5;

// Globals
static QueueHandle_t msg_queue;

//*****************************************************************************
// Tasks

// Task: wait for item on queue and print it
void printMessages(void *parameters) {
  int item;
  BaseType_t xStatus;
  const TickType_t xTicksToDelay = pdMS_TO_TICKS(500);

  // Loop forever
  while (1) {
    // See if there's a message in the queue (do not block)
    Serial.print("Number of messages in the queue: ");
    Serial.println(uxQueueMessagesWaiting(msg_queue));
    
    xStatus = xQueueReceive(msg_queue, (void *)&item, 0); // 0 -> no wait for the message
    if (xStatus == pdTRUE) {
      Serial.println(item);
    } else {
      Serial.println("No new message!");
    }

    // Wait before trying again
    vTaskDelay(xTicksToDelay);
  }
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // put your setup code here, to run once:
  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Queue Demo---");

  // Create queue
  msg_queue = xQueueCreate(msg_queue_len, sizeof(int));

  // Start print task
  xTaskCreatePinnedToCore(printMessages,
                          "Print Messages",
                          1024,
                          NULL,
                          1,
                          NULL,
                          app_cpu);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int num = 0;
  BaseType_t xStatus;

  // Try to add item to queue for 10 ticks, fail if queue is full
  xStatus = xQueueSend(msg_queue, (void *)&num, 10);
  if (xStatus != pdTRUE) {
    Serial.println("Queue full");
  }
  
  num++;

  // Wait before trying again
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
