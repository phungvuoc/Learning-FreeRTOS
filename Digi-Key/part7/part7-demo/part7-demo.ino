// Part 7 - demo: counting Semaphore
// TODO: can use Mutex to protect serial print
// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
static const int num_tasks = 5;   // Number of tasks to create

// Example struct for passing a string as a parameter
typedef struct Message {
  char body[20];
  uint8_t len;
} Message;

// Globals
static SemaphoreHandle_t sem_params;  // Counts down when parameters used

//*****************************************************************************
// Tasks

void myTask(void *parameters) {
  // Copy the message struct from the parameter to a local variable
  Message msg = *(Message *)parameters;

  // Increment semaphore to indicate that the parameter has been used
  xSemaphoreGive(sem_params);

  // Print out message contents
  Serial.print("Received: ");
  Serial.print(msg.body);
  Serial.print(" | len: ");
  Serial.println(msg.len);

  // Wait for a while and delete self
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  vTaskDelete(NULL);
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // put your setup code here, to run once:
  char task_name[12];
  Message msg;
  char text[20] = "All your base";

  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Counting Semaphore Demo---");

  // Create semaphores (initialize at 0)
  sem_params = xSemaphoreCreateCounting(num_tasks, 0);

  // Create message to use as argument common to all tasks
  strcpy(msg.body, text);
  msg.len = strlen(text);

  for (int i=0; i < num_tasks; i++) {
    // Generate unique name string for task
    sprintf(task_name, "Task %i", i);

    // Start task and pass argument (common Message struct)
    xTaskCreatePinnedToCore(myTask, task_name, 1024, 
                            (void *)&msg, 
                            1, NULL, app_cpu);
  } 

  // Wait for all tasks to read shared memory (&msg)
  for (int i=0; i < num_tasks; i++) {
    xSemaphoreTake(sem_params, portMAX_DELAY);
  }

  // Notify that all taks have been created
  Serial.println("All tasks created");
}

void loop() {
  // put your main code here, to run repeatedly:
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
