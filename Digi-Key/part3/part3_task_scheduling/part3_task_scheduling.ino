// use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// some string to print
const char msg[] = "Task Scheduling";

// task handles
static TaskHandle_t task_1 = NULL;
static TaskHandle_t task_2 = NULL;

//**********************************************
// tasks

// task: print to Serial Terminal with lower priority
void startTask1(void *parameter) {
  // count number of charaters in string
  int msg_len = strlen(msg);

  // print string to Terminal
  while (1) {
    Serial.println();
    for (int i = 0; i < msg_len; i++) {
      Serial.print(msg[i]);
    }
    Serial.println();

    // put the task into the blocked state
    // RTOS will return this task to the ready state when the delay time is up
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1000 ms
  }
}

// task: print to Serial Terminal with higher priority
void startTask2(void *parameter) {
  while (1) {
    Serial.print('*');
    vTaskDelay(100 / portTICK_PERIOD_MS); // 100 ms
  }
}

// *******************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {
  // put your setup code here, to run once:
  // configure Serial (go slow so we can catch the preemption)
  Serial.begin(300);

  // Wait a moment to start (so we dont't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Task Demo---");

  // print self priority
  Serial.print("Setup and loop task running on core ");
  Serial.print(xPortGetCoreID());
  Serial.print(" with priority ");
  Serial.print(uxTaskPriorityGet(NULL));

  // task to run forever
  xTaskCreatePinnedToCore(startTask1,
                          "Task 1",
                          1024,
                          NULL,
                          1,
                          &task_1,
                          app_cpu);

  // task to run once with higher priority
  xTaskCreatePinnedToCore(startTask2,
                          "Task 2",
                          1024,
                          NULL,
                          2,
                          &task_2,
                          app_cpu);                     
}

void loop() {
  // put your main code here, to run repeatedly:
  // Suspend the higher priority task for some intervals
  for (int i = 0; i < 3; i++) {
    vTaskSuspend(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    vTaskResume(task_2);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }

  // Delete the lower priority task
  if (task_1 != NULL) {
    vTaskDelete(task_1);
    task_1 = NULL;
  }

  Serial.println();
}
