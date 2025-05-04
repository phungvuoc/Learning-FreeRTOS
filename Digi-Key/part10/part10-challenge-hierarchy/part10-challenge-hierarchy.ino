/**
 * Part 10 - challenge - Deadlock - Hierarchy
 * ESP32 Dining Philosophers
 * 
 * The classic "Dining Philosophers" problem in FreeRTOS form.
 * 
 * Based on http://www.cs.virginia.edu/luther/COA2/S2019/pa05-dp.html
 * 
 * Date: February 8, 2021
 * Author: Shawn Hymel
 * License: 0BSD
 */

// You'll likely need this on vanilla FreeRTOS
//#include semphr.h

// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Settings
enum { NUM_TASKS = 5 };           // Number of tasks (philosophers)
enum { TASK_STACK_SIZE = 2048 };  // Bytes in ESP32, words in vanilla FreeRTOS

// Globals
static SemaphoreHandle_t bin_sem;   // Wait for parameters to be read
static SemaphoreHandle_t done_sem;  // Notifies main task when done
static SemaphoreHandle_t chopstick[NUM_TASKS]; // array

//*****************************************************************************
// Tasks

// The only task: eating
void eat(void *parameters) {

  int num;
  char buf[50];

  // Copy parameter and increment semaphore count
  num = *(int *)parameters;
  xSemaphoreGive(bin_sem);

  int left_num;
  int right_num;

  if (num < (NUM_TASKS - 1)) {
    left_num = num;
  } else {
    left_num = num - 1;
  }
  right_num = (left_num + 1) % NUM_TASKS;

  // Solution:
  // Assign priority: pick up lower-numbered chopstick first
  /*
  if (num < (num + 1) % NUM_TASKS) {
    left_num = num;
    right_num = (num + 1) % NUM_TASKS;
  } else {
    // swap
    left_num = (num + 1) % NUM_TASKS;
    right_num = num;
  }
  */

  // Take left chopstick
  xSemaphoreTake(chopstick[left_num], portMAX_DELAY);
  sprintf(buf, "Philosopher %i took left chopstick %i", num, left_num);
  Serial.println(buf);

  // Add some delay to force deadlock
  vTaskDelay(1 / portTICK_PERIOD_MS);

  // Take right chopstick
  xSemaphoreTake(chopstick[right_num], portMAX_DELAY);
  sprintf(buf, "Philosopher %i took right chopstick %i", num, right_num);
  Serial.println(buf);

  // Do some eating
  sprintf(buf, "Philosopher %i is eating", num);
  Serial.println(buf);
  vTaskDelay(10 / portTICK_PERIOD_MS);

  // Put down right chopstick
  xSemaphoreGive(chopstick[right_num]);
  sprintf(buf, "Philosopher %i returned right chopstick %i", num, right_num);
  Serial.println(buf);

  // Put down left chopstick
  xSemaphoreGive(chopstick[left_num]);
  sprintf(buf, "Philosopher %i returned left chopstick %i", num, left_num);
  Serial.println(buf);

  // Notify main task and delete self
  xSemaphoreGive(done_sem);
  vTaskDelete(NULL);
}

//*****************************************************************************
// Main (runs as its own task with priority 1 on core 1)

void setup() {

  char task_name[20];

  // Configure Serial
  Serial.begin(115200);

  // Wait a moment to start (so we don't miss Serial output)
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println();
  Serial.println("---FreeRTOS Dining Philosophers Challenge---");

  // Create kernel objects before starting tasks
  bin_sem = xSemaphoreCreateBinary();
  done_sem = xSemaphoreCreateCounting(NUM_TASKS, 0);
  for (int i = 0; i < NUM_TASKS; i++) {
    chopstick[i] = xSemaphoreCreateMutex();
  }

  // Have the philosphers start eating
  for (int i = 0; i < NUM_TASKS; i++) {
    sprintf(task_name, "Philosopher %i", i);
    xTaskCreatePinnedToCore(eat,
                            task_name,
                            TASK_STACK_SIZE,
                            (void *)&i,
                            1,
                            NULL,
                            app_cpu);
    xSemaphoreTake(bin_sem, portMAX_DELAY);
  }


  // Wait until all the philosophers are done
  for (int i = 0; i < NUM_TASKS; i++) {
    xSemaphoreTake(done_sem, portMAX_DELAY);
  }

  // Say that we made it through without deadlock
  Serial.println("Done! No deadlock occurred!");
}

void loop() {
  // Do nothing in this task
}