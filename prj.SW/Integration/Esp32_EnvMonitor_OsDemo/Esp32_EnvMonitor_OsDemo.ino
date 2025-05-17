/******************************************************/
/* Includes */
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

/******************************************************/
/* OS SETTINGS */
#define MINOR_TASK_SIZE_BYTES 8192     
#define MAJOR_TASK_SIZE_BYTES 16384    
#define TASK_PERIOD_2S 2000u
#define TASK_PERIOD_100MS 100u
#define TASK_PERIOD_10S 10000u

/* Semaphores */
SemaphoreHandle_t xInitSemaphore_C0;
SemaphoreHandle_t xInitSemaphore_C1;

/* Task Handlers */
TaskHandle_t Task_C0_Init_handle = NULL;
TaskHandle_t Task_C0_2s_handle = NULL;
TaskHandle_t Task_C0_100ms_handle = NULL;

TaskHandle_t Task_C1_Init_handle = NULL;
TaskHandle_t Task_C1_10s_handle = NULL;

/* Task Counters */
static bool init_flag_C0 = false;
static bool init_flag_C1 = false;
static volatile signed long long cnt_100ms_C0 = 0;
static volatile signed long long cnt_2s_C0 = 0;
static volatile signed long long cnt_10s_C1 = 0;

/* Critical area mux */
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

/* Debugger Settings */
#define DEBUG_SETUP true
#define DEBUG_INIT false
#define DEGUG_COUNTER true
#define DEGUG_DATA false
#define DEBUG_POT_PWM false
#define DEBUG_HTTP false

/******************************************************
 * FUNCTION PROTOTYPES */

/*----Std-Functions-------------------------------------------*/
void setup();
void loop();

/*----OS-Functions--------------------------------------------*/
void Task_C0_Init(void *pvParameters);
void Task_C0_2s(void *pvParameters);
void Task_C0_100ms(void *pvParameters);

void Task_C1_Init(void *pvParameters);
void Task_C1_10s(void *pvParameters);

/*----Saftety&Debug-Functions---------------------------------*/
void DebugPrint();


/******************************************************/
/* ARDUINO CORE FUNCTIONS */

void setup()
{

  /* Initial delay*/
  vTaskDelay(500 / portTICK_PERIOD_MS);

  /* Init serial */
  Serial.begin(115200);
#if DEBUG_SETUP
  Serial.println("[setup][Serial.begin(115200);]");
#endif

  /* Check if PSRAM exists */
  if (psramInit()) 
  {
#if DEBUG_SETUP
        Serial.println("[setup][PSRAM is initialized]");
        Serial.print("[setup][Total PSRAM:");Serial.print(ESP.getPsramSize());Serial.println("]");
        Serial.print("[setup][Free PSRAM:");Serial.print(ESP.getFreePsram());Serial.println("]");
#endif
  } 
  else 
  {
#if DEBUG_SETUP
        Serial.println("[setup][PSRAM is not available]");
#endif
  }

  /* Semaphore for C0 tasks */
  xInitSemaphore_C0 = xSemaphoreCreateCounting(2, 0);
  if (xInitSemaphore_C0 == NULL) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create C0 semaphore]");
#endif
    while(1); 
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created C0 semaphore]");
#endif
  }

  /* Semaphore for C1 tasks */
  xInitSemaphore_C1 = xSemaphoreCreateBinary();
  if (xInitSemaphore_C1 == NULL) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create C1 semaphore]");
#endif
    while(1); 
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created C1 semaphore]");
#endif
  }


  /* Create tasks with error checking */
  BaseType_t xReturned_Task_C0_Init;
  BaseType_t xReturned_Task_C0_2s;
  BaseType_t xReturned_Task_C0_100ms;
  BaseType_t xReturned_Task_C1_Init;
  BaseType_t xReturned_Task_C1_10s;

  /* Task_C0_Init */
  xReturned_Task_C0_Init = xTaskCreatePinnedToCore(&Task_C0_Init, "Task_C0_Init", MAJOR_TASK_SIZE_BYTES, NULL, 24, &Task_C0_Init_handle, 0);
  if (xReturned_Task_C0_Init != pdPASS) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create Task_C0_Init]");
#endif
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created Task_C0_Init]");
#endif
  }

  /* Task_C0_2s */
  xReturned_Task_C0_2s = xTaskCreatePinnedToCore(&Task_C0_2s, "Task_C0_2s", MAJOR_TASK_SIZE_BYTES, NULL, 10, &Task_C0_2s_handle, 0);
  if (xReturned_Task_C0_2s != pdPASS) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create Task_C0_2s]");
#endif
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created Task_C0_2s]");
#endif
  }

  /* Task_C0_100ms */
  xReturned_Task_C0_100ms = xTaskCreatePinnedToCore(&Task_C0_100ms, "Task_C0_100ms", MAJOR_TASK_SIZE_BYTES, NULL, 20, &Task_C0_100ms_handle, 0);
  if (xReturned_Task_C0_100ms != pdPASS) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create Task_C0_100ms]");
#endif
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created Task_C0_100ms]");
#endif
  }

  /* Task_C1_Init */
  xReturned_Task_C1_Init = xTaskCreatePinnedToCore(&Task_C1_Init, "Task_C1_Init", MINOR_TASK_SIZE_BYTES, NULL, 23, &Task_C1_Init_handle, 1);
  if (xReturned_Task_C1_Init != pdPASS) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create Task_C1_Init]");
#endif
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created Task_C1_Init]");
#endif
  }

  /* Task_C1_10s */
  xReturned_Task_C1_10s =  xTaskCreatePinnedToCore(&Task_C1_10s, "Task_C1_10s", MAJOR_TASK_SIZE_BYTES, NULL, 9, &Task_C1_10s_handle, 1);
  if (xReturned_Task_C1_10s != pdPASS) 
  {
#if DEBUG_SETUP
    Serial.println("[setup][Failed to create Task_C1_10s]");
#endif
  }
  else
  {
#if DEBUG_SETUP
    Serial.println("[setup][Created Task_C1_10s]");
#endif
  }

}

void loop()
{
  /* "Debugger" service */
  DebugPrint();
  vTaskDelay(500 / portTICK_PERIOD_MS);
}

/* End of Arduino Core Functions 
 ******************************************************/


/******************************************************
 * TASK IMPLEMENTATIONS */

/******************************* Task_C0_Init ***************************************
  Input:   void *pvParameters
  Output:  None
  Remarks: Task Init, calls init interfaces for Core 0, run once
           Gives sempahore to core 0 tasks after init done = true
************************************************************************************/
void Task_C0_Init(void *pvParameters)
{

  /* Task sub-routines*/

  /* End of task sub-routines*/

  /* Init flag */
  init_flag_C0 = true;

  /* After Init suspend task and release semaphore for Core 0 tasks */
  if(init_flag_C0)
  {
    if(xInitSemaphore_C0)
    {    
      xSemaphoreGive(xInitSemaphore_C0);
      xSemaphoreGive(xInitSemaphore_C0);
    }

    vTaskSuspend(NULL);
  }
}

/********************************* Task_C0_2s ***************************************
  Input:   void *pvParameters
  Output:  None
  Remarks: Task_C0_2s, calls sub-routines for Core 0 aka sensor measurement and LCD display data
           Takes sempahore from Task_C0_Init after init done = true
************************************************************************************/
void Task_C0_2s(void *pvParameters)
{
  if (xSemaphoreTake(xInitSemaphore_C0, portMAX_DELAY) == pdTRUE) 
  {

    for (;;)
    {
      
      /* Task counter */
      portENTER_CRITICAL(&myMutex);
      cnt_2s_C0 += 1;
      portEXIT_CRITICAL(&myMutex);

      /* Task sub-routines*/

      /* End of task sub-routines*/

      /* Task period */
      vTaskDelay(TASK_PERIOD_2S / portTICK_PERIOD_MS);
    }
  }
}


/******************************* Task_C0_100ms ***************************************
  Input:   void *pvParameters
  Output:  None
  Remarks: Task_C0_2s, calls sub-routines for Core 0 aka alert checks
           Takes sempahore from Task_C0_Init after init done = true
************************************************************************************/
void Task_C0_100ms(void *pvParameters)
{
  if (xSemaphoreTake(xInitSemaphore_C0, portMAX_DELAY) == pdTRUE) 
  {
    for (;;)
    {

      /* Task counter */
      portENTER_CRITICAL(&myMutex);
      cnt_100ms_C0 += 1;
      portEXIT_CRITICAL(&myMutex);

      /* Task sub-routines*/

      /* End of task sub-routines*/

      /* Task period */
      vTaskDelay(TASK_PERIOD_100MS / portTICK_PERIOD_MS);
    }
  }
}

/******************************* Task_C1_Init ***************************************
  Input:   void *pvParameters
  Output:  None
  Remarks: Task Init, calls init interfaces for Core 1, run once
           Gives sempahore to core 1 tasks after init done = true
************************************************************************************/
void Task_C1_Init(void *pvParameters)
{

  /* Task sub-routines*/

  /* End of task sub-routines*/

  /* Init flag */
  init_flag_C1 = true;

  /* After Init suspend task and release semaphore for Core 1 tasks */
  if(init_flag_C1)
  {
    if(xInitSemaphore_C1)
    {
      xSemaphoreGive(xInitSemaphore_C1);
      }
    vTaskSuspend(NULL);
  }

}

/******************************* Task_C1_10s ****************************************
  Input:   void *pvParameters
  Output:  None
  Remarks: Task_C1_10s, calls sub-routines for Core 1 aka Wifi_SendDataToGoogleSpreadsheets
           Takes sempahore from Task_C1_Init after init done = true
           It will not send data until Task_C0_2s is running, in order to send valid data 
           to the cloud service.
************************************************************************************/
void Task_C1_10s(void *pvParameters)
{
  if (xSemaphoreTake(xInitSemaphore_C1, portMAX_DELAY) == pdTRUE) 
  {
    for (;;)
    {

      /* Task counter */
      portENTER_CRITICAL(&myMutex);
      cnt_10s_C1 += 1;
      portEXIT_CRITICAL(&myMutex);


      /* Task sub-routines*/

      /* End of task sub-routines*/

      /* Task period */
      vTaskDelay(TASK_PERIOD_10S / portTICK_PERIOD_MS);
    }
  }
}

/* End of Task Functions 
 ******************************************************/

/******************************************************
 * Debugger Service IMPLEMENTATION */

void DebugPrint()
{
#if DEGUG_COUNTER
  Serial.print("[Task_C0_Init][stat:");Serial.print(init_flag_C1);Serial.print("]");Serial.print("_|_");
  Serial.print("[Task_C0_2s][cnt:");Serial.print(cnt_2s_C0);Serial.print("]");Serial.print("_|_");
  Serial.print("[Task_C0_100ms][cnt:");Serial.print(cnt_100ms_C0);Serial.print("]");Serial.print("_|_");
  Serial.print("[Task_C1_Init][stat:");Serial.print(init_flag_C1);Serial.print("]");Serial.print("_|_");
  Serial.print("[Task_C1_10s][cnt:");Serial.print(cnt_10s_C1);Serial.print("]");Serial.println();
#endif

}
 /* End of Debugger Service 
 ******************************************************/