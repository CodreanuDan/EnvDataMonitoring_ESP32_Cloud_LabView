/******************************************************
 * Includes */

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

/******************************************************
 * OS SETTINGS */

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

/* Critical area mux for task counters*/
portMUX_TYPE myMutex = portMUX_INITIALIZER_UNLOCKED;

/* End of OS SETTINGS 
 *****************************************************/

/******************************************************
 * Wifi Settings */
const char *ssid = "CDIDC";
const char *password = "SkodaOctavia2005";

/* Connection flag */
bool wifi_connected;

#define LED_BUILTIN 0x2

/* End of Wifi Settings 
 *****************************************************/

/* Debugger Settings */
#define DEBUG_SETUP true
#define DEBUG_INIT false
#define DEGUG_COUNTER true
#define DEGUG_DATA false
#define DEBUG_POT_PWM false
#define DEBUG_HTTP false


/******************************************************
 * FUNCTION PROTOTYPES and Auxiliary Data structures */

/*
* Init status functions returns
*/
typedef enum {
  INIT_FAIL = 0,
  INIT_OK = 1
} INIT_STATUS;

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

/*----Init-Functions------------------------------------------*/
INIT_STATUS Wifi_Init();

/*----Wifi-Functions------------------------------------------*/
bool Wifi_Callback(bool (*fptr)(void));
bool Wifi_SetupNetworkConnection();

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

  /* Init WiFi connection */
  INIT_STATUS wifi_status = Wifi_Init();
  if (wifi_status == INIT_OK) 
  {
    init_flag_C1 = true;
  }

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
  if (!init_flag_C1)
  {
    Serial.println("[WiFi Init] Not done in setup(). Suspending.");
    vTaskSuspend(NULL);
  }

  if (init_flag_C1)
  {   
    if (xInitSemaphore_C1)
    {
      xSemaphoreGive(xInitSemaphore_C1);
    }
  }

  vTaskSuspend(NULL);
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
  Serial.print("[Time: "); Serial.print(millis()); Serial.println("ms] ");
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


/******************************************************
 * INIT FUNCTIONS */


/********************************* Wifi_Init ****************************************
  Input:   None
  Output:  INIT_STATUS
  Remarks: This function initites Wifi connection using a callback
************************************************************************************/
INIT_STATUS Wifi_Init()
{
  if (Wifi_Callback(&Wifi_SetupNetworkConnection)) 
  {
    return INIT_OK;
  } 
  else 
  {
    return INIT_FAIL;
  }
}

/* End of Init Functions 
 ******************************************************/

/******************************************************
 * WiFi FUNCTIONS */

/*
 * Wifi_Callback - used to call wifi functions that exist or will be implemented in the future
 */
bool Wifi_Callback(bool (*fptr)(void))
{
  if (fptr) 
  {
    return fptr();
  }
  return false;
}

/****************************  Wifi_SetupNetworkConnection ***************************
  Input:   None
  Output:  bool
  Remarks: This function attempts to connect to a wifi network
           If connection is succesful the wifi_connected flag is set true 
************************************************************************************/ 
bool Wifi_SetupNetworkConnection()
{
  int timeout = 0;
  const int MAX_TIMEOUT = 20;
  wifi_connected = false;

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 

  WiFi.mode(WIFI_STA); 

#if DEBUG_HTTP
  Serial.println("[WiFi] Scanning for networks...");
#endif

  int n = WiFi.scanNetworks();
  if (n <= 0) 
  {
#if DEBUG_HTTP
    Serial.println("[WiFi] No networks found.");
#endif
  } 
  else 
  {
#if DEBUG_HTTP
    Serial.printf("[WiFi] %d networks found:\n", n);
    for (int i = 0; i < n; ++i) 
    {
      Serial.printf("  [%d] %s (RSSI: %d)\n", i + 1, WiFi.SSID(i).c_str(), WiFi.RSSI(i));
    }
#endif
  }

#if DEBUG_HTTP
  Serial.printf("[WiFi] Attempting to connect to: %s\n", ssid);
#endif

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && timeout < MAX_TIMEOUT) 
  {
    delay(500);
    timeout++;
#if DEBUG_HTTP
    Serial.print(".");
#endif
  }

  Serial.println(); 

  if (WiFi.status() == WL_CONNECTED) 
  {
    wifi_connected = true;
    digitalWrite(LED_BUILTIN, HIGH);

#if DEBUG_HTTP
    Serial.printf("[WiFi] Connected to: %s\n", WiFi.SSID().c_str());
    Serial.printf("[WiFi] IP Address: %s\n", WiFi.localIP().toString().c_str());
#endif
    WiFi.scanDelete();
    return true;
  } 
  else 
  {
#if DEBUG_HTTP
    Serial.println("[WiFi] Failed to connect.");
#endif
    WiFi.scanDelete();
    return false;
  }
}


