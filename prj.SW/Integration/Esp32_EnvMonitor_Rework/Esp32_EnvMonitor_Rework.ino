
/******************************************************
 * Includes */
#include <Arduino.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include <driver/adc.h>
#include <DHT.h>

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

 /* End of Includes 
 ******************************************************/

/******************************************************
 * PINOUT */
#define DHT_22_SDA_PIN_IN_25 25
#define MQ_2_AO_PIN_IN_ADC_35 35
#define BUZZER_PIN_OUT_33 33
#define POTENTIOMETER_PIN_IN_34 34
#define RED_LED_PIN_OUT_32 32
#define GREEN_LED_PIN_OUT_26 26
#define I2C_SDA_PIN_21 21
#define I2C_SCL_PIN_22 22
#define LED_BUILTIN 2

/* End of PINOUT 
 ******************************************************/

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
 * SYS STATUS */
static volatile bool co_status;
static volatile bool lpg_status;
static volatile bool smoke_status;
static volatile bool alarm_status;
 /* End of SYS STATUS 
 ******************************************************/

/******************************************************
 * ADC CONFIG */
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTENUATION ADC_ATTEN_DB_11
#define DEFAULT_VREF 1100u
#define ADC_MAX_RESOLUTION 4095.0f

typedef enum {
  CAL_TWO_POINT = 0,
  CAL_VREF_EFUSE,
  CAL_VREF_DEF,
  CAL_FAILED
} Adc_Calib_Values;
uint8_t Adc_CalibrationSettingValue;

/* End of ADC CONFIG 
 ******************************************************/

/******************************************************
 * Wifi Settings */

const char *ssid = "CDIDC";
const char *password = "SkodaOctavia2005";

/* Connection flag */
bool wifi_connected;

/* Google Scripts Location */
String googleScriptURL = "https://script.google.com/a/macros/student.tuiasi.ro/s/AKfycbxoNbKtwmbg_m_UjYyqUhhDF4yr9LLUXoEMrd9-jK2HmoFF6gF_khVkUFtDfsQDn1s/exec";

/* End of Wifi Settings 
 *****************************************************/


/******************************************************
 * Buzzer volume control CONFIG */

/* PWM Channel 0-15 */
#define LEDC_CHANNEL 0

/* Resolutuion in bits (8 bits = 0-255 duty cycle) */    
#define LEDC_TIMER_RESOLUTION   8 

/* Buzzer Freq (Hz) */   
#define BUZZER_FREQ  1000  

typedef struct{
  /* (0 - 4095) */
  volatile uint16_t potValue; 
  /* 0-4095 to 0-255 (duty cycle for 8 bits) */
  volatile uint8_t dutyCycle; 
} VolumeSettings;
VolumeSettings volumeSettings ;

/* End of Buzzer volume control CONFIG 
 ******************************************************/

/******************************************************
 * MQ2 CONFIG */

/*
 * define the load resistance on the board, in kilo ohms
 * RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
 * which is derived from the chart in datasheet
 */
int RL_VALUE=5;                                     
float RO_CLEAN_AIR_FACTOR=9.83;                     
                                                    
/***********************MQ2_Software Related Macros************************************/
/*
 * define how many samples you are going to take in the calibration phase
 */
int CALIBARAION_SAMPLE_TIMES=500;                   

/*
 * define the time interal(in milisecond) between each samples in the
 * cablibration phase
 */
int CALIBRATION_SAMPLE_INTERVAL=10;                 
                                                    
/*
 * define how many samples you are going to take in normal operation
 */                                                    
int READ_SAMPLE_INTERVAL=10;                        

/*
 * define the time interal(in milisecond) between each samples in 
 * normal operation
 */
int READ_SAMPLE_TIMES=10;                           
                                                    
/**********************MQ2_Application Related Macros**********************************/
#define         GAS_LPG             0   
#define         GAS_CO              1   
#define         GAS_SMOKE           2    

#define         GAS_CO_ALARM_LIMIT       50
#define         GAS_LPG_ALARM_LIMIT      2000
#define         GAS_SMOKE_ALARM_LIMIT    200
 
/*****************************MQ2_Globals***********************************************/

/*
 * two points are taken from the curve.
 * with these two points, a line is formed which is "approximately equivalent"
 * to the original curve. 
 * data format:{ x, y, slope}; point1: (lg200, 0.21), point2: (lg10000, -0.59)
 */
float           LPGCurve[3]  =  {2.3,0.21,-0.47};   
 
/*
 * two points are taken from the curve.
 * with these two points, a line is formed which is "approximately equivalent" 
 * to the original curve.
 * data format:{ x, y, slope}; point1: (lg200, 0.72), point2: (lg10000,  0.15) 
 */
float           COCurve[3]  =  {2.3,0.72,-0.34};    

/*
 * two points are taken from the curve.
 * with these two points, a line is formed which is "approximately equivalent"
 * to the original curve.
 * data format:{ x, y, slope}; point1: (lg200, 0.53), point2: (lg10000,  -0.22) 
 */
float           SmokeCurve[3] ={2.3,0.53,-0.44};    
                                                                                                                                                                                                 
/*
 * Ro is initialized to 10 kilo ohms
 */
float           Ro           =  10;

 /* End of MQ2 CONFIG 
 ******************************************************/

/******************************************************
 * LCD CONFIG */

#define LCD_ADDR 0x27
#define LCD_COLUMNS 16u
#define LCD_ROWS 2u
/* Pointer used for INIT function of the LCD */
LiquidCrystal_I2C* lcd_ptr = nullptr;

 /* End of LCD CONFIG 
  ******************************************************/

/******************************************************
 * DHT CONFIG */

#define DHT_TYPE DHT22
#define DHT_PIN  DHT_22_SDA_PIN_IN_25
/* Pointer used for INIT function of the DHT Sensor */
DHT* dht_ptr = nullptr;

 /* DHT CONFIG 
 ******************************************************/

/*
 * Struct to keep the byte vectors for custom chars
 */
typedef struct {
  byte thermometer[8];
  byte wifi[8];
  byte humidity[8];
  byte heart[8];
  byte smoke[8];
  byte co[8];
  byte lpg[8];
  byte adc[8];
} CustomCharsByte;

/*
 * Vector to keep the byte vectors for custom chars
 */
CustomCharsByte customCharsByte = {
  {  
    0b00100,
    0b01010,
    0b01110,
    0b01010,
    0b10111,
    0b10001,
    0b10111,
    0b01110
  },  // thermometer
  {  
    0b11100,
    0b11000,
    0b10100,
    0b00010,
    0b01000,
    0b00101,
    0b00011,
    0b00111
  },  // wifi
  {
    0b00000,
    0b00100,
    0b01010,
    0b01010,
    0b10001,
    0b10011,
    0b10111,
    0b01110
  },  // humidity
  {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000
  },  // heart
  {
    0b00110,
    0b01001,
    0b00100,
    0b10010,
    0b01100,
    0b00000,
    0b11111,
    0b11111
  }, // smoke
  {
    0b01110,
    0b10001,
    0b10000,
    0b10001,
    0b01110,
    0b00000,
    0b11111,
    0b11111
  }, // co
  {
    0b01110,
    0b10000,
    0b10111,
    0b10001,
    0b01110,
    0b00000,
    0b11111,
    0b11111
  }, // lpg
  {
    0b00100,
    0b00100,
    0b00100,
    0b01010,
    0b10001,
    0b10001,
    0b10001,
    0b11111
  }, // adc

};

/*
 * Struct to keep the custom chars
 */
typedef struct {
  uint8_t thermometer;
  uint8_t wifi;
  uint8_t humidity;
  uint8_t heart;
  uint8_t smoke;
  uint8_t co;
  uint8_t lpg;
  uint8_t adc;
} CustomChars;
CustomChars customChars;

/*
 * Struct to keep sensor data and to acces it from all SW interfaces
 */
typedef struct{
  float temperature;
  float humidity;
  float heatIndex;
  float adcMq2Voltage;
  float Ro;
  long  iPPM_LPG;
  long  iPPM_CO;
  long  iPPM_Smoke;
  uint32_t adcRawValue;
} SensorData;
SensorData sensorData;

/*
 * Init status functions returns
 */
typedef enum {
  INIT_FAIL = 0,
  INIT_OK = 1
} INIT_STATUS;


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

/*----Init-Functions------------------------------------------*/
INIT_STATUS Port_Init();
INIT_STATUS Adc_Init(uint8_t sensorPin, uint8_t adcResolution, uint8_t adcAttenuation);
INIT_STATUS I2C_Init(uint8_t sda_pin, uint8_t scl_pin, uint32_t clk_spd);
INIT_STATUS LCD_Init(uint8_t lcd_addr, uint8_t lines, uint8_t rows, CustomCharsByte* charsByte, CustomChars* chars);
INIT_STATUS MQCalibration(int mq_pin, SensorData* data);
INIT_STATUS Dht_Init(uint8_t dht_pin, uint8_t dht_type);
INIT_STATUS Wifi_Init();

INIT_STATUS Wifi_Init();
/*----MQ-Functions--------------------------------------------*/
float MQResistanceCalculation(int raw_adc);
float MQRead(int mq_pin);
void MQGetGasPercentage(float rs_ro_ratio, int gas_id, SensorData* data);
long MQGetPercentage(float rs_ro_ratio, float *pcurve);
void MQGetVoltage(uint32_t adc_raw_value, SensorData* data);

/*----DHT-Functions-------------------------------------------*/
void DHT_Measurement(SensorData* data);
void DHT_Callback(void (*fptr)(SensorData* data));
void DHT_ReadTemperature_Callback(SensorData* data);
void DHT_ReadHumidity_Callback(SensorData* data);
void DHT_ComputeHeatIndex_Callback(SensorData* data);

/*----LCD-Functions-------------------------------------------*/
void LcdDisplayData(SensorData* data, CustomChars* chars);

/*----Saftety&Debug-Functions---------------------------------*/
void LimitAlertCheck(SensorData* data, VolumeSettings* settings);
void DebugPrint(SensorData* data, VolumeSettings* settings);

/*----Wifi-Functions------------------------------------------*/
bool Wifi_Callback(bool (*fptr)(void));
bool Wifi_SetupNetworkConnection();
void Wifi_SendDataToGoogleSpreadsheets(SensorData* data);

/*----Volume-Settings-----------------------------------------*/
void ManageBuzzerVolume(VolumeSettings* settings, uint8_t pot_pin);

/******************************************************
 * ARDUINO CORE FUNCTIONS */

void setup()
{
  vTaskDelay(500 / portTICK_PERIOD_MS); 
  Serial.begin(115200);

  /* Init WiFi connection */
  INIT_STATUS wifi_status = Wifi_Init();
  if (wifi_status == INIT_OK) 
  {
    init_flag_C1 = true;
  }

  if (psramInit()) 
  {
        Serial.println("[setup]PSRAM is initialized");
        Serial.print("[setup][Total PSRAM: ]");
        Serial.println(ESP.getPsramSize());
        Serial.print("[setup][Free PSRAM: ]");
        Serial.println(ESP.getFreePsram());
  } 
  else 
  {
        Serial.println("[setup][PSRAM is not available]");
  }

  if (lcd_ptr == nullptr) 
  {
    Serial.println("[setup][!lcd_ptr]");
  }
  if (dht_ptr == nullptr) 
  {
    Serial.println("[setup][!dht_ptr]");
  }

  /* Semaphore for C0 tasks */
  xInitSemaphore_C0 = xSemaphoreCreateCounting(2, 0);
  if (xInitSemaphore_C0 == NULL) 
  {
    Serial.println("[setup][Failed to create C0 semaphore]");
    while(1); 
  }
  else
  {
    Serial.println("[setup][Created C0 semaphore]");
  }

  /* Semaphore for C1 tasks */
  xInitSemaphore_C1 = xSemaphoreCreateBinary();
  if (xInitSemaphore_C1 == NULL) 
  {
    Serial.println("[setup][Failed to create C1 semaphore]");
    while(1); 
  }
  else
  {
    Serial.println("[setup][Created C1 semaphore]");
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
    Serial.println("[setup][Failed to create Task_C0_Init]");
  }
  else
  {
    Serial.println("[setup][Created Task_C0_Init]");
  }

  /* Task_C0_2s */
  xReturned_Task_C0_2s = xTaskCreatePinnedToCore(&Task_C0_2s, "Task_C0_2s", MAJOR_TASK_SIZE_BYTES, NULL, 10, &Task_C0_2s_handle, 0);
  if (xReturned_Task_C0_2s != pdPASS) 
  {
    Serial.println("[setup][Failed to create Task_C0_2s]");
  }
  else
  {
    Serial.println("[setup][Created Task_C0_2s]");
  }

  /* Task_C0_100ms */
  xReturned_Task_C0_100ms = xTaskCreatePinnedToCore(&Task_C0_100ms, "Task_C0_100ms", MAJOR_TASK_SIZE_BYTES, NULL, 20, &Task_C0_100ms_handle, 0);
  if (xReturned_Task_C0_100ms != pdPASS) 
  {
    Serial.println("[setup][Failed to create Task_C0_100ms]");
  }
  else
  {
    Serial.println("[setup][Created Task_C0_100ms]");
  }

  /* Task_C1_Init */
  xReturned_Task_C1_Init = xTaskCreatePinnedToCore(&Task_C1_Init, "Task_C1_Init", MINOR_TASK_SIZE_BYTES, NULL, 23, &Task_C1_Init_handle, 1);
  if (xReturned_Task_C1_Init != pdPASS) 
  {
    Serial.println("[setup][Failed to create Task_C1_Init]");
  }
  else
  {
    Serial.println("[setup][Created Task_C1_Init]");
  }

  /* Task_C1_10s */
  xReturned_Task_C1_10s =  xTaskCreatePinnedToCore(&Task_C1_10s, "Task_C1_10s", MAJOR_TASK_SIZE_BYTES, NULL, 9, &Task_C1_10s_handle, 1);
  if (xReturned_Task_C1_10s != pdPASS) 
  {
    Serial.println("[setup][Failed to create Task_C1_10s]");
  }
  else
  {
    Serial.println("[setup][Created Task_C1_10s]");
  }

}

void loop()
{
  /* "Debugger" service */
  DebugPrint(&sensorData, &volumeSettings);
  vTaskDelay(250 / portTICK_PERIOD_MS);
}

/* End of Arduino Core Functions 
 ******************************************************/

/******************************************************
 * INIT FUNCTIONS */


/******************************** Port_Init ****************************************
  Input:   None
  Output:  INIT_STATUS
  Remarks: This function initiates the ports used in the project  
************************************************************************************/
INIT_STATUS Port_Init()
{
  /* Red LED */
  pinMode(RED_LED_PIN_OUT_32, OUTPUT);
  /* Green LED */
  pinMode(GREEN_LED_PIN_OUT_26, OUTPUT);
  /* Set buzzer pin to PWM output using internal PWM esp module*/
  pinMode(BUZZER_PIN_OUT_33, OUTPUT);
  // ledcSetup(LEDC_CHANNEL, BUZZER_FREQ, LEDC_TIMER_RESOLUTION);
  // ledcAttachPin(BUZZER_PIN_OUT_33, LEDC_CHANNEL);
  /*-----------------------------------------------------------*/
  /* Potentiometer */
  pinMode(POTENTIOMETER_PIN_IN_34, INPUT_PULLUP);
  /* DHT pin digital input */
  pinMode(DHT_22_SDA_PIN_IN_25, INPUT_PULLUP);
  /* MQ2 Adc pin analog input*/
  pinMode(MQ_2_AO_PIN_IN_ADC_35, INPUT_PULLUP);

  vTaskDelay(10 / portTICK_PERIOD_MS);

  return INIT_OK;
}

/********************************** Adc_Init ****************************************
  Input:   uint8_t sensorPin, uint8_t adcResolution, uint8_t adcAttenuation
  Output:  INIT_STATUS
  Remarks: This function initiates the ADC port for MQ2 sensor  
************************************************************************************/
INIT_STATUS Adc_Init(uint8_t sensorPin, uint8_t adcResolution, uint8_t adcAttenuation)
{
  // pinMode(sensorPin, INPUT);

  /* Set the ADC resolution to 12 bits */
  analogReadResolution(adcResolution);

  /* Set the ADC attenuation  for the LM35 sensor pin
    * ADC_0db:   ~~> 1.1V
    * ADC_2_5db: ~~> 1.5V
    * ADC_6db:   ~~> 2.2V
    * ADC_11db:  ~~> 3.3V 
    * {!!!} use static cast for adcAttenuation beacuse the func expects elements from adc_attenuation_t enum 
    */
  analogSetPinAttenuation(sensorPin, static_cast<adc_attenuation_t>(adcAttenuation));

#if DEBUG_INIT
  Serial.println("ADC calibrated");
#endif

  vTaskDelay(10 / portTICK_PERIOD_MS);
  return INIT_OK;
}

/********************************** I2C_Init ****************************************
  Input:   uint8_t sda_pin, uint8_t scl_pin, uint32_t clk_spd
  Output:  INIT_STATUS
  Remarks: This function initiates the I2C protocol and scans for devices connected to the bus
************************************************************************************/
INIT_STATUS I2C_Init(uint8_t sda_pin, uint8_t scl_pin, uint32_t clk_spd)
{
  /* Init I2C */
  Wire.begin(sda_pin, scl_pin);

  /* Set clock speed to 100 MHz */
  Wire.setClock(clk_spd);
 
  /* I2C scanning */
  byte error, address;
  uint8_t nDevices = 0;

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {

#if DEBUG_INIT
      Serial.print("I2C device found at address 0x");
      Serial.println(address, HEX);
#endif

      nDevices++;
      return INIT_OK;
    }
  }

  if (nDevices == 0)
  {

#if DEBUG_INIT
    Serial.println("No I2C devices found");
#endif

    return INIT_FAIL;
  }

  return INIT_OK;
  
}

/********************************** LCD_Init ****************************************
  Input:   uint8_t lcd_addr, uint8_t lines, uint8_t rows, CustomCharsByte* charsByte, CustomChars* chars
  Output:  INIT_STATUS
  Remarks: This function initiates the LCD display and creats custom chars also must be called
           before MQ_Calibration in order to pre-heat the sensor resistance as an animation is provided
************************************************************************************/
INIT_STATUS LCD_Init(uint8_t lcd_addr, uint8_t lines, uint8_t rows, CustomCharsByte* charsByte, CustomChars* chars)
{

  /* Create LCD object using the pointer for LCD */
  lcd_ptr = new LiquidCrystal_I2C(lcd_addr, lines, rows);
  if (!lcd_ptr) 
  {
    Serial.println("Failed to allocate memory for LCD");
    return INIT_FAIL;
  }

  /*----------------------------------------*/
  /*
   * Start LCD
   */ 
  lcd_ptr->init(); 
  lcd_ptr->backlight();
  lcd_ptr->clear();

  /*----------------------------------------*/
  /*
   * Create custom chars
   */ 
  lcd_ptr->createChar(0, charsByte->thermometer);
  chars->thermometer = 0;
  lcd_ptr->createChar(1, charsByte->wifi);
  chars->wifi = 1;
  lcd_ptr->createChar(2, charsByte->humidity);
  chars->humidity = 2;
  lcd_ptr->createChar(3, charsByte->heart);
  chars->heart = 3;
  lcd_ptr->createChar(4, charsByte->smoke);
  chars->smoke = 4;
  lcd_ptr->createChar(5, charsByte->co);
  chars->co = 5;
  lcd_ptr->createChar(6, charsByte->lpg);
  chars->lpg = 6;
  lcd_ptr->createChar(7, charsByte->adc);
  chars->adc = 7;

  /*----------------------------------------*/
  /*
   * Show loading screen
   */    
  lcd_ptr->setCursor(0, 0);     
  lcd_ptr->print("Start MQ Warmpup");
  lcd_ptr->setCursor(0, 1);   
  for(uint8_t i = 0; i < 16; i++)
  {
    lcd_ptr->print(".");
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  lcd_ptr->clear();

  /*----------------------------------------*/
  vTaskDelay(10 / portTICK_PERIOD_MS);
  return INIT_OK; 

}

/********************************** DHT_Init ****************************************
  Input:   uint8_t dht_pin, uint8_t dht_type
  Output:  INIT_STATUS
  Remarks: This function creates the DHT object using the pointe
************************************************************************************/
INIT_STATUS Dht_Init(uint8_t dht_pin, uint8_t dht_type)
{
  /* Create DHT object using the pointer for DHT - DHT pin and DHT type: 11/22 */
  dht_ptr = new DHT(dht_pin, dht_type);
  if (!dht_ptr) 
  {
    Serial.println("Failed to allocate memory for DHT");
    return INIT_FAIL;
  }
  dht_ptr->begin();
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  return INIT_OK;
}

/********************************* Wifi_Init ****************************************
  Input:   None
  Output:  INIT_STATUS
  Remarks: This function initites Wifi connection using a callback
************************************************************************************/
INIT_STATUS Wifi_Init()
{
  /* Init Wifi with Wifi callback */
  Wifi_Callback(&Wifi_SetupNetworkConnection);
  return INIT_OK;
}

/***************************** MQCalibration ****************************************
  Input:   mq_pin - analog channel
  Output:  INIT_STATUS; (*Ro of the sensor in the struct)
  Remarks: This function assumes that the sensor is in clean air. It use  
           MQResistanceCalculation to calculates the sensor resistance in clean air 
           and then divides it with RO_CLEAN_AIR_FACTOR. RO_CLEAN_AIR_FACTOR is about 
           10, which differs slightly between different sensors.
************************************************************************************/ 
INIT_STATUS MQCalibration(int mq_pin, SensorData* data)
{
  int i;
  float val=0;

  for (i=0;i<CALIBARAION_SAMPLE_TIMES;i++) 
  {            
    val += MQResistanceCalculation(analogRead(mq_pin));
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }

  /* calculate the average value */
  val = val/CALIBARAION_SAMPLE_TIMES;  

  /* divided by RO_CLEAN_AIR_FACTOR yields the Ro */
  val = val/RO_CLEAN_AIR_FACTOR;    

  /* according to the chart in the datasheet */                                       
  data->Ro = val;                                       

  if(val)
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

  /* Port Init */
  Port_Init();

  /* Adc Init */
  Adc_Init(MQ_2_AO_PIN_IN_ADC_35, 12, ADC_11db);

  /* I2C Init */
  I2C_Init(I2C_SDA_PIN_21, I2C_SCL_PIN_22, 100000);

  /* LCD Init */
  LCD_Init(LCD_ADDR, LCD_COLUMNS, LCD_ROWS, &customCharsByte, &customChars);

  /* MQ  Init */
  MQCalibration(MQ_2_AO_PIN_IN_ADC_35, &sensorData);

  /* DHT Init */
  Dht_Init(DHT_PIN, DHT_TYPE);

#if DEBUG_INIT
  Serial.println("Calculated Ro:" + String(sensorData.Ro));
#endif

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

      float _Ro = sensorData.Ro;

#if DEBUG_DATA
      Serial.println("*Ro: " + String(_Ro));
#endif

      /* Task sub-routines*/

      /* MQ 2 measurements */
      MQGetGasPercentage(MQRead(MQ_2_AO_PIN_IN_ADC_35) / _Ro, GAS_LPG, &sensorData);
      MQGetGasPercentage(MQRead(MQ_2_AO_PIN_IN_ADC_35) / _Ro, GAS_CO, &sensorData);
      MQGetGasPercentage(MQRead(MQ_2_AO_PIN_IN_ADC_35) / _Ro, GAS_SMOKE, &sensorData);
      MQGetVoltage(analogRead(MQ_2_AO_PIN_IN_ADC_35), &sensorData);

      /* DHT measurements */
      DHT_Measurement(&sensorData);

      /* LCD function */
      LcdDisplayData(&sensorData, &customChars);

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

      // ManageBuzzerVolume(&volumeSettings, POTENTIOMETER_PIN_IN_34);

      /* LimitAlertCheck (alarm and led signals) */
      LimitAlertCheck(&sensorData, &volumeSettings);

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

      Wifi_SendDataToGoogleSpreadsheets(&sensorData);

      /* End of task sub-routines*/

      /* Task period */
      vTaskDelay(TASK_PERIOD_10S / portTICK_PERIOD_MS);
    }
  }
}

/* End of Task Functions 
 ******************************************************/


/******************************************************
 * MQ Functions */

/****************** MQResistanceCalculation ****************************************
  Input:   raw_adc - raw value read from adc, which represents the voltage
  Output:  the calculated sensor resistance
  Remarks: The sensor and the load resistor forms a voltage divider. Given the voltage
          across the load resistor and its resistance, the resistance of the sensor
          could be derived.
************************************************************************************/ 
float MQResistanceCalculation(int raw_adc)
{
  return ( ((float)RL_VALUE*(ADC_MAX_RESOLUTION-raw_adc)/raw_adc));
}
 
/*****************************  MQRead *********************************************
  Input:   mq_pin - analog channel
  Output:  Rs of the sensor 
  Remarks: This function use MQResistanceCalculation to caculate the sensor resistenc (Rs).
          The Rs changes as the sensor is in the different consentration of the target
          gas. The sample times and the time interval between samples could be configured
          by changing the definition of the macros.
************************************************************************************/ 
float MQRead(int mq_pin)
{
  int i;
  float rs=0;
  
  /* reading with oversampling */
  for (i=0;i<READ_SAMPLE_TIMES;i++) 
  {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }
 
  rs = rs/READ_SAMPLE_TIMES;
 
  return rs;  
}
 
/*****************************  MQGetGasPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
           gas_id      - target gas type
  Output:  ppm of the target gas -> SensorData struct
  Remarks: This function passes different curves to the MQGetPercentage function which 
           calculates the ppm (parts per million) of the target gas.
************************************************************************************/ 
void MQGetGasPercentage(float rs_ro_ratio, int gas_id, SensorData* data)
{
  if ( gas_id == GAS_LPG ) 
  {
    long gasPercentage = MQGetPercentage(rs_ro_ratio,LPGCurve);
    if(gasPercentage)
    {
      data->iPPM_LPG = gasPercentage;
    }
    else
    {
      data->iPPM_LPG = 0;
    }
  } 
  else if ( gas_id == GAS_CO ) 
  {
    long gasPercentage = MQGetPercentage(rs_ro_ratio,COCurve);
    if(gasPercentage)
    {
      data->iPPM_CO = gasPercentage;
    }
    else
    {
      data->iPPM_CO = 0;
    }
  } 
  else if ( gas_id == GAS_SMOKE ) 
  {
    long gasPercentage = MQGetPercentage(rs_ro_ratio,SmokeCurve);
    if(gasPercentage)
    {
      data->iPPM_Smoke = gasPercentage;
    }
    else
    {
      data->iPPM_Smoke = 0;
    }
  }    

}
 
/*****************************  MQGetPercentage **********************************
  Input:   rs_ro_ratio - Rs divided by Ro
          pcurve      - pointer to the curve of the target gas
  Output:  ppm of the target gas
  Remarks: By using the slope and a point of the line. The x(logarithmic value of ppm) 
          of the line could be derived if y(rs_ro_ratio) is provided. As it is a 
          logarithmic coordinate, power of 10 is used to convert the result to non-logarithmic 
          value.
************************************************************************************/ 
long  MQGetPercentage(float rs_ro_ratio, float *pcurve)
{
  return (pow(10,( ((log(rs_ro_ratio)-pcurve[1])/pcurve[2]) + pcurve[0])));
}

/********************************  MQGetVoltage *************************************
  Input:   uint32_t adc_raw_value, SensorData* data
  Output:  adc voltage -> SensorData struct
           adc raw value -> SensorData struct
  Remarks: Gets raw adc data and converts it to voltage     
************************************************************************************/ 
void MQGetVoltage(uint32_t adc_raw_value, SensorData* data)
{
  float voltage = (adc_raw_value / ADC_MAX_RESOLUTION) * 3.3;
  data->adcMq2Voltage = voltage;
  data->adcRawValue = adc_raw_value;

}

/* End of MQ Functions 
 ******************************************************/


/******************************************************
 * DHT Functions */

/********************************  DHT_Measurement *************************************
  Input:   SensorData* data
  Output:  Temperature   -> SensorData struct
           Humidity      -> SensorData struct
           HeatIndex     -> SensorData struct
  Remarks: Reads DHT data using DHT_Callback
************************************************************************************/ 
void DHT_Measurement(SensorData* data)
{
  DHT_Callback(&DHT_ReadTemperature_Callback, data);
  DHT_Callback(&DHT_ReadHumidity_Callback, data);
  DHT_Callback(&DHT_ComputeHeatIndex_Callback, data);
}

/*
 *  DHT_Callback
 */
void DHT_Callback(void (*fptr)(SensorData* data), SensorData* data) 
{
  if (fptr) 
  {
    fptr(data);
  }
}

/*
 *  DHT_ReadTemperature_Callback
 *  data->temperature = temperature; (Celsius)
 */
void DHT_ReadTemperature_Callback(SensorData* data) 
{
  float temperature = dht_ptr->readTemperature(false);
  if (!isnan(temperature)) 
  {
    data->temperature = temperature;
  }
}

/*
 *  DHT_ReadHumidity_Callback
 *  data->humidity = humidity;
 */
void DHT_ReadHumidity_Callback(SensorData* data) 
{
  float humidity = dht_ptr->readHumidity();
  if (!isnan(humidity)) 
  {
    data->humidity = humidity;
  }
}

/*
 *  DHT_ComputeHeatIndex_Callback
 *  data->heatIndex = heatIndex;
 */
void DHT_ComputeHeatIndex_Callback(SensorData* data) 
{
  float heatIndex = dht_ptr->computeHeatIndex(data->temperature, data->humidity, false);
  if (!isnan(heatIndex)) 
  {
    data->heatIndex = heatIndex;
  }
}

/* End of DHT Functions 
 ******************************************************/

/******************************************************
 * DEBUG and Safety Functions */


/********************************  DebugPrint *************************************
  Input:   SensorData* data, VolumeSettings* settings
  Output:  None
  Remarks: Prints debug data from SWC`s
************************************************************************************/ 
void DebugPrint(SensorData* data, VolumeSettings* settings)
{

  Serial.print("[Time: "); Serial.print(millis()); Serial.println("ms] ");

#if DEGUG_DATA
  Serial.print("* Adc_Raw: " + String(data->adcRawValue));
  Serial.print(" Vin: " + String(data->adcMq2Voltage));
  Serial.println(" Ro:" + String(data->Ro) + " kOhm");

  Serial.print("* iPPM_LPG: " + String(data->iPPM_LPG));
  Serial.print(" iPPM_CO: " + String(data->iPPM_CO));
  Serial.println(" iPPM_Smoke: " + String(data->iPPM_Smoke));

  Serial.print("* Temperature: " + String(data->temperature) + "C ");
  Serial.print(" Humidity: "+String(data->humidity)+ "% ");
  Serial.println(" Heat Index: " + String(data->heatIndex));

  Serial.println();
#endif

#if DEGUG_COUNTER
    Serial.print("[Task_C0_Init][stat:");Serial.print(init_flag_C1);Serial.print("]");Serial.print("_|_");
    Serial.print("[Task_C0_2s][cnt:");Serial.print(cnt_2s_C0);Serial.print("]");Serial.print("_|_");
    Serial.print("[Task_C0_100ms][cnt:");Serial.print(cnt_100ms_C0);Serial.print("]");Serial.print("_|_");
    Serial.print("[Task_C1_Init][stat:");Serial.print(init_flag_C1);Serial.print("]");Serial.print("_|_");
    Serial.print("[Task_C1_10s][cnt:");Serial.print(cnt_10s_C1);Serial.print("]");Serial.println();
#endif

#if DEBUG_POT_PWM
    Serial.println("Potentiometer DBG: " + String(settings->potValue) + " -> Duty Cycle: " + String(settings->dutyCycle));
#endif

}

/****************************  LimitAlertCheck *************************************
  Input:   SensorData* data, VolumeSettings* settings (unused)
  Output:  None
  Remarks: Check gas percenatge and set alarm and led accordingly
************************************************************************************/ 
void LimitAlertCheck(SensorData* data, VolumeSettings* settings)
{
  long iPPM_LPG = data->iPPM_LPG;
  long iPPM_CO = data->iPPM_CO ;
  long iPPM_Smoke = data->iPPM_Smoke;

  /* Check gas percenatge and set alarm and led accordingly */
  if(iPPM_LPG < GAS_LPG_ALARM_LIMIT && iPPM_CO < GAS_CO_ALARM_LIMIT &&  iPPM_Smoke < GAS_SMOKE_ALARM_LIMIT)
  {
    digitalWrite(GREEN_LED_PIN_OUT_26, HIGH);
    digitalWrite(RED_LED_PIN_OUT_32, LOW);
    noTone(BUZZER_PIN_OUT_33);
    // ledcWrite(LEDC_CHANNEL, 0);
  }
  else
  {
    digitalWrite(GREEN_LED_PIN_OUT_26, LOW);
    digitalWrite(RED_LED_PIN_OUT_32, HIGH);
    tone(BUZZER_PIN_OUT_33, 500);

#if DEBUG_POT_PWM
    Serial.println("Potentiometer T100ms: " + String(settings->potValue) + " -> Duty Cycle: " + String(settings->dutyCycle));
#endif

    // ledcWrite(LEDC_CHANNEL, settings->dutyCycle);
  }

}

/****************************  ManageBuzzerVolume **********************************
  Input:   VolumeSettings* settings, uint8_t pot_pin
  Output:  None
  Remarks: Will be used in the future
************************************************************************************/ 
void ManageBuzzerVolume(VolumeSettings* settings, uint8_t pot_pin)
{
  uint16_t potValue = analogRead(pot_pin);
  settings->potValue = potValue;
  uint8_t dutyCycle = map(settings->potValue, 0, 4095, 0, 255);
  settings->dutyCycle = dutyCycle;
}

/* End of DEBUG and Safety Functions 
 ******************************************************/


/******************************************************
 * LCD Functions */

/****************************  LcdDisplayData **********************************
  Input:   SensorData* data, CustomChars* chars
  Output:  None
  Remarks: This function flip flops as the cnt_2s_C0 is even or not
           in order to display all system data on the LCD display
           using a simple FSM
************************************************************************************/ 
void LcdDisplayData(SensorData* data, CustomChars* chars)
{
  /* case selector */
  uint8_t selector = cnt_2s_C0 % 2;

  /* LCD cleanup */
  lcd_ptr->clear();  

  switch(selector)
  {
    case 0:
      /*---Row_1---------------------*/
      lcd_ptr->setCursor(0, 0);

      lcd_ptr->write(byte(chars->thermometer));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->temperature);
      
      lcd_ptr->print(" ");

      lcd_ptr->write(byte(chars->humidity));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->humidity);

      lcd_ptr->print(" ");

      lcd_ptr->write(byte(chars->heart));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->heatIndex);

      /*---Row_2---------------------*/
      lcd_ptr->setCursor(0, 1);

      lcd_ptr->write(byte(chars->wifi));
      lcd_ptr->print(":");
      if(wifi_connected) lcd_ptr->print("ON");
      if(!wifi_connected) lcd_ptr->print("OFF");

      lcd_ptr->print("   ");

      lcd_ptr->write(byte(chars->adc));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->adcRawValue);

      vTaskDelay(25 / portTICK_PERIOD_MS);

      break;

    case 1:
      /*---Row_1---------------------*/
      lcd_ptr->setCursor(0, 0);
      lcd_ptr->write(byte(chars->smoke));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->iPPM_Smoke);
      lcd_ptr->print("ppm");

      lcd_ptr->print(" ");

      lcd_ptr->write(byte(chars->lpg));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->iPPM_LPG);
      lcd_ptr->print("ppm");

      /*---Row_2---------------------*/
      lcd_ptr->setCursor(0, 1);

      lcd_ptr->write(byte(chars->co));
      lcd_ptr->print(":");
      lcd_ptr->print((int)data->iPPM_CO);
      lcd_ptr->print("ppm");

      vTaskDelay(25 / portTICK_PERIOD_MS);

      break;

  }

}

/* End of LCD Functions 
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

/************************** Wifi_SendDataToGoogleSpreadsheets ***********************
  Input:   SensorData* data
  Output:  None
  Remarks: This function sends data to Google Spreadsheet if sensor data is available
************************************************************************************/ 
void Wifi_SendDataToGoogleSpreadsheets(SensorData* data)
{
  /* Send data only if data from sensors is available */
  if(cnt_2s_C0 > 0)
  {
    if (WiFi.status() == WL_CONNECTED)
    {
        HTTPClient http;

        /* Build URL */
        String url = googleScriptURL +
                    "?temperature=" + String(data->temperature, 2) +
                    "&humidity=" + String(data->humidity, 2) +
                    "&heatIndex=" + String(data->heatIndex, 2) +
                    "&adcMq2Voltage=" + String(data->adcMq2Voltage, 2) +
                    "&Ro=" + String(data->Ro, 2) +
                    "&iPPM_LPG=" + String(data->iPPM_LPG) +
                    "&iPPM_CO=" + String(data->iPPM_CO) +
                    "&iPPM_Smoke=" + String(data->iPPM_Smoke) +
                    "&adcRawValue=" + String(data->adcRawValue);

        /* Init GET Request */
        http.begin(url);  
        /* Send GET Request */               
        int httpResponseCode = http.GET(); 

        if (httpResponseCode > 0)
        {
            String response = http.getString();

#if DEBUG_HTTP
            Serial.println("HTTP Response code: " + String(httpResponseCode));
            Serial.println("Response: " + response);
#endif

        }
        else
        {
#if DEBUG_HTTP
            Serial.println("Error on sending GET: " + String(httpResponseCode));
#endif
        }

        /* Close connection */
        http.end(); 
    }
    else
    {

#if DEBUG_HTTP
        Serial.println("WiFi not connected");
#endif

    }
  }
}

/* End of Wifi Functions 
 ******************************************************/
