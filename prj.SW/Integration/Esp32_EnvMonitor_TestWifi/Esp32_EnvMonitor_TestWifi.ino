#include <WiFi.h>

// WiFi Credentials
const char* ssid = "CDIDC";
const char* password = "SkodaOctavia2005";

// WiFi Connection Status Flag
bool wifi_connected = false;

// Debugging flag
#define DEBUG_HTTP 1

/******************************************************
 * WiFi FUNCTIONS
 */

// Enum for connection status
typedef enum {
  INIT_FAIL = 0,
  INIT_OK = 1
} INIT_STATUS;  // This should be defined globally before any function using it

/*
 * Wifi_Callback - used to call wifi functions that exist or will be implemented in the future
 */
void Wifi_Callback(void (*fptr)(void)) {
  if (fptr) {
    fptr();
  }
}

/****************************  Wifi_SetupNetworkConnection ***************************
  Input:   None
  Output:  None
  Remarks: This function attempts to connect to a WiFi network
           If connection is successful, the wifi_connected flag is set true 
************************************************************************************/ 
void Wifi_SetupNetworkConnection() {
  int timeout = 0;
  const int MAX_TIMEOUT = 20;

#if DEBUG_HTTP
  Serial.print("Connecting to WiFi...");
#endif
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED && timeout < MAX_TIMEOUT) {
    wifi_connected = false;
    delay(1000);
    timeout++;

#if DEBUG_HTTP
    Serial.print(".");
#endif
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifi_connected = true;

#if DEBUG_HTTP
    Serial.println(" Connected!");
#endif
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());  // Display the local IP
  } else {
    Serial.println("Failed to connect to WiFi.");
  }
}

/********************************* Wifi_Init ****************************************
  Input:   None
  Output:  INIT_STATUS
  Remarks: This function initializes WiFi connection using a callback
************************************************************************************/
INIT_STATUS Wifi_Init() {
  // Initialize Wifi with Wifi callback
  Wifi_Callback(&Wifi_SetupNetworkConnection);
  return INIT_OK;
}

/****************************  Wifi_ScanNetworks ****************************************
  Input:   None
  Output:  None
  Remarks: This function scans for available WiFi networks and displays them in the serial monitor
************************************************************************************/
void Wifi_ScanNetworks() {
  Serial.println("Scanning for WiFi networks...");

  int n = WiFi.scanNetworks();
  if (n == 0) {
    Serial.println("No networks found.");
  } else {
    Serial.print(n);
    Serial.println(" networks found:");

    for (int i = 0; i < n; ++i) {
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (Signal Strength: ");
      Serial.print(WiFi.RSSI(i));
      Serial.println(" dBm)");
    }
  }
}

void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Initialize WiFi
  Wifi_Init();

  // Scan for networks
  Wifi_ScanNetworks();
}

void loop() {
  // Main loop, you can add further functionality here
}
