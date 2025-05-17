#include <DHT.h>

// Definirea pinului și tipului de senzor
#define DHTPIN 25         // Pinul 25 pentru senzorul DHT11
#define DHTTYPE DHT11     // Tipul senzorului: DHT11

// Inițializarea senzorului DHT
DHT dht(DHTPIN, DHTTYPE);

// Funcție pentru calculul Heat Index
float computeHeatIndex(float temperature, float humidity) {
  // Formula pentru Heat Index, care este o aproximare a temperaturii percepute
  float T = temperature;   // Temperatura în grade Celsius
  float R = humidity;      // Umiditatea relativă în procente

  // Formula pentru calculul Heat Index (aproximare)
  float heatIndex = -8.78469475505676 + 1.61139411 * T + 2.33854883889 * R - 0.14611605 * T * R - 0.012308094 * T * T - 0.0164248277778 * R * R + 0.002211732 * T * T * R + 0.00072546 * T * R * R - 0.00000358 * T * T * R * R;

  return heatIndex;
}

void setup() {
  // Inițializarea comunicației seriale
  Serial.begin(115200);

  // Inițializarea senzorului DHT
  dht.begin();

  // Afișarea mesajului de pornire
  Serial.println("DHT11 Test");
}

void loop() {
  // Citirea valorilor de la senzor
  float temperature = dht.readTemperature(); // Citire temperatură în grade Celsius
  float humidity = dht.readHumidity();       // Citire umiditate

  // Verificarea erorilor de citire
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Eroare la citirea senzorului DHT11!");
    return;
  }

  // Calculul Heat Index
  float heatIndex = computeHeatIndex(temperature, humidity);

  // Afișarea valorilor citite și a Heat Index-ului
  Serial.print("Temperatura: ");
  Serial.print(temperature);
  Serial.print(" °C, ");
  Serial.print("Umiditate: ");
  Serial.print(humidity);
  Serial.print(" %, ");
  Serial.print("Heat Index: ");
  Serial.print(heatIndex);
  Serial.println(" °C");

  // Pauză între citiri (de 2 secunde)
  delay(2000);
}
