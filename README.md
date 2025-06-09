# 🧪 Virtual Instrument for Monitoring Environmental Parameters  
**with LabVIEW Interface, ESP32, and Google Spreadsheets Integration**

📄 [📘 Project Documentation (PDF)](https://github.com/CodreanuDan/EnvDataMonitoring_ESP32_Cloud_LabView/blob/main/prj.DOCS/Documentatie_IMPM.pdf)

---

## 📌 Project Description

This project consists of a **virtual instrument (VI)** designed to monitor environmental parameters such as:

- Temperature  
- Humidity  
- Air quality (gas concentration)

It integrates:

- An **ESP32 microcontroller** for sensor data acquisition  
- **Google Spreadsheets** for cloud-based data logging  
- A **LabVIEW interface** for real-time visualization and thermal comfort calculation

The system sends sensor data to Google Sheets via HTTP POST requests and provides real-time alerts based on predefined thresholds.

🔬 Initially, the system was created and simulated in [**Wokwi**](https://wokwi.com/projects/428946479635668993).  
🖼️ *Simulation screenshot:*  
![Wokwi simulation](https://github.com/user-attachments/assets/f193f156-f02d-47e8-970d-f94a4b2154c7)

---

## 🔧 Hardware Components

- **ESP32 Dev Module**
- **DHT11** sensor (temperature & humidity)
- **MQ-2** sensor (smoke & gas detection)
- **LCD Display** (I2C interface)
- Breadboard, jumper wires

🖼️ Hardware setup examples:  
![Sensor diagram](https://github.com/user-attachments/assets/37c78320-5569-4472-9f55-a37e5bbca82c)  
![LCD Modes](https://github.com/user-attachments/assets/15ee7928-55ed-426a-b7a3-072922689562)

---

## 🧠 Software Architecture

The software architecture is based on two key layers:

1. **ESP32 Firmware**
   - Acquires data from sensors
   - Sends data via HTTP POST to Google Sheets
2. **LabVIEW GUI**
   - Reads data via serial (VISA)
   - Calculates the THI comfort index
   - Displays alerts and data indicators

### Details:
- **DHT11** uses a single-wire digital protocol
- **MQ-2** sends analog data via the ESP32's ADC
- ESP32 runs two tasks on separate cores:
  - Core 0: Sensor data acquisition
  - Core 1: Data transmission

📊 *Task distribution chart:*  
![Core Task Table](https://github.com/user-attachments/assets/17492f16-398d-42f6-9d28-b8dff0c8e4a0)  
📐 *Software architecture diagram:*  
![Architecture](https://github.com/user-attachments/assets/1e8bdff3-cbcf-46ac-a06f-49bc9591ca5f)

---

## 🔁 Data Flow Overview

- 🌡️ **DHT11 sensor acquisition**  
  ![DHT Read](https://github.com/user-attachments/assets/46ce31fc-b611-4f9a-87f8-76c65ca639c7)

- 🫁 **MQ-2 gas detection logic**  
  ![MQ2 ADC Logic](https://github.com/user-attachments/assets/d4ebf676-23e9-4f32-a849-4ffefca97e72)

- 🚨 **Alert logic via thresholds/state machine**  
  ![Alert State Machine](https://github.com/user-attachments/assets/e60a227b-70b3-4f5c-9df9-735c2b177bc0)

- ☁️ **Cloud integration via Google Sheets**  
  ![Google Sheets Example](https://github.com/user-attachments/assets/40935e36-3817-4735-9b36-9f896d96e39d)

---

## 🖥️ LabVIEW Interface

The **LabVIEW GUI** receives sensor data via **serial communication (VISA)**. It displays:

- 📉 Temperature and Humidity
- 🫁 Gas Concentration (from MQ-2)
- 🌡️ **Thermal Comfort Index (THI)**
- 🚨 Alert status

The **THI (Temperature-Humidity Index)** is calculated with the following formula:

`THI = (T × 1.8 + 32) - (0.55 - 0.0055 × RH) × ((T × 1.8 + 32) - 58)`

Where:
- `T` is temperature in °C
- `RH` is relative humidity (%)

---

### 🔺 THI Comfort Thresholds

| THI (°F) | Color   | Comfort Level         |
|----------|---------|------------------------|
| < 65     | 🟢 Green  | Comfortable            |
| 65–70    | 🟡 Yellow | Slight discomfort      |
| 70–75    | 🟠 Orange | Moderate discomfort    |
| 75–80    | 🔴 Red    | Severe discomfort      |
| > 80     | 🟣 Maroon | Health hazard          |

📸 *LabVIEW Interface Screenshot:*  
![LabVIEW UI](https://github.com/user-attachments/assets/a791a8ae-99b0-4605-af1f-6fa4af44465d)

---

## 🖼️ Physical Device – Real Life Implementation

<p align="center">
  <img src="https://github.com/user-attachments/assets/e2474199-d8e2-43dc-83dc-570e33408d01" alt="P1" width="45%" />
  <img src="https://github.com/user-attachments/assets/76b687cd-4852-4772-8cea-7b9a5b71a4e9" alt="P2" width="45%" />
</p>
<p align="center"><i>Device running normally – no alert (P1 & P2)</i></p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/647eb296-e245-4146-a4d1-be5b3d631847" alt="P3" width="45%" />
  <img src="https://github.com/user-attachments/assets/1aa83415-8ead-460b-909f-1c94f9523a2e" alt="P4" width="45%" />
</p>
<p align="center"><i>PPM Limit Exceeded – alert triggered (P3 & P4)</i></p>
