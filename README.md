# 🧪 Virtual Instrument for Monitoring Environmental Parameters with LabVIEW Interface, ESP32 and Google Spreadsheets Integration

📄 [Link to project documentation (PDF)]((https://github.com/CodreanuDan/EnvDataMonitoring_ESP32_Cloud_LabView/blob/main/prj.DOCS/Documentatie_IMPM.pdf))

---

## 📌 Project Description

This project consists of a **virtual instrument** (VI) designed to **monitor environmental parameters** such as temperature, humidity, and air quality. The system integrates:

- **ESP32 microcontroller** for sensor data acquisition
- **Google Spreadsheets** for cloud-based data logging
- **LabVIEW** as a visualization and analysis interface
- **Comfort index (THI) calculation** for thermal comfort evaluation

The system reads data from environmental sensors and transmits it via HTTP POST to Google Sheets, enabling real-time monitoring and data storage.
It was created and simulated firstly in Wokwi.com: 
📄 [Link to project simultation](https://github.com/CodreanuDan/EnvDataMonitoring_ESP32_Cloud_LabView/blob/main/prj.DOCS/Documentatie_IMPM.pdf](https://wokwi.com/projects/428946479635668993)
🖼️ *[![image](https://github.com/user-attachments/assets/f193f156-f02d-47e8-970d-f94a4b2154c7))]*
---

## 🔧 Hardware Components

- **ESP32 Dev Module**
- **DHT11** sensor for temperature and humidity
- **MQ-2** sensor for gas and smoke detection
- **LCD Display** (I2C interface)
- Jumper wires, Breadboard

🖼️ *[![image](https://github.com/user-attachments/assets/37c78320-5569-4472-9f55-a37e5bbca82c)]*
🖼️ *[![image](![Screenshot 2025-06-09 150809](https://github.com/user-attachments/assets/15ee7928-55ed-426a-b7a3-072922689562))]*
---

## 🧠 Software Architecture

The software architecture consists of two main layers:
1. **ESP32 firmware** for data acquisition and HTTP communication
2. **LabVIEW application** for data visualization, processing, and comfort index calculation

- **DHT11** communicates over a digital single-wire protocol
- **MQ-2** analog values are read via ESP32's **ADC channel**
- Communication with Google Sheets is done using the **HTTP POST method**
- Serial communication is initialized in setup, and tasks are assigned across the ESP32's **dual-core processor**:
    - Core 0: Sensor data acquisition
    - Core 1: Data transmission to Google Sheets

📊 *[Task allocation table![image](https://github.com/user-attachments/assets/17492f16-398d-42f6-9d28-b8dff0c8e4a0)]*  
📐 *[Software architecture diagram![image](https://github.com/user-attachments/assets/1e8bdff3-cbcf-46ac-a06f-49bc9591ca5f)]*

---

## 🔁 Data Flow Overview

- 🌡️ **DHT11 acquisition**  
  *[DHT11 read logic![image](https://github.com/user-attachments/assets/46ce31fc-b611-4f9a-87f8-76c65ca639c7)]*

- 🫁 **MQ-2 gas detection**  
  *[ADC read logic![image](https://github.com/user-attachments/assets/d4ebf676-23e9-4f32-a849-4ffefca97e72)]*

- 🚨 **Alert system and logic**  
  *[state machine image![image](https://github.com/user-attachments/assets/e60a227b-70b3-4f5c-9df9-735c2b177bc0)]*

- 📊 *[Cloud data![image](https://github.com/user-attachments/assets/40935e36-3817-4735-9b36-9f896d96e39d)]*  
---

## 🖥️ LabVIEW Interface

The LabVIEW GUI receives data via VISA (Serial Communication), performs calculations and displays:

- Temperature and Humidity
- Gas Concentration
- **Thermal Comfort Index (THI)**
- Alert status

The THI (Temperature-Humidity Index) is calculated using:

THI = (T * 1.8 + 32) - (0.55 - 0.0055 * RH) * ((T * 1.8 + 32) - 58);
---

### 🔺 THI Thresholds for Comfort Index

| THI (°F) | Color         | Comfort Level          |
|----------|---------------|------------------------|
| < 65     | 🟢 Green      | Comfortable            |
| 65–70    | 🟡 Yellow     | Slight discomfort      |
| 70–75    | 🟠 Orange     | Moderate discomfort    |
| 75–80    | 🔴 Red        | Severe discomfort      |
| > 80     | 🟣 Maroon     | Health hazard          |

📸 *[ LabVIEW application interface with indicators and gauge visualization![image](https://github.com/user-attachments/assets/a791a8ae-99b0-4605-af1f-6fa4af44465d)]*
