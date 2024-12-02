# Technical Documentation for the GPS/RFID Device

## 1. Introduction

The GPS/RFID device is a multi-functional embedded system designed for transport and personnel tracking applications. The device integrates GPS for real-time location tracking and RFID for personnel or item identification. This system communicates with a central server to transmit location and RFID data over GSM.

## 2. Key Features

- **GPS Tracking:** Captures real-time location data with a minimum satellite requirement for accuracy.
- **RFID Identification:** Reads RFID tags and queues the data for transmission to the server.
- **Dual-Core Processing:** Tasks are distributed across two cores for efficient processing.
- **Low Power Mode:** Utilizes deep sleep to conserve power when idle.
- **GSM Communication:** Sends collected GPS and RFID data to a server over a GSM network.
- **Real-Time Data Handling:** Ensures timely processing and transmission of GPS and RFID data.
- **Button Control:** A physical button to start and stop data collection and transmission.
- **Indicators:** LEDs for GPS, RFID, and power status, along with a buzzer for alerts.

## 3. Hardware Components

| Component          | Purpose                                   | Pins Used                                  |
|--------------------|-------------------------------------------|-------------------------------------------|
| ESP32             | Microcontroller                          | GPIO pins as configured in code.          |
| RC522 RFID Reader | Reads RFID tags                          | SCK (18), MISO (19), MOSI (23), SDA (21), RST (22) |
| NEO-6M GPS Module | Provides GPS data                        | RX (16), TX (17)                          |
| SIM800 GSM Module | Handles GSM communication                | RX (4), TX (5)                            |
| Push Button       | Toggles data collection mode             | GPIO 15                                   |
| LEDs              | Indicators for GPS, RFID, and power      | GPIO 2, 12, 13                            |
| Buzzer            | Alerts for specific events               | GPIO 14                                   |

## 4. Software Components

### 1. Libraries Used
- **TinyGPS++:** Parses GPS data.
- **SPI and MFRC522:** For SPI communication and RFID module control.
- **ArduinoQueue:** Provides queue data structure for storing RFID and GPS data.
- **TimeLib:** Handles timestamp calculations.
- **esp_sleep.h:** Manages ESP32's deep sleep mode.

### 2. Pin Configuration

Example:
```cpp
#define GPS_LED_PIN 2
#define RFID_LED_PIN 12
#define POWER_LED_PIN 13
#define BUZZER_PIN 14
#define BUTTON_PIN GPIO_NUM_15
```

### 3. Task-Based Architecture
- **GPS Task:** Collects and queues GPS data periodically.
- **RFID Task:** Reads RFID data and queues it for transmission.
- **LED Blink Task:** Manages status indicators (blinking LEDs).
- **Task Distribution:** Tasks are distributed across the two ESP32 cores for concurrent execution.

## 5. System Workflow

### Initialization
- Initializes GPS, RFID, GSM, and peripheral components.
- Configures deep sleep wake-up on button press.

### Operation Modes
- **Start Mode:** Activated by pressing the button. Starts data collection and transmission tasks.
- **Stop Mode:** Pressing the button again stops data collection. Sends final queued data, clears queues, and enters deep sleep.

### Data Handling
- **GPS Data:** Collected every 10 seconds, validated with a minimum of 4 satellites, and sent to the server as a JSON payload.
- **RFID Data:** Reads RFID tags as they are scanned, queues, and transmits in intervals (20 seconds default).

### LED Functionality
- **Power LED:** Indicates power and task status.
- **GPS LED:** Provides feedback on GPS functionality.
- **RFID LED:** Blinks when an RFID tag is successfully read.
- **Buzzer:** Alerts for specific events like RFID scanning.

### Deep Sleep
- Enters low power mode when the stop command is issued, and all data is successfully sent.
- Wake-up is triggered by a button press.

## 6. Server Communication

### Server Details
- **IP Address:** 65.2.148.124
- **Port:** 5402

### JSON Payload Format
- **Start Trip:**  
  ```json
  [{"id": "1", "ty": 1, "ts": 1635692881, "loc": "12.9716,77.5946"}]
  ```
- **Stop Trip:**  
  ```json
  [{"id": "1", "ty": 2, "ts": 1635692881, "loc": "12.9716,77.5946"}]
  ```
- **GPS Data:**  
  ```json
  [{"id": "1", "ty": 3, "ts": 1635692881, "loc": "12.9716,77.5946"}]
  ```
- **RFID Data:**  
  ```json
  [{"id": "1", "ty": 4, "ts": 1635692881, "rid": "93 0C 68 1A"}]
  ```

### Data Transmission
- Ensures successful transmission before dequeuing.
- Reboots GSM module if transmission fails for over 20 seconds.

## 7. Deep Sleep and Power Management
- **Low Power Mode:** Reduces power consumption by entering deep sleep.
- **Wake-Up:** Configured to wake on a button press (GPIO 15).
- **LED Indicators:** Powered off during sleep to conserve energy.

## 8. Error Handling
- Retries failed transmissions with exponential backoff.
- Reboots GSM module on repeated failures.
- Clears queues and reinitializes tasks on state changes.

## 9. Future Enhancements
- Add support for over-the-air (OTA) firmware updates.
- Integrate voltage monitoring for backup battery status.

## 10. Conclusion
This GPS/RFID device integrates location tracking and RFID identification into a single, efficient system. Its modular design and robust error handling make it ideal for applications like transport management and personnel tracking. The use of dual-core processing and deep sleep ensures high performance while maintaining energy efficiency.

