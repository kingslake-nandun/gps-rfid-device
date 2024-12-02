#include <TinyGPS++.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ArduinoQueue.h>
#include "esp_sleep.h"
#include <TimeLib.h>  // Include the Time library for easier timestamp calculations


/*
RFID Pinout
18 SCK
19 MISO
21 SDA
22 RST
23 MOSI
*/

#define GPS_LED_PIN 2
#define RFID_LED_PIN 12
#define POWER_LED_PIN 13
#define BUZZER_PIN 14
#define BUTTON_PIN GPIO_NUM_15  // Define the button pin using GPIO_NUM_XX type  // Push button pin

#define MIN_SATELLITES 4 //minimum sattelites required for accurate gps

// RFID Pins
#define SS_PIN 21
#define RST_PIN 22
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

////////////////////////////////////////////////////////////////////////////////change following according to requirements///////////////////////////////////////////////////////////

const String deviceId = "1";  //specify the id of the device
const String serverLink = "65.2.148.124";
const String port = "5402";

const long interval = 10000;  // Interval to update GPS (10 seconds)
const long rfidSendInterval = 20000;  // 20 seconds interval

#define RFID_QUEUE_SIZE 60
#define GPS_QUEUE_SIZE 360

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Queue to store RFID data                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                
ArduinoQueue<String> rfidQueue(RFID_QUEUE_SIZE);

// Queue to store GPS data 
ArduinoQueue<String> gpsQueue(GPS_QUEUE_SIZE);  

// GPS Pins
static const int RXPin = 16, TXPin = 17; 
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
HardwareSerial gpsSerial(1);  // Use Serial1 for GPS

// GSM Module Pins
HardwareSerial gsmSerial(2);  // Use Serial2 for GSM (can use other pins)
const int gsmRXPin = 4, gsmTXPin = 5;

// Timing for RFID data sending
unsigned long lastRFIDSendTime = 0;

// Timing for GPS data sending
unsigned long previousMillis = 0;

// Flag to control task execution
volatile bool tasksRunning = false;
volatile bool stateChanged = false;  //  flag to indicate state change

unsigned long lastDebounceTime = 0;  // the last time the button was toggled
unsigned long debounceDelay = 5000;    // debounce time in milliseconds

// Global variable to control LED blinking
volatile bool blinkFlag = false;  // This can be modified by other tasks
volatile bool gpsSentFlag = false;  //to indicate signal is available and blink gps led 1s

volatile bool dataSendFinished = false;  //to indicate if there is data still to send

volatile bool reCheck = false;  //to indicate if data is sent after pressing stop

// Interval for LED blinking
const int ledBlinkInterval = 500;  // Blink every 0.5s 

void IRAM_ATTR handleButtonPress() {
  unsigned long currentTime = millis();

  // If the time since the last press is greater than debounceDelay, consider it valid
  if (currentTime - lastDebounceTime > debounceDelay) {
    tasksRunning = !tasksRunning;  // Toggle task running state
    stateChanged = true;  // Set the state changed flag

    lastDebounceTime = currentTime;  // Update the last debounce time
  }
}

void setup() {
  Serial.begin(115200);   // Initialize Serial Monitor communication

  gpsSerial.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);    // Initialize GPS serial communication
  gsmSerial.begin(9600, SERIAL_8N1, gsmRXPin, gsmTXPin); // Initialize GSM serial communication

  pinMode(POWER_LED_PIN, OUTPUT); // Initialize power Indicator LED
  pinMode(GPS_LED_PIN, OUTPUT);  // Initialize  GPS LED pin
  pinMode(RFID_LED_PIN, OUTPUT);  // Initialize  RFID LED pin
  pinMode(BUZZER_PIN, OUTPUT);  // Initialize  buzzer pin
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Initialize button pin

  vTaskDelay(1000 / portTICK_PERIOD_MS);  // Small delay to avoid 

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);  // Attach interrupt

  SPI.begin();          // Initialize SPI bus
  mfrc522.PCD_Init();   // Initialize RFID module

  Serial.println("RFID Reader Initialized.");


  // Set up wake-up from deep sleep on button press (GPIO 15)
  esp_sleep_enable_ext0_wakeup(BUTTON_PIN, LOW);  // Wake up when the button is pressed (falling edge)


  // Start tasks on different cores
  xTaskCreatePinnedToCore(gpsTask, "GPSTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(rfidTask, "RFIDTask", 10000, NULL, 1, NULL, 0);
    // Start the blink task on Core 1
  xTaskCreatePinnedToCore(
    ledBlinkTask,         // Task function
    "LedBlinkTask",       // Name of the task
    1000,                 // Stack size (in bytes)
    NULL,                 // Task input parameter
    1,                    // Priority of the task
    NULL,                 // Task handle
    1                     // Core where the task should run
  );

  // Ensure that we are not falsely triggering on power-up
  tasksRunning = false;   // Explicitly set to false at the start
  stateChanged = false;   // No state change yet

   // Check if the ESP32 was woken up by the button
  if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) {
    Serial.println("Woken up by button press.");
    tasksRunning = !tasksRunning;  // Toggle task running state
    stateChanged = true;  // Set the state changed flag

  
  } else {
    // Go to deep sleep after setup
    enterDeepSleep();
  }
}

void loop() {
  // The main loop can remain empty as the tasks run independently
  
}

void gpsTask(void *pvParameters) {
  while (true) {

    if(rfidQueue.isEmpty()){
      dataSendFinished = true;
    }else{
      dataSendFinished = false;
    }

    if (stateChanged) {  // Check if the state has changed
      if(dataSendFinished){
        stateChanged = false;  // Reset the flag only if queue is empty
      }
      if (tasksRunning && !reCheck) { 
        unsigned long startMillis = millis();
        digitalWrite(POWER_LED_PIN, HIGH);
        openConnection(serverLink, port);
         // Get the latest GPS data
        String gpsData = getGPSData();
        long timestamp = getTimestamp();
        
        // Wait until valid GPS data and timestamp are available
        while (gpsData == "" || timestamp == -1) {
          
            gpsData = getGPSData();        // Fetch the GPS location data
            timestamp = getTimestamp();    // Fetch the GPS timestamp

            vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to avoid tight loop
          
        }
        int success = 0;
        String jsonToSend = "[{\"id\":" + deviceId + ",\"ty\":1," + "\"ts\":" + timestamp + ",\"loc\":\"" + gpsData + "\"}]";
        while(!success){
          success = sendToServer(jsonToSend, serverLink, port);
          if(((millis() - startMillis )> 20000) && !success ) {  //if data sending fail for 20s 
            startMillis = millis();
            rebootGsm();   //reboot gsm module
          }
        }
        Serial.println("Tasks Started");
      } else if(dataSendFinished) {
        unsigned long stopMillis = millis();
        if(reCheck){
          tasksRunning = false;
          reCheck = false;
        }
        
        blinkFlag = false;
        String gpsData = getGPSData();
        long timestamp = getTimestamp();
        
        // Wait until valid GPS data and timestamp are available
        while ( gpsData == "" || timestamp == -1) {
          
            gpsData = getGPSData();        // Fetch the GPS location data
            timestamp = getTimestamp();    // Fetch the GPS timestamp

            vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to avoid tight loop
          
        }
        int success = 0;
        String jsonToSend = "[{\"id\":" + deviceId + ",\"ty\":2," + "\"ts\":" + timestamp + ",\"loc\":\"" + gpsData + "\"}]";
       
        while(!success){
          success = sendToServer(jsonToSend,serverLink, port);
          if(((millis() - stopMillis )> 20000) && !success ) {    
            stopMillis = millis();
            rebootGsm();   //reboot gsm 
          }
        }
        closeConnection();
        
        // Clear the queues on stop
        while (!rfidQueue.isEmpty()) {
          rfidQueue.dequeue();
        }
        while (!gpsQueue.isEmpty()) {
          gpsQueue.dequeue();
        }
        digitalWrite(GPS_LED_PIN, LOW);
        digitalWrite(POWER_LED_PIN, LOW);
        Serial.println("Tasks Stopped");
        enterDeepSleep();
        
      }else{
        tasksRunning = true;
        reCheck = true;  
      }
    }

    if (tasksRunning) {
      // Check if interval has passed since the last RFID data send
      unsigned long currentMillis = millis();
      if ((currentMillis - lastRFIDSendTime >= rfidSendInterval) && (!rfidQueue.isEmpty())) {
        lastRFIDSendTime = currentMillis;
   
       while(!rfidQueue.isEmpty()){
        String rfidJsonToSend = rfidQueue.getHead();
        int success = sendToServer(rfidJsonToSend, serverLink, port);
        if (!success){
          break;
        }
        else{
          rfidQueue.dequeue();
        }
       }

       if(rfidQueue.isEmpty()){  //queue is empty , data sending successfull
         Serial.println("RFID data sent");
       } else{
         Serial.println("Failed to send RFID data. Will retry.");
       }


      } else {
       
        if ((!gpsQueue.isEmpty())) {
      
          while (!gpsQueue.isEmpty()) {
            String gpsJsonToSend = gpsQueue.getHead();
            int success = sendToServer(gpsJsonToSend, serverLink, port);
            if (!success){
              break;
            }else{
              gpsQueue.dequeue();
            }
          }

          if(gpsQueue.isEmpty()){   //queue is empty , data sending successful
            Serial.println("GPS data sent" );
            blinkFlag = false;
            gpsSentFlag = true; // Turn on the GPS LED
            digitalWrite(GPS_LED_PIN, HIGH);
            vTaskDelay(1000 / portTICK_PERIOD_MS); //turn on LED for 1s
            digitalWrite(GPS_LED_PIN, LOW);
            gpsSentFlag = false;
          }else{
            Serial.println("Failed to send GPS data. Will retry.");
              //try rebooting gsm if queues are growing 
              if((gpsQueue.item_count()>10) || rfidQueue.item_count()>10){
                rebootGsm();
              }
          } 
          
        }
      }

       vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid overwhelming the CPU
    } else {
       vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay to prevent WDT issues
    }
  }
}


void rfidTask(void *pvParameters) {
  while (true) {
    if (tasksRunning) {
      unsigned long currentMillis = millis();

    // Get the latest GPS data
      String gpsData = getGPSData();
      long timestamp = getTimestamp();
      
      // Wait until valid GPS data and timestamp are available
      while (tasksRunning && (gpsData == "" || timestamp == -1)) {
        
          gpsData = getGPSData();        // Fetch the GPS location data
          timestamp = getTimestamp();    // Fetch the GPS timestamp

          vTaskDelay(100 / portTICK_PERIOD_MS);  // Small delay to avoid tight loop
        
      }

      String rfidData = getRFIDData();

      if (rfidData != "") {

        // Combine the RFID data with the GPS data
        String combinedData = "[{\"id\":" + deviceId + ",\"ty\":4,\"ts\":" + timestamp + ",\"loc\":\"" + gpsData + "\",\"rid\":\"" + rfidData + "\"}]";

        if ((!rfidQueue.isFull()) && (gpsData != "" && timestamp != -1)) {  // Check if there's space in the queue
          rfidQueue.enqueue(combinedData);
          Serial.print("RFID Read: ");
          Serial.println(combinedData);
          digitalWrite(RFID_LED_PIN, HIGH); //turn on rfid indicator LED
          tone(BUZZER_PIN, 1000); // turn on buzzer
          vTaskDelay(1000 / portTICK_PERIOD_MS); //turn on LED & buzzer for 1s
          noTone(BUZZER_PIN); 
          digitalWrite(RFID_LED_PIN, LOW);

        } else {
          Serial.println("RFID data queue is full!");
        }
      }

      if (gpsData != "" && timestamp != -1) {
        blinkFlag = false; //gps signal available
        String gpsJsonData = "[{\"id\":" + deviceId + ",\"ty\":3,\"ts\":" + timestamp + ",\"loc\":\"" + gpsData +"\"}]";
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;
          gpsQueue.enqueue(gpsJsonData);  // Add new GPS data to the queue
        }
      }

      vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay to avoid overwhelming the CPU
    } else {
      vTaskDelay(100 / portTICK_PERIOD_MS);  // Delay to prevent WDT issues
    }
  }
}

// Task to blink LED
void ledBlinkTask(void *pvParameters) {
  while (true) {
    if (blinkFlag ) {
      // If blinkFlag is true, blink the LED
      digitalWrite(GPS_LED_PIN, HIGH);
      delay(ledBlinkInterval);
      digitalWrite(GPS_LED_PIN, LOW);
      delay(ledBlinkInterval);
    } else if(gpsSentFlag){
     
    }
    else{
      // If blinkFlag is false, keep the LED off
      digitalWrite(GPS_LED_PIN, LOW);
    }
    // Yield or delay to avoid blocking other tasks
    vTaskDelay(100 / portTICK_PERIOD_MS);  // 100ms delay before next check
  }
}

void rebootGsm(){
  sendCommand("AT+CFUN=1,1");  // Reboot the module
  delay(5000);                 // Wait for the module to reboot
  ShowSerialData();
  gsmSerial.begin(9600, SERIAL_8N1, gsmRXPin, gsmTXPin); // Initialize GSM serial communication
  delay(5000);
  Serial.println("gsm restarted");
  //sendToServer("gsm restarted", serverLink, port);    
}

String getGPSData() {
  unsigned long currentMillis = millis();
  String gpsInfo = "";

  while (gpsSerial.available() > 0) { 
    if (gps.encode(gpsSerial.read())) {
      // Check if the number of satellites is valid
      int satellites = gps.satellites.value();
     // Serial.print("Sattelites: ");
      //Serial.println(satellites);
      if (satellites >= MIN_SATELLITES && gps.location.isValid()) {
        
          gpsInfo = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
        
      } else {
        //Serial.println(F("GPS INVALID"));
        blinkFlag = true;
      }
    }
  }  

  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
    //while (true); // Halt execution
  }
  return gpsInfo;
}

// String getTimestamp() {
//   String timestamp = "";
//   if (gps.date.isValid() && gps.time.isValid()) {
//     char isoTimestamp[21];  // Allocate enough space for "YYYY-MM-DDTHH:MM:SSZ"
    
//     // Construct the ISO-8601 formatted timestamp
//     sprintf(isoTimestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ", 
//             gps.date.year(), gps.date.month(), gps.date.day(), 
//             gps.time.hour(), gps.time.minute(), gps.time.second());
    
//     timestamp = String(isoTimestamp);
//   } else {
//     Serial.println(F("Time INVALID"));
//   }
//   return timestamp;
// }

long getTimestamp() {
  if (gps.date.isValid() && gps.time.isValid()) {
    // Extract date and time from GPS
    int year = gps.date.year();
    int month = gps.date.month();
    int day = gps.date.day();
    int hour = gps.time.hour();
    int minute = gps.time.minute();
    int second = gps.time.second();

    // Define a time structure and set it with GPS date and time
    tmElements_t tm;
    tm.Year = year - 1970;   // `tm.Year` is years since 1970
    tm.Month = month;
    tm.Day = day;
    tm.Hour = hour;
    tm.Minute = minute;
    tm.Second = second;

    // Calculate Unix timestamp
    return makeTime(tm);   // Returns Unix timestamp in seconds
  } else {
    Serial.println(F("Time INVALID"));
    return -1;   // Return -1 if time is invalid
  }
}

////////////////////////////////////////////////////////////////////////////////////////////



// Function to start a connection if not already connected
int openConnection(String serverLink, String port) {

    // Bring up wireless connection
    sendCommand("AT+CIICR");
    ShowSerialData();

    String command = "AT+CIPSTART=\"TCP\",\"" + serverLink + "\",\"" + port + "\"";
    sendCommand(command.c_str());
    String response = ShowSerialData();
    
    // Check if connection was successful
    if (response.indexOf("CONNECT OK") != -1) {
 
      Serial.println("Connection opened.");
      return 1;
    } else {
      Serial.println("Failed to open connection.");
      return 0;
    }
  
}

// Function to close the connection when necessary
void closeConnection() {

    sendCommand("AT+CIPCLOSE");
    ShowSerialData();
  
    sendCommand("AT+CIPSHUT");
    String response = ShowSerialData();

     if (response.indexOf("SHUT OK") != -1) {
   
      Serial.println("Connection closed.");
    } else {
      Serial.println("Failed to close connection.");
    }
  
}

int sendToServer(String jsonToSend, String serverLink, String port) {
  // Check if we're already connected. If not, establish connection.
  String command = "AT+CIPSTATUS";
  sendCommand(command.c_str());
  String status = ShowSerialData();

  if (status.indexOf("CONNECT OK") == -1) {  // Not connected
    //try to reconnect
    if (!openConnection(serverLink, port)) {
      return 0;
    }
  }

  // Send data
  sendCommand("AT+CIPSEND");
  if (ShowSerialData().indexOf(">") != -1) {  // Wait for '>' prompt to send data
    gsmSerial.print(jsonToSend);
    gsmSerial.write(26);  // Send ASCII code 26 (CTRL+Z)
 
    unsigned long timeout = millis();
    String response = "";
    
    // Allow longer time for larger data, but exit early if the response is complete
    while (millis() - timeout < 10000) {  // Set an upper limit for timeout 
      while (gsmSerial.available()) {
        char c = gsmSerial.read();
        response += c;
        Serial.print(c);

        // Break the loop early if "SEND OK" or other success message is detected
        if (response.indexOf("SEND OK") != -1) {
          Serial.println();
          return 1;  // Return early if success message is received
        }
      }
    }
    Serial.println(jsonToSend);
  }
  Serial.println("Failed to send data.");
  return 0;  // Failure
}

void sendCommand(const char* command) {
  Serial.print("Sending command: ");
  Serial.println(command);
  gsmSerial.println(command);
}

String ShowSerialData() {
  unsigned long timeout = millis();
  String response = "";

  while (millis() - timeout < 1000) {  // 3-second timeout
    while (gsmSerial.available()) {
      char c = gsmSerial.read();
      response += c;
      Serial.print(c);
    }
  }

  return response;
}
////////////////////////////////////////////////////////////////////////////////

String getRFIDData() {
  String content = "";
  if (mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()) {
    for (byte i = 0; i < mfrc522.uid.size; i++) {
      content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
      content.concat(String(mfrc522.uid.uidByte[i], HEX));
    }
    content.toUpperCase();
    mfrc522.PICC_HaltA();  // Stop reading after a successful read
  }
  return content.substring(1);
}

void enterDeepSleep() {
  Serial.println("Entering deep sleep...");

  // Turn off LEDs and other peripherals to save power
  digitalWrite(GPS_LED_PIN, LOW);
  digitalWrite(RFID_LED_PIN, LOW);
  digitalWrite(POWER_LED_PIN, LOW);

  // Go to deep sleep until the button is pressed
  esp_deep_sleep_start();
}

