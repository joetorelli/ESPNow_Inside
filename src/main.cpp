#include <Arduino.h>
//#include <WiFi.h>
#include "NetWork.h"
#include "Wire.h"
//#include "SRF.h"
#include "OLED.h"
//#include "settings.h"        // The order is important!
//#include "sensor_readings.h" // The order is important!
//#include "network_config.h"
//#include "SD_Card.h"
//#include <ezTime.h>
//#include <TaskScheduler.h>

// assign i2c pin numbers
#define I2c_SDA 23
#define I2c_SCL 22

/*******************   oled display   ******************/
// Declaration for an SSD1306 OLED_Display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 OLED_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/********************************************************   Receiver   *****************************************/
//ESPNow Feather MAC Adr:         CC:50:E3:BA:CA:A0
//ESPNow_Inside Recevier #2 ESP32S MAC Adr:   A4:CF:12:0B:2B:B4
//ESPNow_Outside Sender #1 ESP32s MAC Adr:  30:AE:A4:46:F0:E4
// Structure example to receive data
// Must match the sender structure
typedef struct struct_message
{
  char a[32];
  int b;
  float c;
  String d;
  bool e;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.a);
  Serial.print("Int: ");
  Serial.println(myData.b);
  Serial.print("Float: ");
  Serial.println(myData.c);
  Serial.print("String: ");
  Serial.println(myData.d);
  Serial.print("Bool: ");
  Serial.println(myData.e);
  Serial.println();
}
void setup()
{
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  //GetMacAdr();
}
void loop()
{
  // put your main code here, to run repeatedly:
}