#include <Arduino.h>
//#include <WiFi.h>
#include "NetWork.h"
#include "Wire.h"
//#include "SRF.h"

#include "settings.h" // The order is important!
//#include "sensor_readings.h" // The order is important!
//#include "network_config.h"
//#include "SD_Card.h"
//#include <ezTime.h>
//#include <TaskScheduler.h>
#include "OLED.h"
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
// assign i2c pin numbers
#define I2c_SDA 21
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
  /*********   init i2c  *********/
  Wire.begin(I2c_SDA, I2c_SCL);
  bool status; // connect status
  DEBUGPRINTLN("I2C INIT OK");

  /********************* oled  ********************/
  // SSD1306_SWITCHCAPVCC = generate OLED_Display voltage from 3.3V internally
  if (!OLED_Display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) // Address 0x3C for 128x32
  {
    DEBUGPRINTLN(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  else
  {
    DEBUGPRINTLN("SSD1306 Init");
  }
  // Clear the oled buffer.
  OLED_Display.clearDisplay();
  OLED_Display.display();
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(SSD1306_WHITE);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("init ESP-NOW");
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);
  //GetMacAdr();
  OLED_Display.setCursor(0, 0);
}
void loop()
{
  WiFi.mode(WIFI_MODE_STA);
  OLED_Display.print(WiFi.macAddress());
  DEBUGPRINTLN(WiFi.macAddress());
  delay(2000);
}