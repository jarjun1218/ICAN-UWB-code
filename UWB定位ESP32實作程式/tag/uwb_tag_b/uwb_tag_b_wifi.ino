/*
For ESP32 UWB or ESP32 UWB Pro
Adding ESP-NOW function for sending ranging result to TAG A 
*/

#include <SPI.h>
#include <HardwareSerial.h>
#include <DW1000Ranging.h>
#include <DW1000.h>
#include <WiFi.h>
#include <WebSerial.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Array.h>

AsyncWebServer server(80);

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define RXD2 16
#define TXD2 17

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
uint16_t Adelay = 16384;
// leftmost two bytes below will become the "short address"
char TAG_ADD[] = "B2:00:22:EA:82:60:3B:9C";

int device_num = 0; // current number of device
int count = 0; // count the number of range added
Array<String, 4> ranging_result;

// Wi-Fi SSID
const char *ssid = "ICAN_327";
const char *password = "ncuicanlab";
const char *host = "192.168.0.131";

void setup() {
  // Serial setup
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  // Wi-Fi setup
  //wifiSetup();
  delay(1000);
  Serial.println("Setup has already done!");
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachNewDevice(newDevice);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  DW1000Ranging.useRangeFilter(true);
  DW1000Ranging.setRangeFilterValue(10);
  //Enable smart power
  DW1000.useSmartPower(true);

  // set antenna delay for anchors only. Tag is default (16384)
  // DW1000.setAntennaDelay(Adelay);
  
  // we start the module as a tag
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_2, false); // for group A
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_5, false); // for group B
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER, false);
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);
  // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);
  DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY, DW1000.CHANNEL_5, false); // for group B

  ranging_result.clear();
}

void loop() {
  DW1000Ranging.loop();
  if (count == device_num && device_num != 0) {
    String temp = "";
    temp = temp + String(device_num) + ",";
    for (int i = 0; i < device_num; i++) {
      temp += ranging_result[i];
      if (i < device_num - 1) {
        temp += ",";
      }
    }
    Serial.println(temp);
    Serial2.println(temp);
    ranging_result.clear();
    count = 0;
  }
}

void newRange() {
  String addr = String(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  float range = DW1000Ranging.getDistantDevice()->getRange();
  ranging_result.push_back(addr + ":" + String(range));
  count++;
  /*
  Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial.print(":");
  Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial.println("m");
  Serial2.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  Serial2.print(":");
  Serial2.print(DW1000Ranging.getDistantDevice()->getRange());
  Serial2.println("m");
  */
  /*
  WebSerial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  WebSerial.print(":");
  WebSerial.print(DW1000Ranging.getDistantDevice()->getRange());
  WebSerial.println("m");
  */
  /*
  Serial.print("\t RX power: ");
  Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  Serial.println(" dBm");
  */
}

void newDevice(DW1000Device *device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
  device_num++;
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
  device_num--;
}

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("WiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  WebSerial.begin(&server);
  server.begin();
}
