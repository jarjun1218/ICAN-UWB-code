/*

For ESP32 UWB or ESP32 UWB Pro

*/

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"
#include <HardwareSerial.h>
extern "C"{
  #include "positioning.h"
}

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4
#define RXD1 21
#define TXD1 22
#define RXD2 16
#define TXD2 17

HardwareSerial SerialPort1(1); // use UART1 to get data from tag B

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

// TAG antenna delay defaults to 16384
// uint16_t Adelay = 16320;

// leftmost two bytes below will become the "short address"
char TAG_ADD[] = "B1:00:22:EA:82:60:3B:9C";

int device_num = 0; // current number of device
int device_num_a = 0;
int count = 0; // count the number of range added
float distant_list[8];
float position_result[3];
float anchor[MAX_NUM_ANCHOR][3] = {0};

void setup() {
    Serial.begin(115200);
    SerialPort1.begin(115200, SERIAL_8N1, RXD1, TXD1); // connect to kalman filter
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2); // connect to tag B
    delay(1000);
    // init the configuration
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
    // define the sketch as anchor. It will be great to dynamically change the type of module
    DW1000Ranging.attachNewRange(newRange);
    DW1000Ranging.attachNewDevice(newDevice);
    DW1000Ranging.attachInactiveDevice(inactiveDevice);
    // Enable the filter to smooth the distance
    DW1000Ranging.useRangeFilter(true);
    DW1000Ranging.setRangeFilterValue(10);
    // Enable smart power
    DW1000.useSmartPower(true);

    // set antenna delay for anchors only. Tag is default (16384)
    // DW1000.setAntennaDelay(Adelay);
    
    // we start the module as a tag
    DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_2, false); // for group A
    // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
    // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER, false);
    // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY, false);
    // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY, false);
    // DW1000Ranging.startAsTag(TAG_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY, false); // for group B

    // receiving anchor position
    bool ready = false;
    int anchorNum = 0;
    Serial.println("Waiting for anchor position input...");
    while (anchorNum < MAX_NUM_ANCHOR) {
      if (SerialPort1.available()) {
        String received = SerialPort1.readStringUntil('\n');
        for (int i = 0; i < 3; i++) {
          float position = splitString(received, ',', i).toFloat();
          anchor[anchorNum][i] = position;
        }
        anchorNum ++;
      }
    }

    Serial.println("Anchor List:");
    for (int i = 0; i < MAX_NUM_ANCHOR; i++) {
      Serial.print("{ A");
      Serial.print(i + 1);
      Serial.print(": ");
      for (int j = 0; j < 3; j++) {
        Serial.print(anchor[i][j]);
        if (j != 2) {
          Serial.print(",");
        }
        if (j == 2) {
          Serial.println(" }");
        }
      }
    }
    
    //ranging result clear
    resetDistantArr();
}

void loop()
{
  DW1000Ranging.loop();
  device_num = device_num_a;
  if (count == device_num) {
    if (Serial2.available()) {  //read range data from tag B
      for (int i = 4; i < 8; i++) { // reset ranging data of tag B
        distant_list[i] = -1;
      }
      String received = Serial2.readStringUntil('\n');
      // Serial.println(received);
      int device_num_b = splitString(received, ',', 0).toInt();
      device_num = device_num_a + device_num_b;
      for (int i = 1; i < device_num_b + 1; i++) {
        String tempSplited = splitString(received, ',', i);
        // ranging_result.push_back(tempSplited);
        String addrStr = splitString(tempSplited, ':', 0);
        String rangeStr = splitString(tempSplited, ':', 1);
        int index = getAddrNum(addrStr) - 1;
        float range = rangeStr.toFloat();
        if (range > 0 && range < 30) {
          distant_list[index] = range;
        } else {
          distant_list[index] = -1;
        }
        count++;
      }
    }
    // String temp = "";
    // for (int i = 0; i < count; i++) {
    //   temp += ranging_result[i];
    //   temp += ",";
    // }
    // Serial.println(temp);
    // Serial.print("{");
    // for (int i = 0; i < 8; i++) {
    //   Serial.print(distant_list[i]);
    //   if (i != 7) {
    //     Serial.print(",");
    //   }
    // }
    // Serial.println("}");
    for (int i = 0; i < 8; i++) {
      if (i < 7) {
        Serial.print(distant_list[i]);
        Serial.print(", ");
      } else {
        Serial.println(distant_list[i]);
      }
    }
    get_position(distant_list, position_result);
    for (int i = 0; i < 3; i++) {
      if (i < 2) {
        Serial.print(position_result[i]);
        Serial.print(",");
        SerialPort1.print(position_result[i]);
        SerialPort1.print(",");
      } else {
        Serial.println(position_result[i]);
        SerialPort1.println(position_result[i]); // output for kalman filter
      }
    }
    // ranging_result.clear();
    count = 0;
  }
  if (digitalRead(2) == HIGH) {
    Serial.println("ESP32 will reset soon...");
    delay(3000);
    ESP.restart();
  }
}

void newRange() {
  String addr = String(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  int index = getAddrNum(addr) - 1;
  float range = DW1000Ranging.getDistantDevice()->getRange();
  // ranging_result.push_back(addr + ":" + String(range));
  if (range > 0 && range < 30) {
    distant_list[index] = range;
  } else {
    distant_list[index] = -1;
  }
  count++;
  // Serial.print(addr);
  // Serial.print(":");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRange());
  // Serial.println("m");
  // Serial.print("\t RX power: ");
  // Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
  // Serial.println(" dBm");
  
}

void newDevice(DW1000Device *device) {
  Serial.print("ranging init; 1 device added ! -> ");
  Serial.print(" short:");
  Serial.println(device->getShortAddress(), HEX);
  device_num_a++;
}

void inactiveDevice(DW1000Device *device) {
  Serial.print("delete inactive device: ");
  Serial.println(device->getShortAddress(), HEX);
  resetDistantArr();
  device_num_a--;
}

int getAddrNum(String addr) {
  int str_len = addr.length() + 1;
  char char_arr[str_len];
  addr.toCharArray(char_arr, str_len);
  return char_arr[1] - '0';
}

String splitString(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void resetDistantArr() {
  for (int i = 0; i < 8; i++) {
    distant_list[i] = -1;
  }
}
