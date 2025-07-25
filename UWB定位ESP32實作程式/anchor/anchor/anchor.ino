/*

For ESP32 UWB or ESP32 UWB Pro

*/

#include <SPI.h>
#include "DW1000Ranging.h"
#include "DW1000.h"

#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define DW_CS 4

// connection pins
const uint8_t PIN_RST = 27; // reset pin
const uint8_t PIN_IRQ = 34; // irq pin
const uint8_t PIN_SS = 4;   // spi select pin

//calibrated Antenna Delay setting for this anchor
uint16_t Adelay = 16530;
// leftmost two bytes below will become the "short address"
char ANCHOR_ADD[] = "A4:00:22:EA:82:60:3B:9C";

void setup()
{
  Serial.begin(115200);
  delay(1000);
  //init the configuration
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
  DW1000Ranging.initCommunication(PIN_RST, PIN_SS, PIN_IRQ); //Reset, CS, IRQ pin
  //define the sketch as anchor. It will be great to dynamically change the type of module
  DW1000Ranging.attachNewRange(newRange);
  DW1000Ranging.attachBlinkDevice(newBlink);
  DW1000Ranging.attachInactiveDevice(inactiveDevice);
  //Enable the filter to smooth the distance
  //DW1000Ranging.useRangeFilter(true);
  //Enable smart power
  DW1000.useSmartPower(true);

  // set antenna delay for anchors only. Tag is default (16384)
  DW1000.setAntennaDelay(Adelay);
  
  //we start the module as an anchor
  DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_2, false); // for group A
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_LOWPOWER, DW1000.CHANNEL_5, false); // for group B
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_LOWPOWER, false);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_LOWPOWER, false);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_SHORTDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_FAST_ACCURACY);
  // DW1000Ranging.startAsAnchor(ANCHOR_ADD, DW1000.MODE_LONGDATA_RANGE_ACCURACY, DW1000.CHANNEL_5, false); // for group B
}

void loop()
{
    DW1000Ranging.loop();
}

void newRange()
{
    Serial.print("from: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
    Serial.print("\t Range: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRange());
    Serial.print(" m");
    Serial.print("\t RX power: ");
    Serial.print(DW1000Ranging.getDistantDevice()->getRXPower());
    Serial.println(" dBm");
}

void newBlink(DW1000Device *device)
{
    Serial.print("blink; 1 device added ! -> ");
    Serial.print(" short:");
    Serial.println(device->getShortAddress(), HEX);
}

void inactiveDevice(DW1000Device *device)
{
    Serial.print("delete inactive device: ");
    Serial.println(device->getShortAddress(), HEX);
}
