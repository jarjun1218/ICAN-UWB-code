#include <esp_now.h>
#include <WiFi.h>

#define MAX_NUM_ANCHOR 8
uint8_t broadcastAddress[] = {0xB8, 0xD6, 0x1A, 0x80, 0x5F, 0x34};
bool ready = false, leave = false;

struct Position {
  float x;
  float y;
  float z;
};
Position receivedData;

struct Packet {
  bool status;
  int num;
  float anchor[MAX_NUM_ANCHOR][3];
};
Packet sendingData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Serial.print("Sending Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == ESP_NOW_SEND_SUCCESS) {
    ready = true;
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));
  Serial.printf("%05.2f,%05.2f,%05.2f\n", receivedData.x, receivedData.y, receivedData.z);
  // Serial.print(receivedData.x);
  // Serial.print(",");
  // Serial.print(receivedData.y);
  // Serial.print(",");
  // Serial.println(receivedData.z);
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

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  // Serial.println(WiFi.macAddress());
  
  // initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error Initializing ESP-NOW");
    return;
  } else {
    Serial.println("ESP-NOW Initialized");
  }

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  // register callback function
  esp_now_register_recv_cb(OnDataRecv);

  while (!ready) {
    int numOfAnchor = 0;
    // initialize the anchor position
    for (int i = 0; i < MAX_NUM_ANCHOR; i++) {
      for (int j = 0; j < 3; j++) {
        sendingData.anchor[i][j] = 0;
      }
    }

    // read the data from serial monitor
    if (Serial.available()) {
      String input = Serial.readStringUntil('\n'); // formate: account;index1,x1,y1,z1;...;...;
      // Serial.println(input);
      if (input != "exit") {
        numOfAnchor = splitString(input, ';', 0).toInt();
        // Serial.println(numOfAnchor);
        for (int i = 1; i <= numOfAnchor; i++) {
          String str = splitString(input, ';', i);
          // Serial.println(str);
          int num = splitString(str, ',', 0).toInt() - 1;
          for (int j = 1; j < 4; j++) {
            float temp = splitString(str, ',', j).toFloat();
            sendingData.anchor[num][j-1] = temp;
            // Serial.println(temp);
          }
        }
        // for (int i = 0; i < 8; i++) {
        //   for (int j = 0; j < 4; j++) {
        //     if (j != 3) {
        //       Serial.print(sendingData.anchor[i][j]);
        //     } else {
        //       Serial.println(sendingData.anchor[i][j]);
        //     }
        //   }
        // }
        sendingData.status = true;
        // Send message via ESP-NOW
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &sendingData, sizeof(sendingData));
        if (result == ESP_OK) {
        } else {
          Serial.println("\r\nError sending the data");
        }
      } else if (input == "exit") {
        leave = true;
        ready = true;
        break;
      }
    }
  }
  if (!leave) {
    Serial.println("Start Positioning!");
  }
}

void loop() {
  if (Serial.available()) {
    if (Serial.readStringUntil('\n') == "reset") {
      sendingData.status = false;
      esp_now_send(broadcastAddress, (uint8_t *) &sendingData, sizeof(sendingData));
      delay(3000);
      ESP.restart();
    }
  }
}

