#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>
#include <WiFiManager.h> 
#include <ArduinoJson.h>

ModbusMaster mb;

#define SerialRS485_RX_PIN    16
#define SerialRS485_TX_PIN    17
#define RS485_DIR_CTRL   4           
#define RS485_RXD_SELECT      LOW
#define RS485_TXD_SELECT      HIGH 

#define SLAVE_ADDRESS_1 1
#define RESET_BUTTON_PIN 0 


const char* mqttServer = "your mqtt server";
const int mqttPort = 1883;
const char* mqttUser = "mqtt user";
const char* mqttPassword = "mqtt password";
const char* device_name = "your device name";

WiFiClient espClient;
PubSubClient client(espClient);

int count = 0;
WiFiManager wifiManager;

void preTransmission() {
  digitalWrite(RS485_DIR_CTRL, RS485_TXD_SELECT);
}

void postTransmission() {
  digitalWrite(RS485_DIR_CTRL, RS485_RXD_SELECT);
}

bool isResetButtonHeld() {
  int holdTime = 0;
  while (digitalRead(RESET_BUTTON_PIN) == LOW) {
    delay(100);
    holdTime += 100;
    if (holdTime >= 3000) {
      Serial.println("Reset button held for 3 seconds!");
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);
  pinMode(RS485_DIR_CTRL, OUTPUT);
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  delay(1000);
  setup_wifi();
  client.setServer(mqttServer, mqttPort);

  mb.begin(SLAVE_ADDRESS_1, Serial2);
  mb.preTransmission(preTransmission);
  mb.postTransmission(postTransmission);
}

void setup_wifi() {
  if (isResetButtonHeld()) {
    Serial.println("Resetting WiFi settings and entering AP mode...");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }
  
  wifiManager.autoConnect("ESP32-Setup");

  Serial.println("Connected to WiFi");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.println("Attempting MQTT Connection...");
    if (client.connect(device_name, mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT Broker");
      client.subscribe("My_Topic");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds");
      delay(5000);
    }
  }
}

void checkResetButton() {
  if (isResetButtonHeld()) {
    Serial.println("Reset button pressed, restarting WiFi setup...");
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
  }
}



void readXYMD02() {
  uint8_t result = mb.readInputRegisters(1, 2); 
  if (result == mb.ku8MBSuccess) {
    float temp_c = mb.getResponseBuffer(0) / 10.0;
    float humi_rh = mb.getResponseBuffer(1) / 10.0;

    StaticJsonDocument<200> jsonDoc;
    jsonDoc["device"] = device_name;
    jsonDoc["temperature"] = temp_c;
    jsonDoc["humidity"] = humi_rh;

    char jsonMessage[200];
    serializeJson(jsonDoc, jsonMessage);

    // your topic
    client.publish("xymd02", jsonMessage);
    Serial.println(jsonMessage); 
  }
}

void loop() {
  checkResetButton();
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  

  readXYMD02();

  delay(1000);
}
