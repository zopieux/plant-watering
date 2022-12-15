#include <Arduino.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <SoftwareSerial.h>

#include "protocol.h"

String kClient = "plant-gw-", kTopic = "plant/";

WiFiClient wifi;
PubSubClient mqtt(wifi);

SoftwareSerial main_serial(2, 0);

void PublishState(const PlantPayload::Data& pay) {
  if (!mqtt.connected()) return;
  const String topic = kTopic + pay.name + "/";
  mqtt.publish((topic + "connected").c_str(), "online", false);
  mqtt.publish((topic + "pump").c_str(), pay.pump_on ? "1" : "0", false);
  mqtt.publish((topic + "humidity").c_str(), String(pay.dryness, 10).c_str(),
               false);
}

void ConnectWifi() {
  WiFi.disconnect();
  WiFi.begin("Alex", "home sweet 127.0.0.1");
  if (auto err = WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.print("onoes WiFi: ");
    Serial.println(err);
    delay(2000);
    ESP.restart();
    return;
  }
  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" netmask: ");
  Serial.print(WiFi.subnetMask());
  Serial.print(" gateway: ");
  Serial.print(WiFi.gatewayIP());
  Serial.print(" DNS server: ");
  Serial.println(WiFi.dnsIP());
}

void ConnectMqtt() {
  if (!mqtt.connected()) {
    if (mqtt.connect(kClient.c_str(), "iot", "verysec123ret")) {
      Serial.println("mqtt connected");
      mqtt.subscribe((kTopic + "+/power/set").c_str(), 0);
    } else {
      Serial.println("mqtt cannot connect");
      delay(3000);
    }
  }
}

void OnMqtt(char* topic, const uint8_t* payload, unsigned int length) {
  String t(topic);
  if (!t.startsWith(kTopic)) return;
  const String rest = t.substring(kTopic.length());
  int idx = rest.indexOf('/');
  if (idx <= 0) return;
  const String name = rest.substring(0, idx);
  const String action = rest.substring(idx + 1);
  if (action != "power/set") return;
  SetPumpPayload p{};
  if (length == 1 && payload[0] == '1') {
    p.data.pump_on = true;
  } else if (length == 1 && payload[0] == '0') {
    p.data.pump_on = false;
  } else {
    return;
  }
  strncpy(p.data.name, name.c_str(), NAME_SIZE);
  while (!main_serial.availableForWrite())
    ;
  main_serial.write(p.buf, SetPumpPayloadSize);
  main_serial.flush();
  Serial.println("wrote SetPumpPayload to serial");
}

void setup() {
  Serial.begin(9600, SerialConfig::SERIAL_8N1, SERIAL_TX_ONLY);
  Serial.println("hello");

  main_serial.begin(9600, SoftwareSerialConfig::SWSERIAL_8N1);

  uint8_t mac[6];
  WiFi.macAddress(mac);
  char macStr[5] = {0};
  sprintf(macStr, "%02x%02x", mac[4], mac[5]);
  kClient += macStr;

  Serial.println(kClient);
  Serial.println(kTopic);

  WiFi.mode(WIFI_STA);
  WiFi.onStationModeDisconnected(
      [](const WiFiEventStationModeDisconnected&) { ESP.restart(); });

  ConnectWifi();

  mqtt.setServer("192.168.1.10", 1883);
  mqtt.setCallback(OnMqtt);
}

void loop() {
  ConnectMqtt();
  mqtt.loop();

  if (main_serial.available() >= PlantPayloadSize) {
    Serial.println("receiving state");
    PlantPayload payload{};
    const auto& data = main_serial.read(payload.buf, PlantPayloadSize);
    PublishState(payload.data);
  }
}
