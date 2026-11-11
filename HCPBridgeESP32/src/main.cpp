static const char* TAG = "main";
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include "hciemulator.h"
#include <ArduinoHA.h> // https://github.com/dawidchyrzynski/arduino-home-assistant
#include <math.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>

/* create this file and add your wlan and mqtt credentials
  const char* ssid = "MyWLANSID";
  const char* password = "MYPASSWORD";
  const char* mqtt_username   = "USERNAME";
  const char* mqtt_pass       = "PASSWORD";
  const uint16_t mqtt_port    = 1883;
*/
#include "../../../private.h"

#define HOSTNAME "HCP-Bridge"

#define DHT_PIN 33
#define DHT_GND_PIN 13
#define DHT_VCC_PIN 32
#define DHTTYPE DHT22
#define DELAY_BETWEEN_TEMP_READINGS (5 * 60 * 1000)
DHT dht(DHT_PIN, DHTTYPE);

//======================================================================================================================
// HA Device Parameter
//======================================================================================================================
#define BROKER_ADDR IPAddress(192,168,188,4) // MQTT Broker Address
// 192.168.188.4
int mqtt_port = 1883;
WiFiClient client;
WebServer server(80);
HADevice device("Promatic4");
HAMqtt mqtt(client, device, 8);

HACover garagedoor("garagedoor", HACover::PositionFeature);
HAButton ventilationButton("ventilation");
//HAButton halfButton("half");
HALight light("garagelight");
HASensor hcistatus("hcistatus");
HASensor doorstate("doorstate");

HASensorNumber tempSensor("temperature", HASensorNumber::PrecisionP1);
HASensorNumber humiditySensor("humidity", HASensorNumber::PrecisionP1);

//======================================================================================================================
// UART and HCI-Emulator Parameter
//======================================================================================================================
#define RS485 Serial2
#define PIN_TXD 17 // UART 2 TXT - G17
#define PIN_RXD 16 // UART 2 RXD - G16
#define BUILTIN_LED 2

HCIEmulator emulator(&RS485);

//======================================================================================================================
// Modbus Task
//======================================================================================================================
volatile unsigned long lastCall = 0;
volatile unsigned long maxPeriod = 0;

void modBusPolling( void * parameter) {
  while(true){
      if(lastCall>0){
          maxPeriod = _max(micros()-lastCall,maxPeriod);
      }
      lastCall=micros();
      emulator.poll();  
      vTaskDelay(1);    
  }
  vTaskDelete(NULL);
}

TaskHandle_t modBusTask;

//======================================================================================================================
// Convert States
//======================================================================================================================
HACover::CoverState toCoverState(const SHCIState& doorState){
  switch (doorState.doorState)
  {
  case DOOR_MOVE_OPENPOSITION:
    return HACover::StateOpening;
  case DOOR_MOVE_CLOSEPOSITION:
    return HACover::StateClosing;
  case DOOR_OPEN_POSITION:
    return HACover::StateOpen;
  case DOOR_CLOSE_POSITION:
    return HACover::StateClosed;
  case DOOR_HALF_POSITION:
    return HACover::StateOpen;
  default:
    return HACover::StateStopped;
  }
}

const char* doorStateToStr(const SHCIState& doorState){
  switch (doorState.doorState)
  {
  case DOOR_MOVE_OPENPOSITION:
    return "opening";
  case DOOR_MOVE_CLOSEPOSITION:
    return "closing";
  case DOOR_OPEN_POSITION:
    return "open";
  case DOOR_CLOSE_POSITION:
    return "closed";
  case DOOR_HALF_POSITION:
    return "ventilation";
  default:
    return "stopped";
  }
}

//======================================================================================================================
// Events
//======================================================================================================================
uint8_t publishDelayCounter = 0;
uint8_t lastDoorState = 255;
bool lightstate = false;
void onStatusChanged(const SHCIState& state){
  if(state.valid){
    // Publish every 5th position change to prevent Modbus timeouts
    if (publishDelayCounter == 5){
      garagedoor.setPosition((uint8_t)round(state.doorCurrentPosition/2));
      publishDelayCounter = 0;
    }
    // Publish doorstate only if it has changed to prevent modbus timeouts
    if (lastDoorState != state.doorState){
      doorstate.setValue(doorStateToStr(state));
      garagedoor.setPosition((uint8_t)round(state.doorCurrentPosition/2));
    }
    if (lightstate != state.lampOn){
      light.setState(state.lampOn);
      lightstate = state.lampOn;
    }
    garagedoor.setState(toCoverState(state));
    publishDelayCounter += 1;
  }
}

void onCoverCommand(HACover::CoverCommand cmd, HACover* sender) {
    if (cmd == HACover::CommandOpen) {
        emulator.openDoor();
        // sender->setState(HACover::StateOpening); // report state back to the HA
    } else if (cmd == HACover::CommandClose) {
        emulator.closeDoor();
        // sender->setState(HACover::StateClosing); // report state back to the HA
    } else if (cmd == HACover::CommandStop) {
        emulator.stopDoor();
        // sender->setState(HACover::StateStopped); // report state back to the HA
    }
}

// void onHalfButtonCommand(HAButton* sender)
// {
//     if (sender == &halfButton) {
//         emulator.openDoorHalf();
//     }
// }

void onVentilationButtonCommand(HAButton* sender)
{
    if (sender == &ventilationButton) {
        emulator.openDoorHalf();
    }
}

void onLightCommand(bool state, HALight* sender)
{
  if (state) {
    emulator.turnOnLamp();
  } else {
    emulator.turnOffLamp();
  }
  sender->setState(state); // report state back to the Home Assistant
  lightstate = state;
}


//======================================================================================================================
// Status endpoint: GET /status -> JSON with WiFi state and the last WiFi events
//======================================================================================================================
struct WifiEvent { uint32_t t; bool up; uint8_t reason; int8_t rssi; uint8_t ch; uint8_t bssid[6]; };
unsigned long lastTempUpdate = 0;
uint32_t dhtFailures = 0;
WifiEvent wifiEvents[32]; // ring buffer, written from the WiFi event task; torn reads are harmless
volatile uint32_t wifiEventCount = 0;

void logWifiEvent(bool up, uint8_t reason, int8_t rssi, uint8_t ch, const uint8_t *bssid) {
  WifiEvent &e = wifiEvents[wifiEventCount % 32];
  e = {(uint32_t)(millis() / 1000), up, reason, rssi, ch, {0}};
  memcpy(e.bssid, bssid, 6);
  wifiEventCount++;
}

String mac2str(const uint8_t *m) { char b[18]; snprintf(b, sizeof b, "%02x:%02x:%02x:%02x:%02x:%02x", m[0], m[1], m[2], m[3], m[4], m[5]); return b; }

void handleStatus() {
  String s = "{\"uptime_s\":" + String(millis() / 1000) + ",\"reset_reason\":" + String((int)esp_reset_reason()) +
    ",\"bus_tx\":" + String(emulator.txCount) + ",\"bus_max_gap_ms\":" + String(emulator.maxTxGap) +
    ",\"bus_last_tx_ms_ago\":" + String(millis() - emulator.lastTx) + ",\"bus_max_loop_us\":" + String(maxPeriod) +
    ",\"dht_failures\":" + String(dhtFailures) + ",\"free_heap\":" + String(ESP.getFreeHeap()) + ",\"min_free_heap\":" + String(ESP.getMinFreeHeap()) +
    ",\"sketch_md5\":\"" + ESP.getSketchMD5() + "\",\"connected\":" + (WiFi.isConnected() ? "true" : "false") +
    ",\"ssid\":\"" + WiFi.SSID() + "\",\"bssid\":\"" + WiFi.BSSIDstr() + "\",\"channel\":" + String(WiFi.channel()) +
    ",\"rssi\":" + String(WiFi.RSSI()) + ",\"ip\":\"" + WiFi.localIP().toString() + "\",\"wifi_events_total\":" + String(wifiEventCount) +
    ",\"wifi_events\":[";
  uint32_t n = wifiEventCount, first = n > 32 ? n - 32 : 0;
  for (uint32_t i = first; i < n; i++) {
    const WifiEvent &e = wifiEvents[i % 32];
    if (i > first) s += ",";
    s += String("{\"t\":") + e.t + ",\"ev\":\"" + (e.up ? "connected" : "disconnected") + "\",\"bssid\":\"" + mac2str(e.bssid) +
      "\",\"ch\":" + (int)e.ch + ",\"rssi\":" + (int)e.rssi + ",\"reason\":" + (int)e.reason + "}";
  }
  server.send(200, "application/json", s + "]}");
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  const auto &d = info.wifi_sta_disconnected;
  logWifiEvent(false, d.reason, d.rssi, 0, d.bssid);
  WiFi.disconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  const auto &c = info.wifi_sta_connected;
  logWifiEvent(true, 0, 0, c.channel, c.bssid);
  ESP_LOGI(TAG, "Wifi %s : %s connected!", info.wifi_sta_connected.ssid, info.wifi_sta_connected.bssid);
}

//======================================================================================================================
// Setups
//======================================================================================================================
void setup_wifi() {
  WiFi.setHostname(HOSTNAME);
  // scan all channels and pick the strongest AP (default fast scan takes the first one found)
  WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
  WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //WiFi.setAutoReconnect(true);
  WiFi.onEvent(Wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED); //SYSTEM_EVENT_STA_DISCONNECTED
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup_device(){
  device.setName("Hoermann Promatic 4");
  device.setSoftwareVersion("1.1.0");
  device.setManufacturer("Ragnar's Inc");
  device.setModel("ESP32");
  device.enableSharedAvailability();
  device.enableLastWill();

  garagedoor.setName("Garagentor");
  garagedoor.setDeviceClass("shutter");
  garagedoor.setCurrentPosition(round(emulator.getState().doorCurrentPosition/2));
  garagedoor.onCommand(onCoverCommand);

  ventilationButton.setIcon("mdi:air-filter");
  ventilationButton.setName("Garagentor Lüften");
  ventilationButton.onCommand(onVentilationButtonCommand);

  // halfButton.setIcon("mdi:car-sports");
  // halfButton.setName("Garagentor Durchfahren");
  // halfButton.onCommand(onHalfButtonCommand);

  //light.setIcon("mdi:lightbulb");
  light.setName("Garagentor Licht");
  light.onStateCommand(onLightCommand);

  hcistatus.setIcon("mdi:home");
  hcistatus.setName("HCP-Bridge");

  doorstate.setName("Door state");

  tempSensor.setName("Garage Temperatur");
  tempSensor.setDeviceClass("temperature");
  tempSensor.setStateClass("measurement");
  tempSensor.setUnitOfMeasurement("°C");

  humiditySensor.setName("Garage Luftfeuchte");
  humiditySensor.setDeviceClass("humidity");
  humiditySensor.setStateClass("measurement");
  humiditySensor.setUnitOfMeasurement("%");
}

void setup_mqtt(){
  mqtt.begin(BROKER_ADDR, mqtt_port, MQTT_USER, MQTT_PASSWORD);
}

void setup(){
  // internal LED on while setting up things
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, HIGH);

  ESP_LOGI(TAG, "Setup started");

  RS485.begin(57600,SERIAL_8E1,PIN_RXD,PIN_TXD);

  xTaskCreatePinnedToCore(
      modBusPolling, /* Function to implement the task */
      "ModBusTask", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      //1,  /* Priority of the task */
      configMAX_PRIORITIES -1,
      &modBusTask,  /* Task handle. */
      1); /* Core where the task should run */
  ESP_LOGI(TAG, "RS485 task started");

  setup_wifi();
  ESP_LOGI(TAG, "Wifi started");

  setup_device();
  ESP_LOGI(TAG, "Device started");

  // power DHT22 by digital pins and start it
  pinMode(DHT_GND_PIN, OUTPUT);
  digitalWrite(DHT_GND_PIN, LOW);
  pinMode(DHT_VCC_PIN, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);
  dht.begin();
  ESP_LOGI(TAG, "DHT started");

  setup_mqtt();
  ESP_LOGI(TAG, "MQTT started");

  server.on("/status", handleStatus);
  // GET /mute?ms=N (auth, door closed, N<=10000): stop answering the opener for N ms to find its timeout
  server.on("/mute", []() {
    if (!server.authenticate(OTA_USER, OTA_PASSWORD)) return server.requestAuthentication();
    uint32_t ms = server.arg("ms").toInt();
    if (ms == 0 || ms > 10000) return server.send(400, "text/plain", "ms must be 1..10000");
    if (toCoverState(emulator.getState()) != HACover::StateClosed) return server.send(409, "text/plain", "door not closed");
    emulator.muteUntil = millis() + ms;
    server.send(200, "text/plain", "muted " + String(ms) + " ms");
  });
  // Flash writes stall the Modbus task; half/late answers make the opener reboot and kill the upload.
  // Stay silent on the bus during OTA instead (opener only flags error 7, clear it by power-cycling it).
  ElegantOTA.onStart([]() { emulator.muteUntil = millis() + 120000; });
  ElegantOTA.onEnd([](bool ok) { if (!ok) emulator.muteUntil = 0; });
  ElegantOTA.begin(&server, OTA_USER, OTA_PASSWORD); // begin() would reset a prior setAuth()
  server.begin();
  ESP_LOGI(TAG, "OTA server started");

  emulator.onStatusChanged(onStatusChanged);

  // setup done, internal LED off
  digitalWrite(BUILTIN_LED, LOW);
}

//======================================================================================================================
// mainloop
//======================================================================================================================

void loop(){  
  server.handleClient();
  ElegantOTA.loop();
  mqtt.loop();
  // publish bus status from the main loop once MQTT is up (setValue before connect is silently dropped)
  static int8_t lastHciValid = -1;
  int8_t hciValid = emulator.getState().valid;
  if (hciValid != lastHciValid && mqtt.isConnected() && hcistatus.setValue(hciValid ? "connected" : "disconnected")) {
    lastHciValid = hciValid;
  }
  if((millis() - lastTempUpdate) > (DELAY_BETWEEN_TEMP_READINGS)) {
    ESP_LOGD(TAG, "Try to read DHT22...");
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    if (isnan(temp) || isnan(hum)) {
      dhtFailures++; // failed read: keep the last value in HA instead of publishing garbage
    } else {
      tempSensor.setValue(temp);
      humiditySensor.setValue(hum);
    }
    lastTempUpdate = millis();
    ESP_LOGD(TAG, "DHT22 read temp: %.2f hum: %.2f", temp, hum);
  }
}
