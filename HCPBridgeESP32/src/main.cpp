static const char* TAG = "main";
#include <Arduino.h>
#include <WiFi.h>
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
#include "../../../private/credentials.h"

#define DHT_PIN 12
#define DHT_VCC_PIN 14
#define DHTTYPE DHT22
#define DELAY_BETWEEN_TEMP_READINGS (10 * 60 * 1000)
DHT dht(DHT_PIN, DHTTYPE);

//======================================================================================================================
// HA Device Parameter
//======================================================================================================================
#define BROKER_ADDR IPAddress(192,168,178,37) // MQTT Broker Address
int mqtt_port = 1883;
WiFiClient client;
HADevice device("Promatic4");
HAMqtt mqtt(client, device, 8);

HACover garagedoor("garagedoor", HACover::PositionFeature);
HAButton button("ventilation");
HASwitch light("garagelight");
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
  }else
  {
    hcistatus.setValue("disconnected");
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

void onButtonCommand(HAButton* sender)
{
    if (sender == &button) {
        emulator.openDoorHalf();
    }
}

void onSwitchCommand(bool state, HASwitch* sender)
{
  emulator.toggleLamp();
  sender->setState(state); // report state back to the Home Assistant
  lightstate = state;
}

void Wifi_disconnected(WiFiEvent_t event, WiFiEventInfo_t info){
  WiFi.disconnect(true);

void Wifi_connected(WiFiEvent_t event, WiFiEventInfo_t info){
  ESP_LOGI(TAG, "Wifi %s : %s connected!", info.wifi_sta_connected.ssid, info.wifi_sta_connected.bssid);
}

//======================================================================================================================
// Setups
//======================================================================================================================
void setup_wifi() {
  WiFi.setHostname("HCP-Bridge");
  WiFi.begin(ssid, password);
  //WiFi.setAutoReconnect(true);
  WiFi.onEvent(Wifi_connected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(Wifi_disconnected, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED); //SYSTEM_EVENT_STA_DISCONNECTED
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup_device(){
  device.setName("Hoermann Promatic 4");
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer("Custom");
  device.setModel("ESP32");
  device.enableSharedAvailability();
  device.enableLastWill();

  garagedoor.setName("Garage Door");
  garagedoor.setDeviceClass("shutter");
  garagedoor.setCurrentPosition(round(emulator.getState().doorCurrentPosition/2));
  garagedoor.onCommand(onCoverCommand);

  button.setIcon("mdi:air-filter");
  button.setName("Garage Ventilation");
  button.onCommand(onButtonCommand);

  light.setIcon("mdi:lightbulb");
  light.setName("Garage Light");
  light.onCommand(onSwitchCommand);

  hcistatus.setIcon("mdi:home");
  hcistatus.setName("HCP-Bridge");
  hcistatus.setValue("disconnected");

  doorstate.setName("Door state");

  tempSensor.setIcon("mdi:thermometer");
  tempSensor.setName("Garage Temperature");
  tempSensor.setUnitOfMeasurement("°C");

  humiditySensor.setIcon("mdi:water-percent");
  humiditySensor.setName("Garage Humidity");
  humiditySensor.setUnitOfMeasurement("%");
}

void setup_mqtt(){
  mqtt.begin(BROKER_ADDR, mqtt_port);
}

void setup(){

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

  pinMode(DHT_VCC_PIN, OUTPUT);
  digitalWrite(DHT_VCC_PIN, HIGH);
  dht.begin();
  ESP_LOGI(TAG, "DHT started");

  setup_mqtt();
  ESP_LOGI(TAG, "MQTT started");

  emulator.onStatusChanged(onStatusChanged);

  hcistatus.setValue(emulator.getState().valid ? "connected" : "disconnected");
}

//======================================================================================================================
// mainloop
//======================================================================================================================
unsigned long lastTempUpdate = 0;

void loop(){
  mqtt.loop();
  if((millis() - lastTempUpdate) > (DELAY_BETWEEN_TEMP_READINGS)) {
    tempSensor.setValue(dht.readTemperature());
    humiditySensor.setValue(dht.readHumidity());
    lastTempUpdate = millis();
  }
}