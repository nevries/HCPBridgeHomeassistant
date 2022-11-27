#include <Arduino.h>
#include <WiFi.h>
#include "hciemulator.h"
#include <ArduinoHA.h>
#include <math.h>

/* create this file and add your wlan and mqtt credentials
  const char* ssid = "MyWLANSID";
  const char* password = "MYPASSWORD";
  const char* mqtt_username   = "USERNAME";
  const char* mqtt_pass       = "PASSWORD";
  const uint16_t mqtt_port    = 1883;
*/
#include "../../../private/credentials.h"

//#define DEBUG

//======================================================================================================================
// HA Device Parameter
//======================================================================================================================
#define BROKER_ADDR IPAddress(192,168,178,50)
WiFiClient client;
HADevice device("Supramatic4");
HAMqtt mqtt(client, device);

HACover garagedoor("garagedoor", HACover::PositionFeature);
HAButton button("ventilation");
HASwitch light("garagelight");
HASensor hcistatus("hcistatus");
HASensor doorstate("doorstate");

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
    // Publish every 5 position changes to prevent modbus timeout
    if (publishDelayCounter == 5){
      garagedoor.setPosition((uint8_t)round(state.doorCurrentPosition/2));
      publishDelayCounter = 0;
    }
    // Publish only if doorstate changes to prevent modbus timeout
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

//======================================================================================================================
// Setups
//======================================================================================================================
void setup_wifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
}

void setup_device(){
  device.setName("Hoermann Supramatic 4");
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
  button.setName("Ventilation");
  button.onCommand(onButtonCommand);

  light.setIcon("mdi:lightbulb");
  light.setName("Garage Light");
  light.onCommand(onSwitchCommand);

  hcistatus.setIcon("mdi:home");
  hcistatus.setName("HCI-Bridge");
  hcistatus.setValue("disconnected");

  doorstate.setName("Door state");
}

void setup_mqtt(){
  mqtt.begin(BROKER_ADDR, mqtt_port, mqtt_username, mqtt_pass);
}

void setup(){

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

  setup_wifi();

  setup_device();

  setup_mqtt();

  emulator.onStatusChanged(onStatusChanged);

  hcistatus.setValue(emulator.getState().valid ? "connected" : "disconnected");

}

//======================================================================================================================
// mainloop
//======================================================================================================================
void loop(){     
  mqtt.loop();
}