# HCPBridge
Emulates the Hörmann HAP 1 HCP on ESP32 hardware and publishes several entities to monitor and control the garage door to [home assistant](https://www.home-assistant.io).
Optionally, it connects to a DHT11 or DHT22 sensor to also report temperature and humidity.

Emuliert das Hörmann HAP 1 HCP auf ESP32 Hardware und stellt das Tor über mehrere Entitäten in [Home Assistant](https://www.home-assistant.io) bereit.
Optional kann noch ein DHT11 oder DHT22 Sensor zur Temperatur- und Luftfeuchtigkeitsmessung angeschlossen werden.

Credits to TODO

## Compatible Hörmann Garage Door Openers
- SupraMatic E/P Serie 4
- ProMatic Serie 4

Please note that this project only emulates UAP 1 **HCP** and, hence, is **only** compatible with series 4 Hörmann hardware. Older models use a different pinout and protocol.

Bitte beachten, das Projekt emuliert UAP 1 **HCP** und ist auch **nur** mit der Serie 4 kompatibel! Ältere Antriebe als Serie 4 haben eine andere Pinbelegung und ein komplett anderes Protokoll.


## Funktionen:
- Abrufen des aktuellen Status (Tor, Licht)
- Auslösen der Aktionen (Licht an/aus, Tor öffen, schließen, stoppen sowie Lüftungsstellung


## WebInterface:
![alt text](https://github.com/MCLW/HCPBridgeHomeassistant/blob/master/Images/Homeassistant.png)



## Pinout RS485 (Plug):
![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/plug-min.png)
1. GND (Blue)
2. GND (Yellow)
3. B- (Green)
4. A+ (Red)
5. +25V (Black)
6. +25V (White)

## RS485 Adapter:
![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/rs485board-min.png)  
Zwischen A+ (Red) und B- (Green) ist ein 120 Ohm Widerstand zum terminieren des BUS! 

## Schaltung
![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/combo.png) <br/>
ESP8266 + Arduino Combo (Bisher stabil, benötigt allerdings zwei MCU)

![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/esp32.png) <br/>
ESP32 (Bisher stabil durch Nutzung beider Kerne, 25V auf 5V durch LM2596S DC-DC Step Down Module)

![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/schaltung.png) <br/>
ESP8266 ISR (instabil!, WLAN-Probleme können die ModBus-Verbindung beenden, dann ist ein Neuanlernen erforderlich) 

## Installation
![alt text](https://github.com/hkiam/HCPBridge/raw/master/Images/antrieb-min.png)
- Adapter am Bus anschließen (grüner Pfeil)
- Busscan ausführen (blauer Pfeil auf off und wieder zurück auf off). Der Adapter bekommt erst dann Strom über die 25V Leitung und muss während des Busscans antworten, sonst wird der Strom wieder abgeschaltet. Im Falle eines Fehlers oder wenn der Adapter abgezogen werden soll, einfach die Busscan Prozedur (On/Off) wiederholen. 

## Changelog
06.03.2021: Neue Version mit Arduino Pro Mini als Co MCU
24.02.2021: Neue Version via Interrupt und für ESP32 zur Vermeidung von Timing-Problemen
