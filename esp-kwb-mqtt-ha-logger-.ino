// reads control and sense data from the rs485 bus
// publishes read values via MQTT for HomeAssistant
// Hardware: ESP8266 (wemos d1 mini) + MAX485 module, power supply via USB
// Additionally to the read values, Brennerlaufzeit and Gesamtenergie are calculated, with which pellet consumption can be estimated (~4.3kg/kwh for an EasyFire 2)

// Individual values
// #define WIFISSID "SSID"
// #define WIFIPW  "PW"
// #define MQTTSERVER "IP"
// #define MQTTUSER "USERNAME"
// #define MQTTPASSWORD "PW"
// #define OTAPASSWORD "PW"

#ifndef WIFISSID
#include "conf.h"
#endif

#define LEISTUNGKESSEL 22.0   // KW at 100%
#define TAKT100 (12.5 / 5.0)  // Taktung bei 100% Leistung 5s Laufzeit auf 12.5 sek
#define RLAGESAMTLAUFZEIT 110 // in seconds, look up in boiler menu

int updateEveryMinutes = 1;
// #define PUBLISHUNKNOWN true; // publish control and sense bytes to kwb/ to find specific states while using relay test
// End of individual values

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <stdio.h>
#include <time.h>
#include <ArduinoHA.h>
#include <math.h>

// Globals ********************************************************************
#define MSGMAXLENGTH 256 // in byte
int HauptantriebsImpuls = 0;
long Umdrehungen = 0, ZD = 0, AustragungsGesamtLaufzeit = 0;
long NebenantriebsZeit = 0, HauptantriebsZeit = 0, HANAtimer = 0;  // HANAtimer = Timer zur Ausgabe der Hauptantrieb/Nebenantrieb Ratio

extern long bytecounter;
extern long framecounter;
extern long errorcounter;

//  gemessen 308gr / 229 Umdrehungen = 1.35 g/Umdrehung (passt)
//  Empirische aus 26KW Leistung 1.3338 (???)
//  24,7KW / 4.8 mit 2338 Umdrehungen -> 2.200 (???)
//  22KW * 0.8 * 9,4h * 4kg/h / 1642 Umdrehungen =

// gemessen am Hauptantrieb
// 310gr / 207 Umdrehungen = 1,498 g/Umdrehung (passt) -> 3,58 g/s (???)
// 200gr / 135 Umdrehungen = 1.48 g/Umdrehung (passt) -> auf 56s -> 2,78 g/s (Nein?)
// 298gr /  207 Umdrehungen = 1.44 g/Umdrehung (passt) -> auf 86s > 3,46 g/s (passt)

// macht ca. 207 Umdrehungen / 86s = 2.4 Umdrehungen/s bei Normbetrieb -> 187 Umdrehungen in 3 Min (???)
// nied Fallrohrstand 160 Umdrehungen auf 200gr = 1.25 (???)
// Nebenantrieb/Schnecke: 1990gr mit 373 s = 5.34 gr/s (passt)

float Nebenantriebsfaktor = 5.4;
float Hauptantriebsfakter = (400 / 128.0);  // 400g in 120sek. > 3.333 g/s

unsigned long timerd = 0, lastUpdateCycleMillis = 0;
unsigned long austragungStartedAtMillis = 0;
unsigned long millisAtLastRun = 0; // millis since last loop run
unsigned long millisAtBoilerRestart = 0; // millis since boiler restarted
unsigned long millisRLAOpened = 0; // millis the RLA was opened from fully closed
unsigned long millisRLAStartedToMove = 0;
unsigned long wifiPreviousTime = 0;
int wifiReconnectDelay = 5; //mins

struct ef2 {
  float kwh = 0.1;
  float Rauchgastemperatur = 0.0;
  float Proztemperatur = 0.0;          // Prozessortemperatur? Temperatur der Steuerung?
  float Unterdruck = 0.0;
  float Brennerstunden = 0.0;
  float Kesseltemperatur = 0.0;
  float Geblaese = 0.0;
  float Leistung = 0.0;
  float Saugzug = 0.0;
  float Photodiode = 0.0;
  float Puffertemperatur_unten = 0.0;
  float Puffertemperatur_oben = 0.0;
  float HK1_Aussentemperatur = 0.0;
  float HK1_Vorlauftemperatur = 0.0;
  float Ruecklauftemperatur = 0.0;
  float Boilertemperatur = 0.0;
  float Temp[20];
  int Reinigung = 0;
  int Zuendung = 0;
  int Drehrost = 0;
  int Rauchsauger = 0;
  int AustragungsGesamtLaufzeit = 0;
  int Stoerung1 = 0;
  int Raumaustragung = 0;
  int Hauptantriebimpuls = 0;           // Impulszähler
  int Hauptantrieb = 0;                 // Hauptantrieb Motorlaufzeit in Millisekunden
  int HauptantriebUmdrehungen = 0;      // Umdrehungen Stoker
  int DrehungSaugschlauch = 0;          // -1 left, 0 off, 1 right
  int Boilerpumpe = 0;
  int Heizkreispumpe = 0;
  int Kesselpumpe = 0;                  // in %
  int RLAVentil = 0;                    // -1 close, 0 off, 1 open
  int ext = 0;
  int Kesselstatus = 0;                 // 0 = Off, 1 = ignition, 2 = operation 3 = afterrun
  int HK1_Heizkreismischer = 0;             // -1 close, 0 off, 1 open
  unsigned long HauptantriebsZeit = 1;  // Gesamt Hauptantriebszeit in Millisekunden
  unsigned long Hauptantriebtakt = 1;   // Taktzeit in Millisekunden
  char ctrlMsg[32 * 8 + 33];  // 32 * 0/1 + 33 * space as a String
  char senseMsg[32 * 8 + 33]; // 32 * 0/1 + 33 * space as a String
};

struct ef2 Kessel, oKessel; // current and last boiler state

#include "espinclude.h"

WiFiClient espClient;
HADevice device;
HAMqtt mqtt(espClient, device, 40);

HASensorNumber puffer_oben("kwb_puffer_oben", HASensorNumber::PrecisionP1);
HASensorNumber puffer_unten("kwb_puffer_unten", HASensorNumber::PrecisionP1);
HASensorNumber boiler("kwb_boiler", HASensorNumber::PrecisionP1);
HASensorNumber heizkreis_vorlauf("kwb_heizkreis_vorlauf", HASensorNumber::PrecisionP1);
HASensorNumber heizkreis_aussen("kwb_heizkreis_aussen", HASensorNumber::PrecisionP1);
HASensorNumber kessel_ruecklauf("kwb_kessel_ruecklauf", HASensorNumber::PrecisionP1);
HASensorNumber kessel_temperatur("kwb_kessel_temperatur", HASensorNumber::PrecisionP1);
HASensorNumber kessel_rauchgas("kwb_kessel_rauchgas", HASensorNumber::PrecisionP1);
HABinarySensor kessel_reinigung("kwb_kessel_reinigung");
HABinarySensor kessel_zuendung("kwb_kessel_zuendung");
HABinarySensor kessel_pumpe("kwb_kessel_pumpe");
HABinarySensor kessel_heizkreispumpe("kwb_kessel_heizkreispumpe");
HABinarySensor kessel_boilerpumpe("kwb_kessel_boilerpumpe");
HABinarySensor kessel_raumaustragung("kwb_kessel_raumaustragung");
HABinarySensor kessel_rauchsauger("kwb_kessel_rauchsauger");
// HABinarySensor kessel_anforderung("kwb_kessel_anforderung");
HABinarySensor kessel_stoerung("kwb_kessel_stoerung");
HABinarySensor kessel_drehrost("kwb_kessel_drehrost");
HASensor kessel("kwb_kessel");
HASensorNumber kessel_photodiode("kwb_kessel_photodiode", HASensorNumber::PrecisionP0);
HASensorNumber kessel_geblaese("kwb_kessel_geblaese", HASensorNumber::PrecisionP0);
HASensorNumber kessel_saugzug("kwb_kessel_saugzug", HASensorNumber::PrecisionP0);
HASensorNumber kessel_energie("kwb_kessel_energie", HASensorNumber::PrecisionP3);
HASensorNumber kessel_brennerstunden("kwb_kessel_brennerstunden", HASensorNumber::PrecisionP3);
HASensorNumber kessel_unterdruck("kwb_kessel_unterdruck", HASensorNumber::PrecisionP1);
HASensorNumber kessel_rlageoeffnet("kwb_kessel_rlageoeffnet", HASensorNumber::PrecisionP0);
HASensorNumber kessel_leistung("kwb_kessel_leistung", HASensorNumber::PrecisionP0);

void wifiReconnectIfLost(unsigned long &currentTime) {
  if ((WiFi.status() != WL_CONNECTED) && (currentTime - wifiPreviousTime >= wifiReconnectDelay * 60 * 10)) {
    WiFi.disconnect();
    WiFi.reconnect();
    wifiPreviousTime = currentTime;
  }
}

void wifi_on() {
  byte mac[6];

  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFISSID, WIFIPW);
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  wifiPreviousTime = millis();
}

void configureSensor(HASensorNumber &sensor, const char name[], const char icon[], const char unit[], const char deviceClass[]){
  sensor.setIcon(icon);
  sensor.setName(name);
  sensor.setUnitOfMeasurement(unit);
  if (strlen(deviceClass) != 0)
    sensor.setDeviceClass(deviceClass);
}

void configureSensor(HABinarySensor &sensor, const char name[], const char icon[]){
  sensor.setIcon(icon);
  sensor.setName(name);
}

void configureSensor(HASensor &sensor, const char name[], const char icon[]){
  sensor.setIcon(icon);
  sensor.setName(name);
}

////////////////////// Setup //////////////////////////
void setup() {
  wifi_on();

  Serial.begin(19200);

  ArduinoOTA.begin();
  #ifdef OTAPASSWORD
    ArduinoOTA.setPassword((const char*)OTAPASSWORD);
  #endif

  for (int i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
    Kessel.Temp[i] = 0.0;
  }

  device.setName("KWB Steuerung");
  device.setSoftwareVersion("1.0.0");
  device.setManufacturer("Roy Meissner");
  device.enableSharedAvailability();
  device.enableLastWill();

  // configure sensors
  configureSensor(puffer_oben, "Puffer Oben", "mdi:thermometer", "°C" , "temperature");
  configureSensor(puffer_unten, "Puffer Unten", "mdi:thermometer", "°C" , "temperature");
  configureSensor(boiler, "Boiler", "mdi:thermometer", "°C" , "temperature");
  configureSensor(heizkreis_vorlauf, "Heizkreis Vorlauf", "mdi:thermometer", "°C" , "temperature");
  configureSensor(heizkreis_aussen, "Heizkreis Außen", "mdi:thermometer", "°C" , "temperature");
  configureSensor(kessel_ruecklauf, "Kessel Rücklauf", "mdi:thermometer", "°C" , "temperature");
  configureSensor(kessel_temperatur, "Kessel Temperatur", "mdi:thermometer", "°C" , "temperature");
  configureSensor(kessel_rauchgas, "Kessel Rauchgas", "mdi:thermometer", "°C" , "temperature");
  configureSensor(kessel_energie, "Kessel Energie", "mdi:counter", "kWh", "temperature");
  configureSensor(kessel_brennerstunden, "Kessel Brennerstunden", "mdi:clock-outline", "h" , "duration");
  configureSensor(kessel_unterdruck, "Kessel Unterdruck", "mdi:gauge", "mbar", "pressure");
  configureSensor(kessel_rlageoeffnet, "Kessel RLA geöffnet", "mdi:valve", "%", "");

  configureSensor(kessel_reinigung, "Kessel Reinigung", "mdi:hand-wash-outline");
  configureSensor(kessel_rauchsauger, "Kessel Rauchsauger", "mdi:smoke");
  configureSensor(kessel_zuendung, "Kessel Zündung", "mdi:fire");
  configureSensor(kessel_pumpe, "Kessel Pumpe", "mdi:pump");
  configureSensor(kessel_heizkreispumpe, "Heizkreis Pumpe", "mdi:pump");
  configureSensor(kessel_boilerpumpe, "Boiler Pumpe", "mdi:pump");
  configureSensor(kessel_raumaustragung, "Kessel Raumaustragung", "mdi:warehouse");
  // configureSensor(kessel_anforderung, "Kessel Wärmeanforderung", "mdi:thermometer-plus");
  configureSensor(kessel_stoerung, "Kessel Störung", "mdi:exclamation-thick");
  configureSensor(kessel, "Kessel Status", "mdi:gas-burner");
  configureSensor(kessel_drehrost, "Kessel Drehrost", "mdi:grid");
  configureSensor(kessel_photodiode, "Kessel Photodiode", "mdi:lightbulb", "%", "");
  configureSensor(kessel_geblaese, "Kessel Gebläse", "mdi:fan", "rpm", "");
  configureSensor(kessel_saugzug, "Kessel Saugzug", "mdi:fan", "rpm", "");
  configureSensor(kessel_leistung, "Kessel Leistung", "mdi:flash", "%", "");

  mqtt.begin(MQTTSERVER, MQTTUSER, MQTTPASSWORD);

  lastUpdateCycleMillis = timerd = millis();
}

// String lowerPrecision(float in){
//   char tmp[64];
//   sprintf(tmp, "%.1f", in);
//   return String(tmp);
// }

bool tempdiff(float &a, float &b, float &diff) {
  return (abs(a - b) >= diff);
}

void debugLog(int value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(unsigned long value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(float value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

void publishValueToMQTTOnChange(int &current, int &old, char* topic) {
  if (current != old) {
    debugLog(current, "%d", topic);
    old = current;
  }
}

void publishValueToHAOnChange(int &current, int &old, HABinarySensor &target) {
  if (current != old) {
    target.setState(current);
    old = current;
  }
}

// transforms a message to a byte string, so specific bits can be spotted, which are states of various components, e.g. 01100110 01...
// Byte 0 to X -> Bit 0 to X
// source = source byte array (1 char = 1 byte)
// lengthInBytes = how many bytes shall be converted to bits
// target = target, where to copy the resulting bit array (1 char per bit)
// function from stackoverflow, tested to work correctly
void recordMsgAsBinaryString(unsigned char* source, int lengthInBytes, char* target) {
  int s = 0;
  for (int i = 0; i < lengthInBytes; i++) {
    unsigned char *b = (unsigned char*) &source[i];
    unsigned char byte;
    char tmp[2];

    for (int j = 7; j >= 0; j--) {
      byte = b[0] & (1 << j);
      byte >>= j;
      sprintf(tmp, "%u", byte);
      target[j + (i * 8) + s] = tmp[0]; // add indiv. bits
    }
    target[(i + 1) * 8 + s] = ' '; // add space after 8 bits
    s++;
  }
}

// extract data from the control message
// data = data char array (1 char = 1 byte) = read message
// currentMillis = current milliseconds the MCU is online
// dataLength = length of the data array in byte/chars
void readCTRLMSGFrame(unsigned char* data, unsigned long &currentMillis, int dataLength) {
  #ifdef PUBLISHUNKNOWN
    int recordNoOfBytes = 17; // 0 to 16; what's there after byte 16?; max 32
    recordNoOfBytes = (recordNoOfBytes <= dataLength) ? recordNoOfBytes : dataLength;
    recordMsgAsBinaryString(data, recordNoOfBytes, Kessel.ctrlMsg);
  #endif

  Kessel.Heizkreispumpe = getBit(data, 1, 5);
  if (getBit(data, 1, 7) && getBit(data, 2, 0))            // 1 & 1 = close
    Kessel.HK1_Heizkreismischer = -1;
  else if (getBit(data, 1, 7) && getBit(data, 2, 0) == 0)  // 1 & 0 = open
    Kessel.HK1_Heizkreismischer = 1;
  else                                                     // 0 & 0 = off
    Kessel.HK1_Heizkreismischer = 0;

  if (getBit(data, 2, 3) && getBit(data, 2, 4) && oKessel.RLAVentil != 1) {              // 1 & 1 = open
    Kessel.RLAVentil = 1;
    millisRLAStartedToMove = currentMillis;
  } else if (getBit(data, 2, 3) && getBit(data, 2, 4) == 0 && oKessel.RLAVentil != -1) { // 1 & 0 = close
    Kessel.RLAVentil = -1;
    millisRLAStartedToMove = currentMillis;
  } else if (getBit(data, 2, 3) == 0 && getBit(data, 2, 4) == 0) {                       // 0 & 0 = off
    Kessel.RLAVentil = 0;
  }
  Kessel.Boilerpumpe = getBit(data, 2, 5);
  Kessel.Stoerung1 = 1 - getBit(data, 3, 0);
  Kessel.Drehrost = getBit(data, 3, 6);
  Kessel.Reinigung = getBit(data, 3, 7);
  if (getBit(data, 4, 3) == 1)                              // left
    Kessel.DrehungSaugschlauch = -1;
  else if (getBit(data, 4, 1) == 1)                         // right
    Kessel.DrehungSaugschlauch = 1;
  else                                                      // off
    Kessel.DrehungSaugschlauch = 0;
  Kessel.Rauchsauger = getBit(data, 4, 5);
  Kessel.Kesselpumpe = (getValue(data, 8, 1, 1, false) / 255) * 100;
  Kessel.Raumaustragung = getBit(data, 9, 2) || getBit(data, 9, 5); // Schnecke || Saugturbine
  Kessel.Hauptantriebtakt = getValue(data, 10, 2, 10, false);
  Kessel.Hauptantrieb = getValue(data, 12, 2, 10, false);
  Kessel.Zuendung = getBit(data, 16, 2);

  // calc RLA moving time
  if(Kessel.RLAVentil == 0 && oKessel.RLAVentil != 0) { // jumped to off
    unsigned long RLARuntime = currentMillis - millisRLAStartedToMove; // in MilliSeconds, TODO what if currentMillis overflow?
    if(oKessel.RLAVentil == 1) // opened
      millisRLAOpened += RLARuntime;
    else if(oKessel.RLAVentil == -1) //closed
      millisRLAOpened -= RLARuntime;
    if(millisRLAOpened > RLAGESAMTLAUFZEIT * 1000) // no check for <0 as it is an unsigned long
      millisRLAOpened = RLAGESAMTLAUFZEIT * 1000;
  }
  oKessel.RLAVentil = Kessel.RLAVentil;

  // TODO review the below code
  // Hauptantrieb Range:  0 .. Kessel.Hauptantriebtakt
  if (oKessel.Hauptantriebtakt != 0) // when activated?
    Kessel.HauptantriebsZeit += (oKessel.Hauptantrieb * (currentMillis - millisAtLastRun)) / oKessel.Hauptantriebtakt;

  oKessel.Hauptantrieb = Kessel.Hauptantrieb;
  oKessel.Hauptantriebtakt = Kessel.Hauptantriebtakt;

  float deltat = (currentMillis - millisAtLastRun) / (3600 * 1000.0); // in h
  millisAtLastRun = currentMillis;

  if (Kessel.Leistung > 1) // if burning
    Kessel.Brennerstunden += deltat; // in h
  Kessel.kwh += Kessel.Leistung * deltat; // kW * h = kWh

  // Raumaustragung = SchneckenBunker or Saugturbine
  if (Kessel.Raumaustragung != oKessel.Raumaustragung) {
    if (Kessel.Raumaustragung == 0) { // Raumaustragung switched off
      int austragungslaufzeit = (currentMillis - austragungStartedAtMillis) / 1000; // in seconds
      if (austragungslaufzeit > 800) austragungslaufzeit = 0;  // >800s ??? probably because of misreadings
      Kessel.AustragungsGesamtLaufzeit += austragungslaufzeit;
    } else {                          // Raumaustragung switched on
      austragungStartedAtMillis = currentMillis;
    }
  }
}

// extract data from the sense message
// data = data char array (1 char = 1 byte) = read message
// currentMillis = current milliseconds the MCU is online
// dataLength = length of the data array in byte/chars
void readSenseMSGFrame(unsigned char* data, unsigned long &currentMillis, int dataLength) {
  // States at byte 3 and 4 (maybe 0 to 5?)
  #ifdef PUBLISHUNKNOWN
    int recordNoOfBytes = 6; // 0 to 5; 6 - 23 are known, see below; max 32
    recordNoOfBytes = (recordNoOfBytes <= dataLength) ? recordNoOfBytes : dataLength;
    recordMsgAsBinaryString(data, recordNoOfBytes, Kessel.senseMsg);
  #endif

  Kessel.Hauptantriebimpuls = getBit(data, 3, 7);
  Kessel.ext = getBit(data, 4, 7);
  // sensor values at the follwing bytes, 2 bytes per value
  Kessel.HK1_Vorlauftemperatur = getValue(data, 6, 2, 0.1, true);
  Kessel.Ruecklauftemperatur = getValue(data, 8, 2, 0.1, true);
  Kessel.Boilertemperatur = getValue(data, 10, 2, 0.1, true);
  Kessel.Kesseltemperatur = getValue(data, 12, 2, 0.1, true);
  Kessel.Puffertemperatur_unten = getValue(data, 14, 2, 0.1, true);
  Kessel.Puffertemperatur_oben = getValue(data, 16, 2, 0.1, true);
  Kessel.HK1_Aussentemperatur = getValue(data, 18, 2, 0.1, true);
  Kessel.Rauchgastemperatur = getValue(data, 20, 2, 0.1, true);
  Kessel.Proztemperatur = getValue(data, 22, 2, 0.1, true);
  // see loop for 24 to 31
  Kessel.Photodiode = getValue(data, 32, 2, 0.1, true);
  Kessel.Unterdruck = getValue(data, 34, 2, 0.1, true);
  // see loop for 36 to 68
  Kessel.Saugzug = getValue(data, 69, 2, 0.6, false);
  Kessel.Geblaese = getValue(data, 71, 2, 0.6, false);
  // see loop for 73 to 68
  // what about byte area starting from 89?
  #ifdef PUBLISHUNKNOWN
    for (int i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
      int s = i * 2 + 24;  // 24 to 30
      if (i >= 4)
        s = i * 2 + 28;  // 36 to 50
      else if (i >= 12)  // unknown area between 52 to 68
        s = i * 2 + 49;  // 73 to 87
      if(s + 1 <= dataLength) // s+1 because we read 2 bytes
        Kessel.Temp[i] = getValue(data, s, 2, 0.1, true);
    }
  #endif

  // Range photo -221 to 127 - 348 numbers; x + 255 - offset to zero / range -> 0 to 1 * 100 -> range 0 to 100
  Kessel.Photodiode = round(((Kessel.Photodiode + 221.0) / 348) * 100);
  Kessel.Photodiode = (Kessel.Photodiode < 0) ? 0 : (Kessel.Photodiode > 100) ? 100 : Kessel.Photodiode;
  // old way
  // Kessel.Photodiode = ((int) (Kessel.Photodiode + 255.0) * 100) >> 9; // Result range 6 to 74

  // TODO review the below code
  // zwei gleiche impulse, die vom akt. unterschiedlich sind
  if ((Kessel.Hauptantriebimpuls == oKessel.Hauptantriebimpuls) && (Kessel.Hauptantriebimpuls != HauptantriebsImpuls)) {
    // Hauptantrieb läuft und produziert Impulse
    HauptantriebsImpuls = Kessel.Hauptantriebimpuls;
    Kessel.HauptantriebUmdrehungen++;  // vollst. Takte zählen
    oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
  }  // Impulsende

  oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
}

// Some values change quite fast, which is why they are published immedeately
void publishFastChangingValues() {
  #ifdef PUBLISHUNKNOWN
    if (strcmp(Kessel.senseMsg, oKessel.senseMsg) != 0) {
      mqtt.publish("kwb/senseMsg", Kessel.senseMsg);
      memcpy(&(oKessel.senseMsg), &(Kessel.senseMsg), sizeof(Kessel.senseMsg));
    }
    if (strcmp(Kessel.ctrlMsg, oKessel.ctrlMsg) != 0) {
      mqtt.publish("kwb/ctrlMsg", Kessel.ctrlMsg);
      memcpy(&(oKessel.ctrlMsg), &(Kessel.ctrlMsg), sizeof(Kessel.ctrlMsg));
    }

    for (int i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
      if (tempdiff(Kessel.Temp[i], oKessel.Temp[i], 0.4)) {
        String name = "kwb/temp" + i;
        char buf[64];
        name.toCharArray(buf, name.length());
        debugLog(Kessel.Temp[i], "%d", buf);
        oKessel.Temp[i] = Kessel.Temp[i];
      }
    }
  #endif

  publishValueToMQTTOnChange(Kessel.HK1_Heizkreismischer, oKessel.HK1_Heizkreismischer, "kwb/Heizkreismischer");
  publishValueToMQTTOnChange(Kessel.DrehungSaugschlauch, oKessel.DrehungSaugschlauch, "kwb/DrehungSaugschlauch");

  publishValueToHAOnChange(Kessel.Boilerpumpe, oKessel.Boilerpumpe, kessel_boilerpumpe);
  publishValueToHAOnChange(Kessel.Heizkreispumpe, oKessel.Heizkreispumpe, kessel_heizkreispumpe);
  publishValueToHAOnChange(Kessel.Rauchsauger, oKessel.Rauchsauger, kessel_rauchsauger);
  publishValueToHAOnChange(Kessel.Drehrost, oKessel.Drehrost, kessel_drehrost);
  publishValueToHAOnChange(Kessel.Raumaustragung, oKessel.Raumaustragung, kessel_raumaustragung);
  publishValueToHAOnChange(Kessel.Reinigung, oKessel.Reinigung, kessel_reinigung);
  publishValueToHAOnChange(Kessel.Zuendung, oKessel.Zuendung, kessel_zuendung);

  if (abs(Kessel.Photodiode - oKessel.Photodiode) >= 5) {
    kessel_photodiode.setValue((int)Kessel.Photodiode);
    oKessel.Photodiode = Kessel.Photodiode;
  }
}

void publishBoilerStateToHA (unsigned long &currentMillis) {
  int boilerOnMins = (currentMillis - millisAtBoilerRestart) / (1000 * 60); // minutes since boiler started to run
  if(boilerOnMins < 0) millisAtBoilerRestart = 0; // respect overflow of currentMillis
  if (oKessel.Kesselstatus == 0) { // Off
    if(currentMillis <= 2 * 60 *1000 && (Kessel.Photodiode >= 50 && Kessel.Rauchgastemperatur >= 85)) { // device just started - might be burning (within first 2 minutes after device start)
      kessel.setValue("Brennt");
      Kessel.Kesselstatus = 2;
      millisRLAOpened = RLAGESAMTLAUFZEIT * 1000; // is probably fully opened if boiler is currently burning, faster dail in of the RLA percentage
    } else if (Kessel.Geblaese > 300){
      kessel.setValue("Neustart");
      Kessel.Kesselstatus = 1;
      millisAtBoilerRestart = currentMillis;
    }
  } else if (oKessel.Kesselstatus == 1 && Kessel.Rauchgastemperatur > 85 && Kessel.Geblaese < 2350) { // Restarted & starts to burn
    kessel.setValue("Brennt");
    Kessel.Kesselstatus = 2;
  // NOTE switches too early to afterrun, thus added boilerOnMins to prevent switching just after it started burning, still buggy
  } else if (oKessel.Kesselstatus == 2 && Kessel.Photodiode < 50 && Kessel.Geblaese > 2300 && boilerOnMins > 20) { // Burned & expires
    kessel.setValue("Nachlauf");
    Kessel.Kesselstatus = 3;
  }
  if (Kessel.Photodiode < 20 && Kessel.Geblaese < 300) { // turn off and emergency escape
    kessel.setValue("Aus");
    Kessel.Kesselstatus = 0;
  }
}

// values which change quite slowly
// currentMillis =  current milliseconds the MCU is on
void publishSlowlyChangingValues(unsigned long &currentMillis) {
  puffer_oben.setValue(Kessel.Puffertemperatur_oben);
  puffer_unten.setValue(Kessel.Puffertemperatur_unten);
  boiler.setValue(Kessel.Boilertemperatur);
  heizkreis_vorlauf.setValue(Kessel.HK1_Vorlauftemperatur);
  heizkreis_aussen.setValue(Kessel.HK1_Aussentemperatur);
  kessel_ruecklauf.setValue(Kessel.Ruecklauftemperatur);
  kessel_temperatur.setValue(Kessel.Kesseltemperatur);
  kessel_rauchgas.setValue(Kessel.Rauchgastemperatur);
  kessel_pumpe.setState(Kessel.Kesselpumpe == 100);
  kessel_geblaese.setValue(Kessel.Geblaese);
  kessel_saugzug.setValue(Kessel.Saugzug);
  kessel_energie.setValue(Kessel.kwh);
  kessel_stoerung.setState(Kessel.Stoerung1);
  kessel_brennerstunden.setValue(Kessel.Brennerstunden);
  kessel_unterdruck.setValue(Kessel.Unterdruck);
  // kessel_anforderung.setState((((int)(Kessel.ext)) == 0) ? false : true); // Anfordung not on ext for my boiler

  publishBoilerStateToHA(currentMillis);

  // Approximation by straight line with characteristic points 30-800, 65-1650, 100-2250 (Power - RPM Fan)
  if(Kessel.Geblaese > 2250)
    kessel_leistung.setValue(100);
  else if(Kessel.Geblaese <= 2250 && Kessel.Geblaese >= 750)
    kessel_leistung.setValue((int) (0.0478 * Kessel.Geblaese - 9.89));
  else
    kessel_leistung.setValue(0);

  // (millisRLAOpened / (RLAGESAMTLAUFZEIT * 1000.0)) * 100
  kessel_rlageoeffnet.setValue((int) (millisRLAOpened / (RLAGESAMTLAUFZEIT * 10))); // will be wrong until a full open-close cycle passed
}

// TODO review the code of this function
// currentMillis =  current milliseconds the MCU is on
void otherStuff(unsigned long &currentMillis) {
  // kessel_proztemperatur.setValue(Kessel.Proztemperatur); // HASensorNumber %.1f
  // kessel_HauptantriebUmdrehungen.setValue(Kessel.HauptantriebUmdrehungen); // HASensorNumber int
  // kessel_HauptantriebsZeit.setValue(Kessel.HauptantriebsZeit); // HASensorNumber int
  // kessel_hauptantriebtakt.setValue(Kessel.Hauptantriebtakt / 1000.0); // HASensorNumber %2.1f
  // kessel_pellets.setValue((int) (((Kessel.HauptantriebsZeit)*Hauptantriebsfakter) / 1000)); // HASensorNumber int
  // kessel_pelletsna.setValue((int) ((Kessel.AustragungsGesamtLaufzeit * Nebenantriebsfaktor))); // HASensorNumber int

  framecounter = 0;
  errorcounter = 0;

  // debugLog(framecounter, "%d", "kwb/frames");
  // debugLog(errorcounter, "%d", "kwb/errors");

  // akt Verbrauch berechnen
  if (Kessel.HauptantriebUmdrehungen - Umdrehungen) {
    int d, p;

    d = (Kessel.HauptantriebUmdrehungen - Umdrehungen) * 3600 * 1000 / (currentMillis - timerd);
    // Besp   3.58 * 60 * 60    1000ms / 5000ms
    p = (int)(Hauptantriebsfakter * 60 * 60 * (Kessel.HauptantriebsZeit - ZD)) / (currentMillis - timerd);
    // kessel_deltapelletsh.setValue(p); // HASensorNumber int

    Kessel.Leistung = LEISTUNGKESSEL * TAKT100 * ((float)(Kessel.HauptantriebsZeit - ZD)) / (currentMillis - timerd);
    // kessel_leistung.setValue(Kessel.Leistung); // HASensorNumber %2.1f

    // if (Kessel.Leistung < 1.0 )
    //   kessel_deltapelletsh.setValue(0); // HASensorNumber int

    // Verbrauch pro Stunde gemessen über NA
    // kessel_deltapelletnsh.setValue((int) (Kessel.AustragungsGesamtLaufzeit - AustragungsGesamtLaufzeit) * Nebenantriebsfaktor * 1000 * 3600.0 / ( currentMillis - timerd))); // HASensorNumber int

    AustragungsGesamtLaufzeit = Kessel.AustragungsGesamtLaufzeit;
    Umdrehungen = Kessel.HauptantriebUmdrehungen;
    // kessel_deltat.setValue((millis() - timerd) / 1000); // HASensorNumber int
    ZD = Kessel.HauptantriebsZeit;
    timerd = currentMillis;
  }

  //////////////////////////////////////////////
  // Berechnung HA/NA Verhältnis
  // Alle xx Min  berechnen (-> ca. 1.9 wenn der sinkt gibt es Förderprobleme)
  if (currentMillis > (HANAtimer + 30 * 60 * 1000)) {
    HANAtimer = currentMillis;
    float v;

    if ((Kessel.HauptantriebsZeit - HauptantriebsZeit) && (Kessel.AustragungsGesamtLaufzeit - NebenantriebsZeit)) {  // Wenn der Hauptantrieb lief
      v = (Kessel.HauptantriebsZeit - HauptantriebsZeit) / ((Kessel.AustragungsGesamtLaufzeit - NebenantriebsZeit) * 1000.0);
      // kessel_hana.setValue(v); // HASensorNumber %f
      NebenantriebsZeit = Kessel.AustragungsGesamtLaufzeit;
      HauptantriebsZeit = Kessel.HauptantriebsZeit;
    }
  }

  oKessel.AustragungsGesamtLaufzeit = Kessel.AustragungsGesamtLaufzeit;
  oKessel.Leistung = Kessel.Leistung;
}

//////////////////////////////////////////////////////////////////////////
///////////////////// Main Loop //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned char msgData[MSGMAXLENGTH];
  int dataLength, msgID, frameID;

  mqtt.loop();
  ArduinoOTA.handle();

  bool error = readFrame(msgData, msgID, dataLength, frameID); // Read RS485 dataframe
  // if (error) debugLog(msgID, "Checksum error ID: %d", "kwb/error");

  unsigned long currentMillis = millis();

  if (msgID == 33) // Control message from operating unit to boiler
    readCTRLMSGFrame(msgData, currentMillis, dataLength);
  else if (msgID == 32) // Sense message from boiler to operating unit
    readSenseMSGFrame(msgData, currentMillis, dataLength);

  publishFastChangingValues();

  //////////////////// timed update block (e.g. each minute) ////////////////////
  if (currentMillis > (lastUpdateCycleMillis + updateEveryMinutes * 60 * 1000)) {

    bytecounter = 0;

    publishSlowlyChangingValues(currentMillis);
    otherStuff(currentMillis); // NOTE not sure this need yet (code not reviewed), but kept here for convenience

    memcpy(&oKessel, &Kessel, sizeof Kessel);
    lastUpdateCycleMillis = currentMillis;
    wifiReconnectIfLost(currentMillis);
  }

  delay(5); // delay at the end of the loop to not trigger the SW Watchdog
}
