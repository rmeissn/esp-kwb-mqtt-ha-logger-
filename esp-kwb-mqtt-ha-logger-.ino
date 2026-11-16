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

int updateEverySeconds = 30;
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
long NebenantriebsZeit = 0, HauptantriebsZeit = 0;
unsigned long HANAtimer = 0;  // HANAtimer = Timer zur Ausgabe der Hauptantrieb/Nebenantrieb Ratio

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
unsigned long lastWifiConnectedMillis = 0;
int wifiReconnectDelay = 5; // mins
const unsigned long disconnectRestartTimeoutMs = 15UL * 60UL * 1000UL; // restart if WiFi stays disconnected for 15 min

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
  int Kesselstatus = -1;                // -1 = uninitialized, 0 = Off, 1 = ignition, 2 = operation, 3 = afterrun
  int HK1_Heizkreismischer = 0;             // -1 close, 0 off, 1 open
  unsigned long HauptantriebsZeit = 1;  // Gesamt Hauptantriebszeit in Millisekunden
  unsigned long Hauptantriebtakt = 1;   // Taktzeit in Millisekunden
  char ctrlMsg[32 * 8 + 33];  // 32 * 0/1 + 33 * space as a String
  char senseMsg[32 * 8 + 33]; // 32 * 0/1 + 33 * space as a String
};

struct ef2 Kessel, oKessel; // current and last boiler state

struct FastPublishedState {
  int boilerpumpe = -1;
  int heizkreispumpe = -1;
  int rauchsauger = -1;
  int drehrost = -1;
  int raumaustragung = -1;
  int reinigung = -1;
  int zuendung = -1;
  int stoerung = -1;
  int kesselpumpe = -1;
  int photodiode = -1;
  int heizkreismischer = -99;
  int drehungSaugschlauch = -99;
};
FastPublishedState lastPublished;

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

void checkConnections(unsigned long currentMillis) {
  if (WiFi.status() == WL_CONNECTED) {
    lastWifiConnectedMillis = currentMillis;
    wifiPreviousTime = currentMillis;
  } else {
    if ((currentMillis - lastWifiConnectedMillis) >= disconnectRestartTimeoutMs) {
      ESP.restart();
    }
    if ((currentMillis - wifiPreviousTime) >= ((unsigned long)wifiReconnectDelay * 60UL * 1000UL)) {
      WiFi.reconnect();
      wifiPreviousTime = currentMillis;
    }
  }
}

void wifi_on() {
  byte mac[6];

  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFISSID, WIFIPW);
  if(WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  wifiPreviousTime = lastWifiConnectedMillis = millis();
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
  configureSensor(kessel_energie, "Kessel Energie", "mdi:counter", "kWh", "energy");
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

bool tempdiff(float a, float b, float diff) {
  return (fabsf(a - b) >= diff);
}

void debugLog(int value, const char* formatter, const char* topic) {
  char msg[64];
  snprintf(msg, sizeof(msg), formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(long value, const char* formatter, const char* topic) {
  char msg[64];
  snprintf(msg, sizeof(msg), formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(unsigned long value, const char* formatter, const char* topic) {
  char msg[64];
  snprintf(msg, sizeof(msg), formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(float value, const char* formatter, const char* topic) {
  char msg[64];
  snprintf(msg, sizeof(msg), formatter, value);
  mqtt.publish(topic, msg);
}

void publishValueToMQTTOnChange(int &current, int &old, const char* topic) {
  if (current != old) {
    debugLog(current, "%d", topic);
    old = current;
  }
}

void publishValueToHAOnChange(int current, int &old, HABinarySensor &target) {
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
  if (lengthInBytes < 0) lengthInBytes = 0;
  if (lengthInBytes > 32) lengthInBytes = 32; // matches char[32 * 8 + 33] buffer size
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
  target[lengthInBytes * 8 + s] = '\0';
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
  if (Kessel.RLAVentil == 0 && oKessel.RLAVentil != 0) { // jumped to off
    unsigned long RLARuntime = currentMillis - millisRLAStartedToMove; // in MilliSeconds
    unsigned long maxRLAMillis = (unsigned long)RLAGESAMTLAUFZEIT * 1000UL;
    if (oKessel.RLAVentil == 1) { // opened
      millisRLAOpened = (RLARuntime >= maxRLAMillis || millisRLAOpened > maxRLAMillis - RLARuntime)
        ? maxRLAMillis
        : (millisRLAOpened + RLARuntime);
    } else if (oKessel.RLAVentil == -1) { // closed
      millisRLAOpened = (RLARuntime >= millisRLAOpened) ? 0 : (millisRLAOpened - RLARuntime);
    }
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
    oKessel.Raumaustragung = Kessel.Raumaustragung;
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
      if (i >= 12)       // unknown area between 52 to 68
        s = i * 2 + 49;  // 73 to 87
      else if (i >= 4)
        s = i * 2 + 28;  // 36 to 50
      if (s + 1 < dataLength) // s+1 because we read 2 bytes (indices s and s+1)
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

    for (size_t i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
      if (tempdiff(Kessel.Temp[i], oKessel.Temp[i], 0.4f)) {
        char buf[24];
        snprintf(buf, sizeof(buf), "kwb/temp%u", (unsigned int)i);
        debugLog(Kessel.Temp[i], "%.1f", buf);
        oKessel.Temp[i] = Kessel.Temp[i];
      }
    }
  #endif

  if (!mqtt.isConnected()) {
    lastPublished = FastPublishedState();
    return;
  }

  publishValueToMQTTOnChange(Kessel.HK1_Heizkreismischer, lastPublished.heizkreismischer, "kwb/Heizkreismischer");
  publishValueToMQTTOnChange(Kessel.DrehungSaugschlauch, lastPublished.drehungSaugschlauch, "kwb/DrehungSaugschlauch");
  // publishValueToMQTTOnChange(Kessel.RLAVentil, oKessel.RLAVentil, "kwb/RLAVentil");

  publishValueToHAOnChange(Kessel.Boilerpumpe, lastPublished.boilerpumpe, kessel_boilerpumpe);
  publishValueToHAOnChange(Kessel.Heizkreispumpe, lastPublished.heizkreispumpe, kessel_heizkreispumpe);
  publishValueToHAOnChange(Kessel.Rauchsauger, lastPublished.rauchsauger, kessel_rauchsauger);
  publishValueToHAOnChange(Kessel.Drehrost, lastPublished.drehrost, kessel_drehrost);
  publishValueToHAOnChange(Kessel.Raumaustragung, lastPublished.raumaustragung, kessel_raumaustragung);
  publishValueToHAOnChange(Kessel.Reinigung, lastPublished.reinigung, kessel_reinigung);
  publishValueToHAOnChange(Kessel.Zuendung, lastPublished.zuendung, kessel_zuendung);
  publishValueToHAOnChange(Kessel.Stoerung1, lastPublished.stoerung, kessel_stoerung);
  publishValueToHAOnChange((Kessel.Kesselpumpe == 100) ? 1 : 0, lastPublished.kesselpumpe, kessel_pumpe);

  int currentPhotodiode = (int)Kessel.Photodiode;
  if ((lastPublished.photodiode < 0) || (abs(currentPhotodiode - lastPublished.photodiode) >= 5)) {
    kessel_photodiode.setValue(currentPhotodiode);
    lastPublished.photodiode = currentPhotodiode;
  }
}

bool conditionHeld(bool cond, bool &active, unsigned long &sinceMillis, unsigned long currentMillis, unsigned long holdMs) {
  if (!cond) {
    active = false;
    return false;
  }
  if (!active) {
    active = true;
    sinceMillis = currentMillis;
    return false;
  }
  return (currentMillis - sinceMillis) >= holdMs;
}

void publishBoilerStateToHA(unsigned long currentMillis) {
  static const char* const STATE_LABELS[] = {"Aus", "Neustart", "Brennt", "Nachlauf"};
  const unsigned long OFF_HOLD_MS = 10000UL;      // 10 s: catches brief ~1 min stops without jitter
  const unsigned long START_HOLD_MS = 5000UL;     // 5 s: confirm restart
  const unsigned long BURN_HOLD_MS = 30000UL;     // 30 s: avoid premature Brennt on ignition spike
  const unsigned long AFTERRUN_HOLD_MS = 45000UL; // 45 s: enter Nachlauf promptly
  const unsigned long RECOVER_HOLD_MS = 30000UL;  // 30 s: return from Nachlauf to Brennt

  static int lastPublishedState = -1;
  static bool offActive = false, phaseActive = false;
  static unsigned long offSince = 0, phaseSince = 0;
  static float startMinFlueTemp = 0.0;

  bool isOff = (Kessel.Rauchsauger == 0) && (Kessel.Zuendung == 0) &&
               (Kessel.Geblaese < 300) && (Kessel.Saugzug < 300) && (Kessel.Photodiode < 20);
  bool isStarting = (Kessel.Reinigung == 0) &&
                    ((Kessel.Zuendung == 1) ||
                     ((Kessel.Rauchsauger == 1) && ((Kessel.Geblaese > 300) || (Kessel.Saugzug > 300))));
  // Neustart fan ~2583 rpm vs steady burn <= ~2317 rpm; fallback at >= 115 C avoids getting stuck on heavy full load
  bool isBurningSteady = (Kessel.Rauchsauger == 1) && (Kessel.Photodiode >= 45) &&
                         (((Kessel.Rauchgastemperatur >= 85.0) && (Kessel.Geblaese < 2450)) ||
                          ((Kessel.Rauchgastemperatur >= 115.0) && (Kessel.Photodiode >= 60)));
  // Nachlauf fan ~2497+ rpm while photodiode drops < 45%
  bool isAfterrun = (Kessel.Rauchsauger == 1) && (Kessel.Reinigung == 0) &&
                    (Kessel.Photodiode < 45) && (Kessel.Geblaese > 2380);

  if (Kessel.Kesselstatus < 0) {
    if (isBurningSteady) {
      Kessel.Kesselstatus = 2;
      millisAtBoilerRestart = currentMillis;
      millisRLAOpened = RLAGESAMTLAUFZEIT * 1000UL;
    } else if (isStarting) {
      Kessel.Kesselstatus = 1;
      millisAtBoilerRestart = currentMillis;
      startMinFlueTemp = Kessel.Rauchgastemperatur;
    } else {
      Kessel.Kesselstatus = 0;
    }
  } else {
    int nextState = Kessel.Kesselstatus;
    if ((Kessel.Kesselstatus != 0) && conditionHeld(isOff, offActive, offSince, currentMillis, OFF_HOLD_MS)) {
      nextState = 0;
    } else {
      unsigned long boilerOnMins = (currentMillis - millisAtBoilerRestart) / 60000UL;
      switch (Kessel.Kesselstatus) {
        case 0: // Aus
          if (conditionHeld(isStarting, phaseActive, phaseSince, currentMillis, START_HOLD_MS))
            nextState = 1;
          break;
        case 1: // Neustart: require flue temp to rise >= 5 C above restart minimum so cooling hot-restarts do not jump to Brennt
          if (Kessel.Rauchgastemperatur < startMinFlueTemp)
            startMinFlueTemp = Kessel.Rauchgastemperatur;
          if (conditionHeld(isBurningSteady && (Kessel.Rauchgastemperatur >= (startMinFlueTemp + 5.0)),
                            phaseActive, phaseSince, currentMillis, BURN_HOLD_MS))
            nextState = 2;
          break;
        case 2: // Brennt
          if (conditionHeld(isAfterrun && (boilerOnMins >= 8), phaseActive, phaseSince, currentMillis, AFTERRUN_HOLD_MS))
            nextState = 3;
          break;
        case 3: // Nachlauf
          if (conditionHeld(isBurningSteady && ((Kessel.Photodiode >= 55) || (Kessel.Geblaese <= 2250)),
                            phaseActive, phaseSince, currentMillis, RECOVER_HOLD_MS))
            nextState = 2;
          break;
        default:
          nextState = 0;
          break;
      }
    }

    if (nextState != Kessel.Kesselstatus) {
      Kessel.Kesselstatus = nextState;
      offActive = false;
      phaseActive = false;
      if (nextState == 1) {
        millisAtBoilerRestart = currentMillis;
        startMinFlueTemp = Kessel.Rauchgastemperatur;
      }
    }
  }

  if (!mqtt.isConnected()) {
    lastPublishedState = -1;
  } else if (lastPublishedState != Kessel.Kesselstatus) {
    kessel.setValue(STATE_LABELS[Kessel.Kesselstatus]);
    lastPublishedState = Kessel.Kesselstatus;
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
  kessel_geblaese.setValue(Kessel.Geblaese);
  kessel_saugzug.setValue(Kessel.Saugzug);
  kessel_energie.setValue(Kessel.kwh);
  kessel_brennerstunden.setValue(Kessel.Brennerstunden);
  kessel_unterdruck.setValue(Kessel.Unterdruck);
  // kessel_anforderung.setState((((int)(Kessel.ext)) == 0) ? false : true); // Anfordung not on ext for my boiler

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
  // debugLog(Kessel.Proztemperatur, "%.1f", "kwb/Proztemperatur");
  // debugLog(Kessel.HauptantriebUmdrehungen, "%d", "kwb/HauptantriebUmdrehungen");
  // debugLog(Kessel.HauptantriebsZeit, "%lu", "kwb/HauptantriebsZeit");
  // debugLog(Kessel.Hauptantriebtakt, "%lu", "kwb/Hauptantriebtakt");
  // debugLog(Kessel.Hauptantrieb, "%d", "kwb/Hauptantrieb");
  // debugLog(Kessel.AustragungsGesamtLaufzeit, "%d", "kwb/AustragungsGesamtLaufzeit");
  // kessel_proztemperatur.setValue(Kessel.Proztemperatur); // HASensorNumber %.1f
  // kessel_HauptantriebUmdrehungen.setValue(Kessel.HauptantriebUmdrehungen); // HASensorNumber int
  // kessel_HauptantriebsZeit.setValue(Kessel.HauptantriebsZeit); // HASensorNumber int
  // kessel_hauptantriebtakt.setValue(Kessel.Hauptantriebtakt / 1000.0); // HASensorNumber %2.1f
  // kessel_pellets.setValue((int) (((Kessel.HauptantriebsZeit)*Hauptantriebsfakter) / 1000)); // HASensorNumber int
  // kessel_pelletsna.setValue((int) ((Kessel.AustragungsGesamtLaufzeit * Nebenantriebsfaktor))); // HASensorNumber int

  // debugLog(framecounter, "%ld", "kwb/frames");
  // debugLog(errorcounter, "%ld", "kwb/errors");

  framecounter = 0;
  errorcounter = 0;

  // akt Verbrauch berechnen
  unsigned long deltaMs = currentMillis - timerd;
  if ((Kessel.HauptantriebUmdrehungen != Umdrehungen) && (deltaMs > 0)) {
    int d, p;

    d = (int)(((long long)(Kessel.HauptantriebUmdrehungen - Umdrehungen) * 3600000LL) / (long long)deltaMs);
    // Besp   3.58 * 60 * 60    1000ms / 5000ms
    p = (int)((Hauptantriebsfakter * 3600.0f * (float)(Kessel.HauptantriebsZeit - ZD)) / (float)deltaMs);
    // kessel_deltapelletsh.setValue(p); // HASensorNumber int

    Kessel.Leistung = LEISTUNGKESSEL * TAKT100 * ((float)(Kessel.HauptantriebsZeit - ZD)) / (float)deltaMs;
    // kessel_leistung.setValue(Kessel.Leistung); // HASensorNumber %2.1f

    // if (Kessel.Leistung < 1.0 )
    //   kessel_deltapelletsh.setValue(0); // HASensorNumber int

    // Verbrauch pro Stunde gemessen über NA
    // kessel_deltapelletnsh.setValue((int)(((Kessel.AustragungsGesamtLaufzeit - AustragungsGesamtLaufzeit) * Nebenantriebsfaktor * 3600000.0f) / (float)deltaMs)); // HASensorNumber int

    AustragungsGesamtLaufzeit = Kessel.AustragungsGesamtLaufzeit;
    Umdrehungen = Kessel.HauptantriebUmdrehungen;
    // kessel_deltat.setValue(deltaMs / 1000); // HASensorNumber int
    ZD = Kessel.HauptantriebsZeit;
    timerd = currentMillis;
  }

  //////////////////////////////////////////////
  // Berechnung HA/NA Verhältnis
  // Alle xx Min  berechnen (-> ca. 1.9 wenn der sinkt gibt es Förderprobleme)
  if ((currentMillis - HANAtimer) >= (30UL * 60UL * 1000UL)) {
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
  static unsigned char msgData[MSGMAXLENGTH];
  static bool hasCtrlFrame = false;
  static bool hasSenseFrame = false;
  int dataLength = 0, msgID = 0, frameID = 0;

  mqtt.loop();
  ArduinoOTA.handle();

  unsigned long currentMillis = millis();
  FrameStatus frameStatus = readFrame(msgData, msgID, dataLength, frameID, currentMillis); // Read RS485 dataframe non-blocking
  // if (frameStatus == FRAME_ERROR) debugLog(msgID, "Checksum error ID: %d", "kwb/error");

  if (frameStatus == FRAME_OK) {
    if (msgID == 33) { // Control message from operating unit to boiler
      readCTRLMSGFrame(msgData, currentMillis, dataLength);
      hasCtrlFrame = true;
    } else if (msgID == 32) { // Sense message from boiler to operating unit
      readSenseMSGFrame(msgData, currentMillis, dataLength);
      hasSenseFrame = true;
    }

    if (hasCtrlFrame && hasSenseFrame) {
      publishBoilerStateToHA(currentMillis);
      publishFastChangingValues();
    }
  }

  checkConnections(currentMillis);

  //////////////////// timed update block (e.g. every 30 seconds) ////////////////////
  if ((currentMillis - lastUpdateCycleMillis) >= ((unsigned long)updateEverySeconds * 1000UL)) {

    bytecounter = 0;

    if (mqtt.isConnected() && hasCtrlFrame && hasSenseFrame) {
      publishSlowlyChangingValues(currentMillis);
    }
    otherStuff(currentMillis); // NOTE not sure this need yet (code not reviewed), but kept here for convenience

    memcpy(&oKessel, &Kessel, sizeof Kessel);
    lastUpdateCycleMillis = currentMillis;
  }

  if ((frameStatus != FRAME_OK) || !Serial.available()) {
    delay(5); // delay at the end of the loop to not trigger the SW Watchdog
  }
}
