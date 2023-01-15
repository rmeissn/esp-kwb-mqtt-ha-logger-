// reads control and sense data from the rs485 bus
// Hardware
// Webmos d1 mini + MAX485 module
// power supply via USB
// Additionally to the read values, Brennerlaufzeit and Gesamtenergie are calculated, with which pellet consumption can be estimated (~4.3kg/kwh for an EF2)

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

#define LEISTUNGKESSEL 22.0   // KW at 100 %
#define TAKT100 (12.5 / 5.0)  // Taktung bei 100% Leistung 5s Laufzeit auf 12.5 sek

int updateEveryMinutes = 1;
bool publishUnknown = false; // publish control and sense bytes to kwb/ to find specific states while using relay test

// End of individual values

#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <stdio.h>
#include <time.h>
#include <ArduinoHA.h>
#include <math.h>

#define STATE_WAIT_FOR_HEADER 1
#define STATE_READ_MSG 2
#define MSG_TYPE_CTRL 1
#define MSG_TYPE_SENSE 2
#define FALSE 0
#define TRUE 1

// Globals

int HauptantriebsImpuls = 0;
long Umdrehungen = 0, ZD = 0, AustragungsGesamtLaufzeit = 0;
long NebenantriebsZeit = 0, HauptantriebsZeit = 0, HANAtimer = 0;  // Timer zur Ausgabe der Hauptantrieb/Nebenantrieb Ratio

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

double Nebenantriebsfaktor = 5.4;
double Hauptantriebsfakter = (400.0 / 128.0);  // 400g in 120sek. > 3.333 g/s

unsigned long bytecount = 0;
unsigned long waitcount = 0;
unsigned long longwaitcount = 0;
unsigned long timerd = 0, lastUpdateCycleMillis = 0;
unsigned long austragungStartedAtMillis = 0;
unsigned long timerHauptantrieb = 0;
unsigned long kwhtimer = 0;  // Zeit seit letzer KW Messung

struct ef2 {
  double kwh = 0.1;
  double Rauchgastemperatur = 0.0;
  double Proztemperatur = 0.0;  // Prozessortemperatur? Temperatur der Steuerung?
  double Unterdruck = 0.0;
  double Brennerstunden = 0.0;
  double Kesseltemperatur = 0.0;
  double Geblaese = 0.0;
  double Leistung = 0.0;
  double Saugzug = 0.0;
  double photo = 0.0;
  double Puffer_unten = 0.0;
  double Puffer_oben = 0.0;
  double HK1_aussen = -100.0;
  double HK1_Vorlauf = 0.0;
  double Ruecklauf = 0.0;
  double Boiler = 0.0;
  double Temp[20];
  int Reinigung = 0;
  int Zuendung = 0;
  int Drehrost = 0;
  int Rauchsauger = 0;
  int Austragungslaufzeit = 0;
  int AustragungsGesamtLaufzeit = 0;
  int Stoerung1 = 0;
  int Raumaustragung = 0;
  int Hauptantriebimpuls = 0;           // Impulszähler
  int Hauptantrieb = 0;                 // Hauptantrieb Motorlaufzeit in Millisekunden
  int HauptantriebUmdrehungen = 0;      // Umdrehungen Stoker
  int DrehungSaugschlauch = 0;          // -1 left, 0 off, 1 right
  int BoilerPumpe = 0;
  int HeizkreisPumpe = 0;
  int KesselPumpe = 0;                  // in %
  int RLAVentil = 0;                    // -1 close, 0 off, 1 open
  int ext = 1;
  int Kesselstatus = 0;                 // 0 = Off, 1 = ignition, 2 = operation 3 = afterrun
  int Heizkreismischer = 0;             // -1 close, 0 off, 1 open
  unsigned long HauptantriebsZeit = 1;  // Gesamt Hauptantriebszeit in Millisekunden
  unsigned long Hauptantriebtakt = 1;   // Taktzeit in Millisekunden
  char ctrlMsg[32 * 8];                   // 32 byte
  char senseMsg[32 * 8];                  // 32 byte
};

struct ef2 Kessel, oKessel;  // akt. und "letzter" Kesselzustand

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

unsigned long wifiPreviousTime = 0;
int wifiReconnectDelay = 5; //mins

void wifiReconnectIfLost(unsigned long currentTime) {
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
  puffer_oben.setIcon("mdi:thermometer");
  puffer_oben.setName("Puffer Oben");
  puffer_oben.setDeviceClass("temperature");
  puffer_oben.setUnitOfMeasurement("°C");
  puffer_unten.setIcon("mdi:thermometer");
  puffer_unten.setName("Puffer Unten");
  puffer_unten.setDeviceClass("temperature");
  puffer_unten.setUnitOfMeasurement("°C");
  boiler.setIcon("mdi:thermometer");
  boiler.setName("Boiler");
  boiler.setDeviceClass("temperature");
  boiler.setUnitOfMeasurement("°C");
  heizkreis_vorlauf.setIcon("mdi:thermometer");
  heizkreis_vorlauf.setName("Heizkreis Vorlauf");
  heizkreis_vorlauf.setDeviceClass("temperature");
  heizkreis_vorlauf.setUnitOfMeasurement("°C");
  heizkreis_aussen.setIcon("mdi:thermometer");
  heizkreis_aussen.setName("Heizkreis Außen");
  heizkreis_aussen.setDeviceClass("temperature");
  heizkreis_aussen.setUnitOfMeasurement("°C");
  kessel_ruecklauf.setIcon("mdi:thermometer");
  kessel_ruecklauf.setName("Kessel Rücklauf");
  kessel_ruecklauf.setDeviceClass("temperature");
  kessel_ruecklauf.setUnitOfMeasurement("°C");
  kessel_temperatur.setIcon("mdi:thermometer");
  kessel_temperatur.setName("Kessel Temperatur");
  kessel_temperatur.setDeviceClass("temperature");
  kessel_temperatur.setUnitOfMeasurement("°C");
  kessel_rauchgas.setIcon("mdi:thermometer");
  kessel_rauchgas.setName("Kessel Rauchgas");
  kessel_rauchgas.setDeviceClass("temperature");
  kessel_rauchgas.setUnitOfMeasurement("°C");
  kessel_reinigung.setIcon("mdi:hand-wash-outline");
  kessel_reinigung.setName("Kessel Reinigung");
  kessel_rauchsauger.setIcon("mdi:smoke");
  kessel_rauchsauger.setName("Kessel Rauchsauger");
  kessel_zuendung.setIcon("mdi:fire");
  kessel_zuendung.setName("Kessel Zündung");
  kessel_pumpe.setIcon("mdi:pump");
  kessel_pumpe.setName("Kessel Pumpe");
  kessel_heizkreispumpe.setIcon("mdi:pump");
  kessel_heizkreispumpe.setName("Heizkreis Pumpe");
  kessel_boilerpumpe.setIcon("mdi:pump");
  kessel_boilerpumpe.setName("Boiler Pumpe");
  kessel_raumaustragung.setIcon("mdi:warehouse");
  kessel_raumaustragung.setName("Kessel Raumaustragung");
  // kessel_anforderung.setIcon("mdi:thermometer-plus");
  // kessel_anforderung.setName("Kessel Wärmeanforderung");
  kessel_stoerung.setIcon("mdi:exclamation-thick");
  kessel_stoerung.setName("Kessel Störung");
  kessel.setIcon("mdi:gas-burner");
  kessel.setName("Kessel Status");
  kessel_photodiode.setIcon("mdi:lightbulb");
  kessel_photodiode.setName("Kessel Photodiode");
  kessel_photodiode.setUnitOfMeasurement("%");
  kessel_geblaese.setIcon("mdi:fan");
  kessel_geblaese.setName("Kessel Gebläse");
  kessel_geblaese.setUnitOfMeasurement("rpm");
  kessel_saugzug.setIcon("mdi:fan");
  kessel_saugzug.setName("Kessel Saugzug");
  kessel_saugzug.setUnitOfMeasurement("rpm");
  kessel_energie.setIcon("mdi:counter");
  kessel_energie.setName("Kessel Energie");
  kessel_energie.setDeviceClass("energy");
  kessel_energie.setUnitOfMeasurement("kWh");
  kessel_brennerstunden.setIcon("mdi:clock-outline");
  kessel_brennerstunden.setName("Kessel Brennerstunden");
  kessel_brennerstunden.setDeviceClass("duration");
  kessel_brennerstunden.setUnitOfMeasurement("h");
  kessel_unterdruck.setIcon("mdi:gauge");
  kessel_unterdruck.setName("Kessel Unterdruck");
  kessel_unterdruck.setDeviceClass("pressure");
  kessel_unterdruck.setUnitOfMeasurement("mbar");
  kessel_drehrost.setIcon("mdi:grid");
  kessel_drehrost.setName("Kessel Drehrost");
  // .setIcon("mdi:thermometer");
  // .setName("");
  // .setDeviceClass("temperature");
  // .setUnitOfMeasurement("°C");

  mqtt.begin(MQTTSERVER, MQTTUSER, MQTTPASSWORD);

  lastUpdateCycleMillis = millis();
  timerd = millis();
}

// String lowerPrecision(double in){
//   char tmp[64];
//   sprintf(tmp, "%.1f", in);
//   return String(tmp);
// }

bool tempdiff(double a, double b, double diff) {
  return (abs(a - b) >= diff);
}

void debugLog(int value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog(double value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

// transforms a message to a byte string, so specific bits can be spotted, which are states of various components, e.g. 01100110 01...
// Byte 0 to X -> Bit 0 to X
void recordMsgAsBinaryString(unsigned char* source,int lengthInBytes, char* target) {
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

void readCTRLMSGFrame(unsigned char* anData, unsigned long currentMillis) {
  if (publishUnknown)
    recordMsgAsBinaryString(anData, 16, Kessel.ctrlMsg);      // 0 to 15

  Kessel.HeizkreisPumpe = getbit(anData, 1, 5);
  if (getbit(anData, 1, 7) && getbit(anData, 2, 0))            // 11 = close
    Kessel.Heizkreismischer = -1;
  else if (getbit(anData, 1, 7) && getbit(anData, 2, 0) == 0)  // 10 = open
    Kessel.Heizkreismischer = 1;
  else                                                        // 00 = off
    Kessel.Heizkreismischer = 0;
  if (getbit(anData, 2, 3) && getbit(anData, 2, 4))            // 11 = open
    Kessel.RLAVentil = 1;
  else if (getbit(anData, 2, 3) && getbit(anData, 2, 4) == 0)  // 10 = close
    Kessel.RLAVentil = -1;
  else                                                        // 00 = off
    Kessel.RLAVentil = 0;
  Kessel.BoilerPumpe = getbit(anData, 2, 5);
  Kessel.Stoerung1 = 1 - getbit(anData, 3, 0);
  Kessel.Drehrost = getbit(anData, 3, 6);
  Kessel.Reinigung = getbit(anData, 3, 7);
  if (getbit(anData, 4, 3) == 1)                              // left
    Kessel.DrehungSaugschlauch = -1;
  else if (getbit(anData, 4, 1) == 1)                         // right
    Kessel.DrehungSaugschlauch = 1;
  else                                                        // off
    Kessel.DrehungSaugschlauch = 0;
  Kessel.Rauchsauger = getbit(anData, 4, 5);
  Kessel.KesselPumpe = (getval2(anData, 8, 1, 1, 0) / 255) * 100;
  Kessel.Raumaustragung = getbit(anData, 9, 2) || getbit(anData, 9, 5);  // Schnecke || Saugturbine
  Kessel.Hauptantriebtakt = getval2(anData, 10, 2, 10, 0);
  Kessel.Hauptantrieb = getval2(anData, 12, 2, 10, 0);
  Kessel.Zuendung = getbit(anData, 16, 2);

  // Hauptantrieb Range:  0 .. Kessel.Hauptantriebtakt
  if (oKessel.Hauptantriebtakt != 0)
    Kessel.HauptantriebsZeit += (oKessel.Hauptantrieb * (currentMillis - timerHauptantrieb)) / oKessel.Hauptantriebtakt;

  timerHauptantrieb = currentMillis;
  oKessel.Hauptantrieb = Kessel.Hauptantrieb;
  oKessel.Hauptantriebtakt = Kessel.Hauptantriebtakt;

  // sum kwh
  double deltat = (currentMillis - kwhtimer) / (3600 * 1000.0);  // in h

  if (Kessel.Leistung > 1)
    Kessel.Brennerstunden += deltat;  // if burning
  Kessel.kwh += Kessel.Leistung * deltat;
  kwhtimer = currentMillis;

  // Raumaustragung = SchneckenBunker or Saugturbine
  if (Kessel.Raumaustragung != oKessel.Raumaustragung) {
    if (Kessel.Raumaustragung == 0) {  // switched off
      Kessel.Austragungslaufzeit = (currentMillis - austragungStartedAtMillis) / 1000;
      if (Kessel.Austragungslaufzeit > 800) Kessel.Austragungslaufzeit = 0;  // >800s ???
      Kessel.AustragungsGesamtLaufzeit += Kessel.Austragungslaufzeit;
    } else {  // switched on
      austragungStartedAtMillis = currentMillis;
    }
  }
}

void readSenseMSGFrame(unsigned char* anData, unsigned long currentMillis) {
  // States at byte 3 and 4 (maybe 0 to 5?)
  if (publishUnknown)
    recordMsgAsBinaryString(anData, 6, Kessel.senseMsg); // 0 to 5
  Kessel.Hauptantriebimpuls = getbit(anData, 3, 7);
  Kessel.ext = getbit(anData, 4, 7);
  // sensor values at the follwing bytes, 2 bytes per value
  Kessel.HK1_Vorlauf = getval2(anData, 6, 2, 0.1, 1);
  Kessel.Ruecklauf = getval2(anData, 8, 2, 0.1, 1);
  Kessel.Boiler = getval2(anData, 10, 2, 0.1, 1);
  Kessel.Kesseltemperatur = getval2(anData, 12, 2, 0.1, 1);
  Kessel.Puffer_unten = getval2(anData, 14, 2, 0.1, 1);
  Kessel.Puffer_oben = getval2(anData, 16, 2, 0.1, 1);
  Kessel.HK1_aussen = getval2(anData, 18, 2, 0.1, 1);
  Kessel.Rauchgastemperatur = getval2(anData, 20, 2, 0.1, 1);
  Kessel.Proztemperatur = getval2(anData, 22, 2, 0.1, 1);
  // see loop for 24 to 31
  Kessel.photo = getval2(anData, 32, 2, 0.1, 1);
  Kessel.Unterdruck = getval2(anData, 34, 2, 0.1, 1);
  // see loop for 36 to 68
  Kessel.Saugzug = getval2(anData, 69, 2, 0.6, 0);
  Kessel.Geblaese = getval2(anData, 71, 2, 0.6, 0);
  // see loop for 73 to 68
  // what about byte area starting from 89?

  for (int i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
    int s = i * 2 + 24;  // 24 to 30
    if (i >= 4)
      s = i * 2 + 28;  // 36 to 50
    else if (i >= 12)  // unknown area between 52 to 68
      s = i * 2 + 49;  // 73 to 87
    Kessel.Temp[i] = getval2(anData, s, 2, 0.1, 1);
  }

  // Range photo -221 to 127 - 348 numbers
  // x + 255 - offset to zero / range -> 0 to 1 * 100 -> range 0 to 100
  Kessel.photo = round(((Kessel.photo + 221.0) / 348) * 100);
  Kessel.photo = (Kessel.photo < 0) ? 0 : (Kessel.photo > 100) ? 100 : Kessel.photo;
  // old way
  // Kessel.photo = ((int) (Kessel.photo + 255.0) * 100) >> 9; // Result range 6 to 74

  // zwei gleiche impulse, die vom akt. unterschiedlich sind
  if ((Kessel.Hauptantriebimpuls == oKessel.Hauptantriebimpuls) && (Kessel.Hauptantriebimpuls != HauptantriebsImpuls)) {
    // Hauptantrieb läuft und produziert Impulse
    HauptantriebsImpuls = Kessel.Hauptantriebimpuls;
    Kessel.HauptantriebUmdrehungen++;  // vollst. Takte zählen
    oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
  }  // Impulsende

  oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
}

void publishFastChangingValues() {
  if (publishUnknown) {
    if (strcmp(Kessel.senseMsg, oKessel.senseMsg) != 0) {
      mqtt.publish("kwb/senseMsg", Kessel.senseMsg);
      memcpy(&(oKessel.senseMsg), &(Kessel.senseMsg), sizeof (Kessel.senseMsg));
    }
    if (strcmp(Kessel.ctrlMsg, oKessel.ctrlMsg) != 0) {
      mqtt.publish("kwb/ctrlMsg", Kessel.ctrlMsg);
      memcpy(&(oKessel.ctrlMsg), &(Kessel.ctrlMsg), sizeof (Kessel.ctrlMsg));
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
  }

  if (Kessel.BoilerPumpe != oKessel.BoilerPumpe) {
    kessel_boilerpumpe.setState(Kessel.BoilerPumpe);
    oKessel.BoilerPumpe = Kessel.BoilerPumpe;
  }

  if (Kessel.HeizkreisPumpe != oKessel.HeizkreisPumpe) {
    kessel_heizkreispumpe.setState(Kessel.HeizkreisPumpe);
    oKessel.HeizkreisPumpe = Kessel.HeizkreisPumpe;
  }

  if (Kessel.Heizkreismischer != oKessel.Heizkreismischer) {
    debugLog(Kessel.Heizkreismischer, "%d", "kwb/Heizkreismischer");
    oKessel.Heizkreismischer = Kessel.Heizkreismischer;
  }

  if (Kessel.DrehungSaugschlauch != oKessel.DrehungSaugschlauch) {
    debugLog(Kessel.DrehungSaugschlauch, "%d", "kwb/DrehungSaugschlauch");
    oKessel.DrehungSaugschlauch = Kessel.DrehungSaugschlauch;
  }

  if (Kessel.RLAVentil != oKessel.RLAVentil) {
    debugLog(Kessel.RLAVentil, "%d", "kwb/RLAVentil");
    oKessel.RLAVentil = Kessel.RLAVentil;
  }

  if (Kessel.Rauchsauger != oKessel.Rauchsauger) {
    kessel_rauchsauger.setState(Kessel.Rauchsauger);
    oKessel.Rauchsauger = Kessel.Rauchsauger;
  }

  if (Kessel.Drehrost != oKessel.Drehrost) {
    kessel_drehrost.setState(Kessel.Drehrost);
    oKessel.Drehrost = Kessel.Drehrost;
  }

  if (abs(Kessel.photo - oKessel.photo) >= 5) {
    kessel_photodiode.setValue((int)Kessel.photo);
    oKessel.photo = Kessel.photo;
  }

  if (Kessel.Raumaustragung != oKessel.Raumaustragung) {
    kessel_raumaustragung.setState(Kessel.Raumaustragung);
    oKessel.Raumaustragung = Kessel.Raumaustragung;
  }

  if (Kessel.Reinigung != oKessel.Reinigung) {
    kessel_reinigung.setState(Kessel.Reinigung);
    oKessel.Reinigung = Kessel.Reinigung;
  }

  if (Kessel.Zuendung != oKessel.Zuendung) {
    debugLog(Kessel.Zuendung, "%d", "kwb/zuendung");  // schaltet nie auf 1?
    kessel_zuendung.setState(Kessel.Zuendung);
    oKessel.Zuendung = Kessel.Zuendung;
  }
}

void publishSlowlyChangingValues(unsigned long currentMillis) {
  puffer_oben.setValue(float(Kessel.Puffer_oben));
  puffer_unten.setValue(float(Kessel.Puffer_unten));
  boiler.setValue(float(Kessel.Boiler));
  heizkreis_vorlauf.setValue(float(Kessel.HK1_Vorlauf));
  heizkreis_aussen.setValue(float(Kessel.HK1_aussen));
  kessel_ruecklauf.setValue(float(Kessel.Ruecklauf));
  kessel_temperatur.setValue(float(Kessel.Kesseltemperatur));
  kessel_rauchgas.setValue(float(Kessel.Rauchgastemperatur));
  kessel_pumpe.setState(Kessel.KesselPumpe == 100);
  kessel_geblaese.setValue(float(Kessel.Geblaese));
  kessel_saugzug.setValue(float(Kessel.Saugzug));

  int oldStat = oKessel.Kesselstatus;
  if (oldStat == 0) { // Off
    if(currentMillis <= 2 * 60 *1000 && (Kessel.photo >= 50 && Kessel.Rauchgastemperatur >= 70)) { // device just started - might be burning
      kessel.setValue("Brennt");
      Kessel.Kesselstatus = 2;
    } else if (Kessel.Geblaese > 300){
      kessel.setValue("Neustart");
      Kessel.Kesselstatus = 1;
    }
  } else if (oldStat == 1 && Kessel.Rauchgastemperatur > 75) { // Restarted & starts to burn
    kessel.setValue("Brennt");
    Kessel.Kesselstatus = 2;
  }
    // switches too often to afterrun, photo < 50 is already good, but geblase not - needs more/better conditions
  // } else if (oldStat == 2 && Kessel.photo < 50 && Kessel.Geblaese > 2200) { // Burned & expires
  //   kessel.setValue("Nachlauf");
  //   Kessel.Kesselstatus = 3;
  // }
  if (Kessel.photo < 20 && Kessel.Geblaese < 300) { // turn off and emergency escape
    kessel.setValue("Aus");
    Kessel.Kesselstatus = 0;
  }

  // debugLog(Kessel.ext, "%d", "kwb/anforderung");
  // kessel_anforderung.setState((((int)(Kessel.ext)) == 0) ? false : true);
  kessel_energie.setValue(float(Kessel.kwh));
  kessel_stoerung.setState(Kessel.Stoerung1);
  kessel_brennerstunden.setValue(float(Kessel.Brennerstunden));
  kessel_unterdruck.setValue(float(Kessel.Unterdruck));
}

void otherStuff(unsigned long currentMillis) {
  // kessel_proztemperatur.setValue(float(Kessel.Proztemperatur)); // HASensorNumber %.1f
  // kessel_HauptantriebUmdrehungen.setValue(Kessel.HauptantriebUmdrehungen); // HASensorNumber int
  // kessel_HauptantriebsZeit.setValue(Kessel.HauptantriebsZeit); // HASensorNumber int
  // kessel_hauptantriebtakt.setValue(float(Kessel.Hauptantriebtakt / 1000.0)); // HASensorNumber %2.1f
  // kessel_pellets.setValue((int) ((((double)Kessel.HauptantriebsZeit)*Hauptantriebsfakter) / 1000)); // HASensorNumber int
  // kessel_pelletsna.setValue((int) (((float)Kessel.AustragungsGesamtLaufzeit * Nebenantriebsfaktor))); // HASensorNumber int

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

    Kessel.Leistung = LEISTUNGKESSEL * TAKT100 * ((double)(Kessel.HauptantriebsZeit - ZD)) / ((double)(currentMillis - timerd));
    // kessel_leistung.setValue(float(Kessel.Leistung)); // HASensorNumber %2.1f

    // if (Kessel.Leistung < 1.0 )
    //   kessel_deltapelletsh.setValue(0); // HASensorNumber int

    // Verbrauch pro Stunde gemessen über NA
    // kessel_deltapelletnsh.setValue((int) ((float)(Kessel.AustragungsGesamtLaufzeit - AustragungsGesamtLaufzeit) * Nebenantriebsfaktor * 1000.0 * 3600.0 / ( currentMillis - timerd))); // HASensorNumber int

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
    double v;

    if ((Kessel.HauptantriebsZeit - HauptantriebsZeit) && (Kessel.AustragungsGesamtLaufzeit - NebenantriebsZeit)) {  // Wenn der Hauptantrieb lief
      v = (float)(Kessel.HauptantriebsZeit - HauptantriebsZeit) / ((float)(Kessel.AustragungsGesamtLaufzeit - NebenantriebsZeit) * 1000.0);
      // kessel_hana.setValue(float(v)); // HASensorNumber %f
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
  unsigned char anData[256];
  int nDataLen, nID, frameid, error;

  mqtt.loop();
  ArduinoOTA.handle();

  // Read RS485 dataframe
  int r = readframe(anData, nID, nDataLen, frameid, error);
  // if (!r)
  //   debugLog(nID, "Checksum error ID: %d", "kwb/error");

  unsigned long currentMillis = millis();

  if (nID == 33) {  // Control MSG  (from operating unit to boiler)
    readCTRLMSGFrame(anData, currentMillis);
  } else if (nID == 32) {  // Sense package
    readSenseMSGFrame(anData, currentMillis);
  }

  publishFastChangingValues();

  //////////////////// timed update block (e.g. each minute) ////////////////////
  if (currentMillis > (lastUpdateCycleMillis + updateEveryMinutes * 60 * 1000)) {

    bytecounter = 0;

    publishSlowlyChangingValues(currentMillis);
    otherStuff(currentMillis);

    memcpy(&oKessel, &Kessel, sizeof Kessel);
    lastUpdateCycleMillis = currentMillis;
    wifiReconnectIfLost(currentMillis);
  }

  // delay at the end of the loop to not trigger the SW Watchdog
  delay(5);
}
