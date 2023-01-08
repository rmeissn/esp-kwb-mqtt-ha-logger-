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

#ifndef WIFISSID
  #include "conf.h"
#endif

#define LEISTUNGKESSEL 22.0 // KW at 100 %
#define TAKT100  (12.5 / 5.0) // Taktung bei 100% Leistung 5s Laufzeit auf 12.5 sek

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
#define STATE_READ_MSG  2
#define MSG_TYPE_CTRL 1
#define MSG_TYPE_SENSE 2
#define FALSE 0
#define TRUE 1

// Globals

int updateEveryMinutes = 1, HAimp = 0;
long UD = 0, ZD = 0, SL = 0; // Schneckenlaufzeit
long NAz = 0, HAz = 0, HANAtimer = 0; // Timer zur Ausgabe der HANA Ratio

extern long bytecounter;
extern long framecounter;
extern long errorcounter;

//  gemessen 308gr auf 229 UD = 1.3449
//  Empirische aus 26KW Leistung 1.3338
//  24,7KW/4.8 mit 2338 UD -> 2.200
//  22KW * 0.8 * 9,4h  *  4kg /h  / 1642 UD =

// gemessen am HA
// 310gr 207UD = 1,495 g/UD > 3,58 g/s
// 200gr auf 135 UDs = 1.48 = 56s > 2,78
// 298gr auf 207 UDs = 1.44 = 86s > 3,46g/s

// macht ca. 2.4 UD /s  bei Norm Betrieb 187 UD in 3 Min
// nied Fallrohrstand 160UD auf 200gr = 1.25
// Nebenantrieb/Schnecke: 1990gr mit 373 s = 5.33gr/s

double NAfaktor = 5.4;
double HAfaktor = (400.0 / 128.0) ; // 400g in 120sek. > 3.333 g/s

unsigned long bytecount = 0;
unsigned long waitcount = 0;
unsigned long longwaitcount = 0;
unsigned long timerd = 0, lastUpdateCycleMillis = 0;
unsigned long timerschnecke = 0;
unsigned long timerHauptantrieb = 0 ;
unsigned long kwhtimer = 0; // Zeit seit letzer KW Messung

struct ef2 {
  double kwh = 0.1;
  double Rauchgastemperatur = 0.0;
  double Proztemperatur = 0.0;
  double Unterdruck = 0.0;
  double Brennerstunden = 0.0;
  double Kesseltemperatur = 0.0;
  double Geblaese = 0.0;
  double Leistung = 0.0;
  double Saugzug = 0.0;
  int Reinigung = 0;
  int Zuendung = 0;
  int Drehrost = 0;
  int Schneckenlaufzeit = 0;
  int Schneckengesamtlaufzeit = 0;
  int KeineStoerung = 0;
  int Raumaustragung = 0;
  int Hauptantriebimpuls = 0; // Impulszähler
  int Hauptantrieb = 0 ;      // HAMotor Laufzeit in Millisekunden
  int HauptantriebUD = 0; // Umdrehungen Stoker
  unsigned long Hauptantriebzeit = 1; // Gesamt HAzeit in millisekunden
  unsigned long Hauptantriebtakt = 1; // Taktzeit in millisekunden
  int Pumpepuffer = 0;
  int RLAVentil = 0;
  int ext = 1;
  double photo = 0.0;
  double Puffer_unten = 0.0;
  double Puffer_oben = 0.0;
  double HK1_aussen = -100.0;
  double HK1_Vorlauf = 0.0;
  double Ruecklauf = 0.0;
  double Boiler = 0.0;
  int Kesselstatus = 0; // 0 = Off, 1 = ignition, 2 = operation 3 = afterrun
  double Temp[20];
};

struct ef2 Kessel, oKessel; // akt. und "letzter" Kesselzustandd

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
HABinarySensor kessel_raumaustragung("kwb_kessel_raumaustragung");
HABinarySensor kessel_anforderung("kwb_kessel_anforderung");
HABinarySensor kessel_stoerung("kwb_kessel_stoerung");
HABinarySensor kessel_drehrost("kwb_kessel_drehrost");
HASensor kessel("kwb_kessel");
HASensorNumber kessel_photodiode("kwb_kessel_photodiode", HASensorNumber::PrecisionP0);
HASensorNumber kessel_geblaese("kwb_kessel_geblaese", HASensorNumber::PrecisionP0);
HASensorNumber kessel_saugzug("kwb_kessel_saugzug", HASensorNumber::PrecisionP0);
HASensorNumber kessel_energie("kwb_kessel_energie", HASensorNumber::PrecisionP3);
HASensorNumber kessel_brennerstunden("kwb_kessel_brennerstunden", HASensorNumber::PrecisionP3);
HASensorNumber kessel_unterdruck("kwb_kessel_unterdruck", HASensorNumber::PrecisionP1);

void wifi_on() {
  byte mac[6];

  WiFi.macAddress(mac);
  device.setUniqueId(mac, sizeof(mac));
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFISSID, WIFIPW);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
}

////////////////////// Setup //////////////////////////
void setup() {
  wifi_on();

  Serial.begin(19200);

  for (int r = 0; r < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); r++) {
    Kessel.Temp[r] = 0.0;
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
  kessel_zuendung.setIcon("mdi:fire");
  kessel_zuendung.setName("Kessel Zündung");
  kessel_pumpe.setIcon("mdi:pump");
  kessel_pumpe.setName("Kessel Pumpe");
  kessel_raumaustragung.setIcon("mdi:warehouse");
  kessel_raumaustragung.setName("Kessel Raumaustragung");
  kessel_anforderung.setIcon("mdi:thermometer-plus");
  kessel_anforderung.setName("Kessel Wärmeanforderung");
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

// bool tempdiff(double a, double b, double diff) {
//   return (abs(a - b) >= diff);
// }

void debugLog (int value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

void debugLog (double value, char* formatter, char* topic) {
  char msg[64];
  sprintf(msg, formatter, value);
  mqtt.publish(topic, msg);
}

//////////////////////////////////////////////////////////////////////////
///////////////////// Main Loop //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void loop() {
  mqtt.loop();
  unsigned char anData[256];
  int nDataLen;
  int nID;
  int i, r;
  unsigned long currentMillis = 0;
  int frameid, error;

  // Read RS485 dataframe
  r = readframe(anData, nID, nDataLen, frameid, error);
  // if (!r)
  //   debugLog(nID, "Checksum error ID: %d", "kwb/error");

  currentMillis = millis();

  // Control MSG  / Von Bediengerät an Kessel
  if (nID == 33) {
    Kessel.RLAVentil = getbit(anData, 2, 3);
    Kessel.Pumpepuffer = getbit(anData, 2, 7);
    Kessel.KeineStoerung = getbit(anData, 3, 0);
    Kessel.Drehrost = getbit(anData, 3, 6);
    Kessel.Reinigung = getbit(anData, 3, 7);
    Kessel.Raumaustragung = getbit(anData, 9, 2);
    Kessel.Hauptantriebtakt = getval2(anData, 10, 2, 10, 0);
    Kessel.Hauptantrieb = getval2(anData, 12, 2, 10, 0);
    Kessel.Zuendung = getbit(anData, 16, 2);

    // Hauptantrieb Range:  0 .. Kessel.Hauptantriebtakt
    if (oKessel.Hauptantriebtakt != 0 )
      Kessel.Hauptantriebzeit += (oKessel.Hauptantrieb * (currentMillis - timerHauptantrieb)) / (oKessel.Hauptantriebtakt);

    timerHauptantrieb = currentMillis;
    oKessel.Hauptantrieb = Kessel.Hauptantrieb;
    oKessel.Hauptantriebtakt = Kessel.Hauptantriebtakt;

    // sum kwh
    double deltat = (currentMillis - kwhtimer) / (3600.0 * 1000.0); // in h

    if (Kessel.Leistung > 1)
      Kessel.Brennerstunden += deltat; // Wenn der Kessel läuft
    Kessel.kwh += Kessel.Leistung * deltat;
    kwhtimer = currentMillis;

    // Raumaustragung = SchneckeBunker
    if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 1)) {
      Kessel.Schneckenlaufzeit = (currentMillis - timerschnecke) / 1000;
      if (Kessel.Schneckenlaufzeit > 800) Kessel.Schneckenlaufzeit = 0;
      Kessel.Schneckengesamtlaufzeit += Kessel.Schneckenlaufzeit;
    }
    if ((Kessel.Raumaustragung == 1) && (oKessel.Raumaustragung == 0))
      timerschnecke = currentMillis;
    if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 0))
      Kessel.Schneckenlaufzeit = 0;
  }

  // Sense package
  if (nID == 32) {
    Kessel.Hauptantriebimpuls = getbit(anData, 3, 7);
    Kessel.ext = getbit(anData, 4, 7);
    Kessel.HK1_Vorlauf = getval2(anData, 6, 2, 0.1, 1);
    Kessel.Ruecklauf = getval2(anData, 8, 2, 0.1, 1);
    Kessel.Boiler = getval2(anData, 10, 2, 0.1, 1);
    Kessel.Kesseltemperatur = getval2(anData, 12, 2, 0.1, 1);
    Kessel.Puffer_unten = getval2(anData, 14, 2, 0.1, 1);
    Kessel.Puffer_oben = getval2(anData, 16, 2, 0.1, 1);
    Kessel.HK1_aussen = getval2(anData, 18, 2, 0.1, 1);
    Kessel.Rauchgastemperatur = getval2(anData, 20, 2, 0.1, 1);
    Kessel.Proztemperatur = getval2(anData, 22, 2, 0.1, 1);
    Kessel.Temp[0] = getval2(anData, 24, 2, 0.1, 1);
    Kessel.Temp[1] = getval2(anData, 26, 2, 0.1, 1);
    Kessel.Temp[2] = getval2(anData, 28, 2, 0.1, 1);
    Kessel.Temp[3] = getval2(anData, 30, 2, 0.1, 1);
    Kessel.photo =  getval2(anData, 32, 2, 0.1, 1);
    Kessel.Unterdruck = getval2(anData, 34, 2, 0.1, 1);
    Kessel.Temp[4] = getval2(anData, 36, 2, 0.1, 1);
    Kessel.Temp[5] = getval2(anData, 38, 2, 0.1, 1);
    Kessel.Temp[6] = getval2(anData, 40, 2, 0.1, 1);
    Kessel.Temp[7] = getval2(anData, 42, 2, 0.1, 1);
    Kessel.Temp[8] = getval2(anData, 44, 2, 0.1, 1);
    Kessel.Temp[9] = getval2(anData, 46, 2, 0.1, 1);
    Kessel.Temp[10] = getval2(anData, 48, 2, 0.1, 1);
    Kessel.Temp[11] = getval2(anData, 50, 2, 0.1, 1);
    //unbekanntes Bitfeld?
    Kessel.Saugzug = getval2(anData, 69, 2, 0.6, 0);
    Kessel.Geblaese = getval2(anData, 71, 2, 0.6, 0);
    Kessel.Temp[12] = getval2(anData, 73, 2, 0.1, 1);
    Kessel.Temp[13] = getval2(anData, 75, 2, 0.1, 1);
    Kessel.Temp[14] = getval2(anData, 77, 2, 0.1, 1);
    Kessel.Temp[15] = getval2(anData, 79, 2, 0.1, 1);
    Kessel.Temp[16] = getval2(anData, 81, 2, 0.1, 1);
    Kessel.Temp[17] = getval2(anData, 83, 2, 0.1, 1);
    Kessel.Temp[18] = getval2(anData, 85, 2, 0.1, 1);
    Kessel.Temp[19] = getval2(anData, 87, 2, 0.1, 1);
    //Bitfeld 89?

    // Range photo -221 to 127 - 348 numbers
    // x + 255 - offset to zero / range -> 0 to 1 * 100 -> range 0 to 100
    Kessel.photo = round(((Kessel.photo + 221.0) / 348) * 100);
    Kessel.photo = (Kessel.photo < 0) ? 0 : (Kessel.photo > 100) ? 100 : Kessel.photo;
    // old way
    // Kessel.photo = ((int) (Kessel.photo + 255.0) * 100) >> 9; // Result range 6 to 74

    // zwei gleiche impulse, die vom akt. unterschiedlich sind
    if ((Kessel.Hauptantriebimpuls == oKessel.Hauptantriebimpuls) && (Kessel.Hauptantriebimpuls != HAimp )) {
      // Hauptantrieb läuft und produziert Impulse
      HAimp = Kessel.Hauptantriebimpuls;
      Kessel.HauptantriebUD++; // vollst. Takte zählen
      oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
    } // Impulsende

    oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
  }

  // publish some values directly, as they change
  if (Kessel.Drehrost != oKessel.Drehrost) {
    kessel_drehrost.setState((Kessel.Drehrost == 0) ? false : true);
    oKessel.Drehrost = Kessel.Drehrost;
  }

  if(abs(Kessel.photo - oKessel.photo) >= 5) {
    kessel_photodiode.setValue((int) Kessel.photo);
    oKessel.photo = Kessel.photo;
  }

  // Wenn die Schnecke stehen geblieben ist und vorher lief Schneckenaufzeitausgeben
  if  (Kessel.Raumaustragung != oKessel.Raumaustragung) {
    kessel_raumaustragung.setState((((int)(Kessel.Raumaustragung)) == 0) ? false : true);
    if (Kessel.Raumaustragung == 0) { // live reporting Schneckenlaufzeitausgabe
      // kessel_schneckenlaufzeit.setValue(Kessel.Schneckenlaufzeit); // HASensorNumber int
      oKessel.Schneckenlaufzeit = Kessel.Schneckenlaufzeit;
    }
    oKessel.Raumaustragung = Kessel.Raumaustragung;
  }

  if (Kessel.Reinigung != oKessel.Reinigung) {
    kessel_reinigung.setState((((int)(Kessel.Reinigung)) == 0) ? false : true);
    oKessel.Reinigung = Kessel.Reinigung;
  }

  if (Kessel.Zuendung != oKessel.Zuendung)   {
    kessel_zuendung.setState((((int)(Kessel.Zuendung)) == 0) ? false : true);
    oKessel.Zuendung = Kessel.Zuendung;
  }

  // for (i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
  //  if (tempdiff(Kessel.Temp[i], oKessel.Temp[i], 0.4)) {
  //    debugLog(Kessel.Temp[i], "%d", "kwb/temp" + i);
  //    oKessel.Temp[i] = Kessel.Temp[i];
  //  }
  // }

  //////////////////// timed update block (e.g. each minute) /////////////////////////////
  if (currentMillis > (lastUpdateCycleMillis + updateEveryMinutes * 60 * 1000)) {

    bytecounter = 0;

    puffer_oben.setValue(float(Kessel.Puffer_oben));
    puffer_unten.setValue(float(Kessel.Puffer_unten));
    boiler.setValue(float(Kessel.Boiler));
    heizkreis_vorlauf.setValue(float(Kessel.HK1_Vorlauf));
    heizkreis_aussen.setValue(float(Kessel.HK1_aussen));
    kessel_ruecklauf.setValue(float(Kessel.Ruecklauf));
    kessel_temperatur.setValue(float(Kessel.Kesseltemperatur));
    kessel_rauchgas.setValue(float(Kessel.Rauchgastemperatur));
    kessel_reinigung.setState((((int)(Kessel.Reinigung)) == 0) ? false : true);
    kessel_zuendung.setState((((int)(Kessel.Zuendung)) == 0) ? false : true);
    kessel_pumpe.setState((((int)(Kessel.Pumpepuffer)) == 0) ? false : true);
    kessel_raumaustragung.setState((((int)(Kessel.Raumaustragung)) == 0) ? false : true);
    kessel_photodiode.setValue((int) Kessel.photo);
    kessel_geblaese.setValue(float(Kessel.Geblaese));
    kessel_saugzug.setValue(float(Kessel.Saugzug));

    int oldStat = oKessel.Kesselstatus;
    if(Kessel.photo < 20 && Kessel.Geblaese < 300 ) {
      kessel.setValue("Aus");
      Kessel.Kesselstatus = 0;
    } else if(Kessel.photo >= 20 && Kessel.photo < 60 && Kessel.Geblaese > 2200 ) {
      if(oldStat == 0) {
        kessel.setValue("Neustart");
        Kessel.Kesselstatus = 1;
      } else if(oldStat == 2) {
        kessel.setValue("Nachlauf");
        Kessel.Kesselstatus = 3;
      }
    } else if(Kessel.photo >= 60 && Kessel.Geblaese >= 300 && Kessel.Geblaese <= 2200 ) {
      // enable to jump to 2 if logger was just started and bioler is currently burning
      if((Kessel.Kesselstatus == 0  && oldStat == 0) || (oldStat == 1 || oldStat == 3)){
        kessel.setValue("Brennt");
        Kessel.Kesselstatus = 2;
      }
    }

    kessel_anforderung.setState((((int)(Kessel.ext)) == 0) ? false : true);
    kessel_energie.setValue(float(Kessel.kwh));
    kessel_stoerung.setState((1 - ((int)(Kessel.KeineStoerung)) == 0) ? false : true);
    kessel_brennerstunden.setValue(float(Kessel.Brennerstunden));
    kessel_unterdruck.setValue(float(Kessel.Unterdruck));

    if (Kessel.Raumaustragung == 0) { // live reporting Schneckenlaufzeitausgabe
      // kessel_schneckenlaufzeit.setValue(Kessel.Schneckenlaufzeit); // HASensorNumber int
      oKessel.Schneckenlaufzeit = Kessel.Schneckenlaufzeit;
    }

    // kessel_proztemperatur.setValue(float(Kessel.Proztemperatur)); // HASensorNumber %.1f

    // for (i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++) {
    //   debugLog(Kessel.Temp[i], "%d", "kwb/temp" + i);
    // }

    // kessel_hauptantriebud.setValue(Kessel.HauptantriebUD); // HASensorNumber int
    // kessel_hauptantriebzeit.setValue(Kessel.Hauptantriebzeit); // HASensorNumber int
    // kessel_hauptantriebtakt.setValue(float(Kessel.Hauptantriebtakt / 1000.0)); // HASensorNumber %2.1f
    // kessel_pellets.setValue((int) ((((double)Kessel.Hauptantriebzeit)*HAfaktor) / 1000)); // HASensorNumber int
    // kessel_pelletsna.setValue((int) (((float)Kessel.Schneckengesamtlaufzeit * NAfaktor))); // HASensorNumber int

    framecounter = 0;
    errorcounter = 0;

    // debugLog(framecounter, "%d", "kwb/frames");
    // debugLog(errorcounter, "%d", "kwb/errors");

    // akt Verbrauch berechnen
    if (Kessel.HauptantriebUD - UD) {
      int d, p;

      d = (Kessel.HauptantriebUD - UD) * 3600 * 1000 / (currentMillis - timerd);
      // Besp   3.58 * 60 * 60    1000ms / 5000ms
      p = (int) (HAfaktor * 60 * 60 * ( Kessel.Hauptantriebzeit - ZD) ) / (currentMillis - timerd) ;
      // kessel_deltapelletsh.setValue(p); // HASensorNumber int

      Kessel.Leistung = LEISTUNGKESSEL * TAKT100 * ((double) ( Kessel.Hauptantriebzeit - ZD)) / ((double) (currentMillis - timerd)) ;
      // kessel_leistung.setValue(float(Kessel.Leistung)); // HASensorNumber %2.1f

      // if (Kessel.Leistung < 1.0 )
      //   kessel_deltapelletsh.setValue(0); // HASensorNumber int

      // Verbrauch pro Stunde gemessen über NA
      // kessel_deltapelletnsh.setValue((int) ((float)(Kessel.Schneckengesamtlaufzeit - SL) * NAfaktor * 1000.0 * 3600.0 / ( currentMillis - timerd))); // HASensorNumber int

      SL = Kessel.Schneckengesamtlaufzeit;
      UD = Kessel.HauptantriebUD;
      // kessel_deltat.setValue((millis() - timerd) / 1000); // HASensorNumber int
      ZD = Kessel.Hauptantriebzeit;
      timerd = currentMillis;
    }

    //////////////////////////////////////////////
    // Berechnung HA/NA Verhältnis
    // Alle xx Min  berechnen (-> ca. 1.9 wenn der sinkt gibt es Förderprobleme)
    if (currentMillis > ( HANAtimer + 30 * 60  * 1000)) {
      HANAtimer = currentMillis;
      double v;

      if ((Kessel.Hauptantriebzeit - HAz) && (Kessel.Schneckengesamtlaufzeit - NAz)) { // Wenn der HA lief
        v = (float) (Kessel.Hauptantriebzeit - HAz) / ((float)(Kessel.Schneckengesamtlaufzeit - NAz) * 1000.0  ) ;
        // kessel_hana.setValue(float(v)); // HASensorNumber %f
        NAz = Kessel.Schneckengesamtlaufzeit;
        HAz = Kessel.Hauptantriebzeit;
      }
    }

    oKessel.Schneckengesamtlaufzeit = Kessel.Schneckengesamtlaufzeit;
    oKessel.Leistung = Kessel.Leistung;

    memcpy(&oKessel, &Kessel, sizeof Kessel);
    lastUpdateCycleMillis = currentMillis;
  }

  // delay at the end of the loop to not trigger the SW Watchdog
  delay(5);
}
