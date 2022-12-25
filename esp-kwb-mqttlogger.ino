
// Author windundsternne V.0.99
// Das Programm liest Control und Sense Daten vom Bus
// fhem. Updates können OTA durchgeführt werden.
// Ist bestimmt nicht fehlerfrei läuft aber seit einer Weile stabil
// Hardware
// Webmos d1 mini Pro + HER MAX485 module
// Stromversorgung über Kessel 24V 7800 Spannungsregler
// Neben den Kesselwerten werden nich Brennerlaufzeit Gesamtenergie
// berechnet mit der der Pelletverbrauch bestimmt werden kann (ca. 4.3kg/kwh bei einem EF2)


// Serial:
// Serielle Schnittstelle
// Die Pins TX und RX wie beim Arduino UNO. stattdessen die Pins D7 und D8 (= GPIO 13 / RXD2 bzw. GPIO 15 / TXD2)
// benutzen. Dazu  im Setup die Anweisung Serial.swap() nach Serial.begin() einfügen.



// Individualisierungen

//#define STASSID "MYSSID"          // Wlan SSID
//#define STAPSK  "MYWLANPW"        // Wlan PW
//#define MQTTSERVER "192.168.0.5"  // IP MQTT-Server

//  individuelle Konfig ausserhalb des GIT
#ifndef STASSID
#include "conf.h"
#endif

// Define GPIOPIN 2 wenn SW Serial genutzt werden soll
// Else undef -> Default UART0 via GPIO13 als RX nutzen

//#define SWSERIAL 2

#define RX        2    // GPIO2   //Serial Receive pin
// Aktuell ist der nicht angeschlossen, da  nicht
// gesteuert werden soll
#define TX        4    // GPIO4   //Serial Transmit pin
#define RTS_pin   5 // GPIO5  //RS485 Direction control
#define RS485Transmit    HIGH
#define RS485Receive     LOW



#define OTAHOST "kwbeasyfire"     // unnter dem Namen  Netzwerkschnittstelle in der ArduionIDE
#define INTOPIC "cmd"
#define MQNAME "kwb"

#define LEISTUNGKESSEL 25.0 // KW bei 100 %
#define TAKT100  (12.5 / 5.0)            // Taktung bei 100% Leistung 5s Laufzeit auf 12.5 sek 

// Ende Individualisierungen


#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <stdio.h>
#include <time.h>

#define STATE_WAIT_FOR_HEADER 1
#define STATE_READ_MSG  2
#define MSG_TYPE_CTRL 1
#define MSG_TYPE_SENSE 2
#define FALSE 0
#define TRUE 1



const char* mqtt_server = MQTTSERVER; // IP des MQTT Servers - z.B. fhem
const char* mqtt_user = MQTTUSER;
const char* mqtt_password = MQTTPASSWORD;
const char* ssid = STASSID;
const char* password = STAPSK;

// Globals

long UD = 0;
long SL = 0 ; // Schneckenlaufzeit
long ZD = 0;
int updatemin = 5;
long UDtimer = 0;
long HANAtimer = 0; // Timer zur Ausgabe der HANA Ratio
long NAz = 0, HAz = 0;

int wifistatus = 0;

extern long bytecounter;
extern long framecounter;
extern long errorcounter;
long last_errorcount = 0;

int HAimp = 0;

//  gemessen 308gr auf 229 UD = 1.3449
//  Empirische aus 26KW Leistung 1.3338
//  24,7KW/4.8 mit 2338 UD -> 2.200
//  22KW * 0.8 * 9,4h  *  4kg /h  / 1642 UD =


// gemessen am HA
// 310gr 207UD = 1,495 g/UD > 3,58 g/s
// 200gr auf 135 UDs = 1.48 = 56s > 2,78
// 298gr auf 207 UDs = 1.44 = 86s > 3,46g/s


// mach ca. 2.4 UD /s  bei Norm Betrieb 187 UD in 3 Min
// nied Fallrohrstand 160UD auf 200gr = 1.25
// Nebenantrieb/Schnecke: 1990gr mit 373 s = 5.33gr/s


double NAfaktor = 5.4;
double HAfaktor = (400.0 / 128.0) ; // // 400g in 120sek. > 3.333 g/s

unsigned long count = 0;
unsigned long bytecount = 0;
unsigned long waitcount = 0;
unsigned long longwaitcount = 0;
unsigned long keeptime = 0, timerd = 0, timer1 = 0, timer2 = 0, timer3 = 0, timeru = 0 , timerboot = 0, timerimpuls = 0, timerha = 0;
unsigned long timerprell = 0, timerhazeit = 0;
unsigned long timerschnecke = 0;
unsigned long timerpause = 0;
unsigned long timerHA = 0 ;
unsigned long kwhtimer = 0; // Zeit seit letzer KW Messung

struct ef2
{
  double kwh = 0.1;
  double Rauchgastemperatur = 0.0;
  double Proztemperatur = 0.0;
  double Unterdruck = 0.0;
  double Brennerstunden = 0.0;
  double Kesseltemperatur = 0.0;
  double Geblaese = 0.0;
  double Leistung = 0.0; 
  double Saugzug = 0.0;
  int  Reinigung = 0;
  int Zuendung = 0;
  int Drehrost = 0;
  int Schneckenlaufzeit = 0;
  int Schneckengesamtlaufzeit = 0;
  int KeineStoerung = 0;
  int Raumaustragung = 0;
  int Hauptantriebimpuls = 0; // Impulszähler
  int Hauptantrieb = 0 ;      // HAMotor Laufzeit in Millisekunden
  int HauptantriebUD = 0; // Umdrehungen Stoker
  unsigned long  Hauptantriebzeit = 1; // Gesamt HAzeit in millisekunden
  unsigned long  Hauptantriebtakt = 1; // Taktzeit in millisekunden
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
  double Temp[20];
};

struct ef2 Kessel, oKessel; // akt. und "letzter" Kesselzustandd

WiFiClient espClient;
PubSubClient client(espClient);

#ifdef SWSERIAL
#include <SoftwareSerial.h>  // https://github.com/PaulStoffregen/SoftwareSerial
SoftwareSerial RS485Serial(RX, TX);
#endif

#include "espinclude.h"

//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// Wifi einschalten,, Verbinden und mqtt Connecten
// OTA einrichten
void wifi_on()
{


  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    // Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTAHOST);

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    //Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    // Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    //Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      // Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      // Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      // Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      // Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      // Serial.println("End Failed");
    }
  });

  client.setServer(mqtt_server, 1883);
  wifistatus = 1;
}


// WIFi Abschalten
void wifi_off()
{
  mqttreconnect();
  client.publish("kwb/Info", "WIFI Off");
  delay(1999);
  WiFi.mode(WIFI_OFF);
  wifistatus = 0;
}

////////////////////// Setup //////////////////////////
void setup() {
  char msg[64];
  char rec[10];

  wifi_on();

  ArduinoOTA.begin();
  mqttreconnect();
  client.setKeepAlive(5);
  ArduinoOTA.handle();

#ifdef SWSERIAL
  // RS485
  pinMode(RTS_pin, OUTPUT);
  // RS485
  pinMode(RX, INPUT);
  // Start the Modbus serial Port RS485
  RS485Serial.begin(19200);
  // RS485 Einlesen
  digitalWrite(RTS_pin, RS485Receive);      // Init Receive
  client.publish("kwb/info", "Booting (Software Serial)");
#else
  Serial.begin(19200);
  Serial.swap(); // RX auf Pin D7
  client.publish("kwb/info", "Booting (UART0 Serial)");
#endif

  sprintf(msg, "%d", updatemin );
  client.publish("kwb/updatemin", msg);

  sprintf(msg, "%.3f", 0);
  client.publish("kwb/kwh", msg);

  for (int r = 0; r < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); r++)
  {
    Kessel.Temp[r] = 0.0;
  }

  // 10 Werte ausgeben um zu schauen ob die ser. tut und beim MQ alles ankommt...
  for (int r = 0; r < 10; r++)
  {
    rec[r] = readbyte();
  }

  int r = 0;
  sprintf(msg, "bytes read RS485: %d %d %d %d %d %d %d %d %d %d", r, rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++], rec[r++] );
  client.publish("kwb/rec", msg);

  timerboot = millis();
  timer1 = millis();
  timeru = millis();
  keeptime = millis();
  timerd = millis();

  // Impulsstat
  timerpause = millis();
}

//////////////////////////////////////////////////////////////////////////
///////////////////// L O O P ////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
void loop() {
  unsigned char anData[256];
  int nDataLen;
  int byte;
  char msg[256];
  int nID;
  int i, r;
  int value;
  unsigned long  milli = 0;
  int frameid, error;

  // Datenframe vom RS485 einlesen
  r = readframe(anData, nID, nDataLen, frameid, error);
  if (!r)
  {
    sprintf(msg, "Checksum error ID: %d", nID);
    client.publish("kwb/error", msg);    
  }
  milli = millis();

  ///////////////////////////////////
  // Control MSG  / Von Bediengerät an Kessel
  if (nID == 33)
  {
    Kessel.RLAVentil = getbit(anData, 2, 3);
    Kessel.Pumpepuffer = getbit(anData, 2, 7);
    Kessel.KeineStoerung = getbit(anData, 3, 0);
    Kessel.Drehrost = getbit(anData, 3, 6);
    Kessel.Reinigung = getbit(anData, 3, 7);
    Kessel.Raumaustragung = getbit(anData, 9, 2);
    Kessel.Hauptantriebtakt = getval2(anData, 10, 2, 10, 0);
    Kessel.Hauptantrieb = getval2(anData, 12, 2, 10, 0);
    Kessel.Zuendung = getbit(anData, 16, 2);

    // kwh summieren
    double deltat = (milli - kwhtimer) / (3600.0 * 1000.0); // in h

    // Hauptantrieb Range:  0 .. Kessel.Hauptantriebtakt
    if (oKessel.Hauptantriebtakt != 0 )
    {
      Kessel.Hauptantriebzeit += (oKessel.Hauptantrieb * (milli - timerHA)) / (oKessel.Hauptantriebtakt);
    }

    timerHA = milli;
    oKessel.Hauptantrieb = Kessel.Hauptantrieb;
    oKessel.Hauptantriebtakt = Kessel.Hauptantriebtakt;

    if (Kessel.Leistung > 1) {
      Kessel.Brennerstunden += deltat; // Wenn der Kessel läuft
    }
    Kessel.kwh += Kessel.Leistung * deltat;
    kwhtimer = milli;
  }

  ///////////////////////////
  // Sense Paket empfangen
  if (nID == 32)
  {
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

    // zwei gleiche impulse, die vom akt. unterschiedlich sind
    if ((Kessel.Hauptantriebimpuls == oKessel.Hauptantriebimpuls)
        && (Kessel.Hauptantriebimpuls != HAimp ))
    { // Hauptantrieb läuft und produziert Impulse
      HAimp = Kessel.Hauptantriebimpuls;
      Kessel.HauptantriebUD++; // vollst. Takte zählen
      last_errorcount = errorcounter;
      timerimpuls = milli;
      oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
    } // Impulsende

    oKessel.Hauptantriebimpuls = Kessel.Hauptantriebimpuls;
  } // Ende Sense Paket auslesen

  // Raumaustragung = SchneckeBunker
  if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 1))
  {
    Kessel.Schneckenlaufzeit = (milli - timerschnecke) / 1000;
    if (Kessel.Schneckenlaufzeit > 800) Kessel.Schneckenlaufzeit = 0;
    Kessel.Schneckengesamtlaufzeit += Kessel.Schneckenlaufzeit;
  }
  if ((Kessel.Raumaustragung == 1) && (oKessel.Raumaustragung == 0))
  {
    timerschnecke = milli;
  }
  if ((Kessel.Raumaustragung == 0) && (oKessel.Raumaustragung == 0))
  {
    Kessel.Schneckenlaufzeit = 0;
  }
  //oKessel.Raumaustragung = Kessel.Raumaustragung;

  if (nID != 33 && nID != 32)
  {
    sprintf(msg, "Unknown Package: %d", nID);
    client.publish("kwb/error", msg);
  } 

  // Wichtig!!!
  // alle x Sekunden ## Update, damit OTA und Ping tun
  if (milli > (timeru + 2 * 1000))
  {
    timeru = milli;
    ArduinoOTA.handle(); // OTA nur wenn Kessel nicht brennt
    client.loop(); // Zeit für den Callback+MQTT Ping
  }

  // manche Werte direkt ausgeben,  wenn eine Änderung da ist
  // Wenn die Schnecke stehen geblieben ist und vorher lief Schneckenaufzeitausgeben
  if  (Kessel.Raumaustragung != oKessel.Raumaustragung)
  {
    if (Kessel.Raumaustragung == 0)
    {
      // live reporting Schneckenlaufzeitausgabe 
      mqttreconnect();
      sprintf(msg, "%d", Kessel.Schneckenlaufzeit);
      client.publish("kwb/Schneckenlaufzeit", msg);
      oKessel.Schneckenlaufzeit = Kessel.Schneckenlaufzeit;
    }
    oKessel.Raumaustragung = Kessel.Raumaustragung;
  }

  if (Kessel.Reinigung != oKessel.Reinigung)
  {
    mqttreconnect();
    sprintf(msg, "%d", Kessel.Reinigung);
    client.publish("kwb/Reinigung", msg);
    oKessel.Reinigung = Kessel.Reinigung;
  }

  if (Kessel.Zuendung != oKessel.Zuendung)
  {
    mqttreconnect();
    sprintf(msg, "%d", Kessel.Zuendung);
    client.publish("kwb/Zuendung", msg);
    oKessel.Zuendung = Kessel.Zuendung;
  }

  if (tempdiff(Kessel.HK1_Vorlauf, oKessel.HK1_Vorlauf, 0.4))
  {
    send_value("kwb/HK1_Vorlauf", Kessel.HK1_Vorlauf);
    oKessel.HK1_Vorlauf = Kessel.HK1_Vorlauf;
  }
  
  if (tempdiff(Kessel.Ruecklauf, oKessel.Ruecklauf, 0.4))
  {
    send_value("kwb/Ruecklauf", Kessel.Ruecklauf);
    oKessel.Ruecklauf = Kessel.Ruecklauf;
  }
  
  if (tempdiff(Kessel.Boiler, oKessel.Boiler, 0.4))
  {
    send_value("kwb/Boiler", Kessel.Boiler);
    oKessel.Boiler = Kessel.Boiler;
  }

  if (tempdiff(Kessel.Proztemperatur, oKessel.Proztemperatur, 0.4))
  {
    send_value("kwb/Proztemperatur", Kessel.Proztemperatur);
    oKessel.Proztemperatur = Kessel.Proztemperatur;
  }

  if (tempdiff(Kessel.Puffer_unten, oKessel.Puffer_unten, 0.4))
  {
    send_value("kwb/Puffer_unten", Kessel.Puffer_unten);
    oKessel.Puffer_unten = Kessel.Puffer_unten;
  }

  if (tempdiff(Kessel.Puffer_oben, oKessel.Puffer_oben, 0.4))
  {
    send_value("kwb/Puffer_oben", Kessel.Puffer_oben);
    oKessel.Puffer_oben = Kessel.Puffer_oben;
  }

  if (tempdiff(Kessel.HK1_aussen, oKessel.HK1_aussen, 0.4))
  {
    send_value("kwb/HK1_aussen", Kessel.HK1_aussen);
    oKessel.HK1_aussen = Kessel.HK1_aussen;
  }

  for (i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++)
  {
    if (tempdiff(Kessel.Temp[i], oKessel.Temp[i], 0.4))
    {
      sprintf(msg, "kwb/Temp%d", i);
      send_value(msg, Kessel.Temp[i]);
      oKessel.Temp[i] = Kessel.Temp[i];
    }
  }

  //////////////////// 5 min Block /////////////////////////////
  // Alle anderen Änderungen werden erst ausgegeben wenn
  // timer1 abgelaufen
  // wenn Änderung alle x*60 Sekunden Ausgabe an MQTT
  if (milli > (timer1 + updatemin * 60 * 1000))
  {
    mqttreconnect();
    
    bytecounter = 0;

    if (Kessel.Raumaustragung == 0)
    {
      // live reporting Schneckenlaufzeitausgabe 
      sprintf(msg, "%d", Kessel.Schneckenlaufzeit);
      client.publish("kwb/Schneckenlaufzeit", msg);
      oKessel.Schneckenlaufzeit = Kessel.Schneckenlaufzeit;
    }
  
    sprintf(msg, "%d", Kessel.Reinigung);
    client.publish("kwb/Reinigung", msg);
  
    sprintf(msg, "%d", Kessel.Zuendung);
    client.publish("kwb/Zuendung", msg);
  
    send_value("kwb/HK1_Vorlauf", Kessel.HK1_Vorlauf);
    
    send_value("kwb/Ruecklauf", Kessel.Ruecklauf);
    
    send_value("kwb/Boiler", Kessel.Boiler);
  
    send_value("kwb/Proztemperatur", Kessel.Proztemperatur);
  
    send_value("kwb/Puffer_unten", Kessel.Puffer_unten);
  
    send_value("kwb/Puffer_oben", Kessel.Puffer_oben);
  
    send_value("kwb/HK1_aussen", Kessel.HK1_aussen);
  
    for (i = 0; i < (sizeof(Kessel.Temp) / sizeof(Kessel.Temp[0])); i++)
    {
      sprintf(msg, "kwb/Temp%d", i);
      send_value(msg, Kessel.Temp[i]);
    }

    sprintf(msg, "%d / %d", framecounter, errorcounter);
    client.publish("kwb/frames/errors", msg);
    framecounter = 0;
    errorcounter = 0;

    sprintf(msg, "%d", Kessel.HauptantriebUD);
    client.publish("kwb/HauptantriebUD", msg);

    sprintf(msg, "%d", Kessel.Hauptantriebzeit / 1000);
    client.publish("kwb/Hauptantriebzeit", msg);

    sprintf(msg, "%2.1f", (double) Kessel.Hauptantriebtakt / 1000.0);
    client.publish("kwb/Hauptantriebtakt", msg);

    sprintf(msg, "%d", (int)( (((double)Kessel.Hauptantriebzeit)*HAfaktor) / 1000));
    client.publish("kwb/Pellets", msg);

    // Messung Über Schneckenantrieb
    sprintf(msg, "%d", (int)(((float)Kessel.Schneckengesamtlaufzeit * NAfaktor)  ));
    client.publish("kwb/PelletsNA", msg);

    // akt Verbrauch berechnen
    if (Kessel.HauptantriebUD - UD)
    {
      int d, p;
      d = (Kessel.HauptantriebUD - UD) * 3600 * 1000 / (milli - timerd);

      // Besp   3.58 * 60 * 60    1000ms / 5000ms
      p = (int) (HAfaktor * 60 * 60 * ( Kessel.Hauptantriebzeit - ZD) ) / (milli - timerd) ;
      sprintf(msg, "%d", p);
      client.publish("kwb/deltaPelletsh", msg);

      Kessel.Leistung = LEISTUNGKESSEL * TAKT100 * ((double) ( Kessel.Hauptantriebzeit - ZD)) / ((double) (milli - timerd)) ;
      sprintf(msg, "%2.1f", Kessel.Leistung);
      client.publish("kwb/Leistung", msg);
      
      if (Kessel.Leistung < 1.0 )
      {
        client.publish("kwb/deltaPelletsh", "0");
      }
      
      // Verbrauch pro Stunde gemessen über NA
      sprintf(msg, "%d", (int) ((float)(Kessel.Schneckengesamtlaufzeit - SL) * NAfaktor * 1000.0 * 3600.0 / ( milli - timerd)));
      client.publish("kwb/deltaPelletsNAh", msg);

      SL = Kessel.Schneckengesamtlaufzeit;

      UD = Kessel.HauptantriebUD;
      //sprintf(msg, "%d", (millis() - timerd) / 1000);
      //client.publish("kwb/deltat", msg);
      ZD = Kessel.Hauptantriebzeit;
      timerd = milli;
    }

    //////////////////////////////////////////////
    // Berechnung HA/NA Verhältnis
    // Alle xx Min  berechnen (-> ca. 1.9 wenn der sinkt gibt es Förderprobleme)
    if (milli > ( HANAtimer + 30 * 60  * 1000))
    {
      HANAtimer = milli;
      double v;

      if ((Kessel.Hauptantriebzeit - HAz) && (Kessel.Schneckengesamtlaufzeit - NAz)) // Wenn der HA lief
      {
        v = (float) (Kessel.Hauptantriebzeit - HAz) / ((float)(Kessel.Schneckengesamtlaufzeit - NAz) * 1000.0  ) ;
        sprintf(msg, "%f", v);
        client.publish("kwb/HANA", msg);
        NAz = Kessel.Schneckengesamtlaufzeit;
        HAz = Kessel.Hauptantriebzeit;
      }
    }

    oKessel.Schneckengesamtlaufzeit = Kessel.Schneckengesamtlaufzeit;

    sprintf(msg, "%d", 1 - Kessel.KeineStoerung);
    client.publish("kwb/Stoerung", msg);

    oKessel.Leistung = Kessel.Leistung;
    sprintf(msg, "%.3f", Kessel.kwh);
    client.publish("kwb/kwh", msg);

    sprintf(msg, "%.3f", Kessel.Brennerstunden);
    client.publish("kwb/Brennerstunden", msg);

    sprintf(msg, "%d", Kessel.Pumpepuffer );
    client.publish("kwb/Pumpepuffer", msg);

    sprintf(msg, "%d", Kessel.Raumaustragung);
    client.publish("kwb/Raumaustragung", msg);

    sprintf(msg, "%.1f", Kessel.Kesseltemperatur);
    client.publish("kwb/Kesseltemperatur", msg);

    sprintf(msg, "%.1f", Kessel.Rauchgastemperatur);
    client.publish("kwb/Rauchgastemperatur", msg);

    sprintf(msg, "%.0f / %.0f", Kessel.Saugzug,Kessel.Geblaese);
    client.publish("kwb/Saugzug", msg);

    sprintf(msg, "%.1f", Kessel.Unterdruck );
    client.publish("kwb/Unterdruck", msg);
      
    if (Kessel.Geblaese > 10.0)
      client.publish("kwb/Kessel", "brennt");
    else
      client.publish("kwb/Kessel", "aus");
   
    sprintf(msg, "%d", Kessel.ext);
    client.publish("kwb/Anforderung", msg);

    sprintf(msg, "%d", ((int) (Kessel.photo + 255.0) * 100) >> 9);
    client.publish("kwb/photodiode", msg);

    memcpy(&oKessel, &Kessel, sizeof Kessel);
    
    timer1 = milli;
  } // timer1 Ausgabe alle 5min

  // Delay am Loopende damit SW Watchdog nicht auslöst
  delay(5);
}
