/*
  @author Tomáš Sedláček  
*/

#include <ArduinoWiFiServer.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiGratuitous.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiType.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>

// Aktuálne využívané knižnice
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include "index.h"

// Makrá 
#define SEALEVELPRESSURE_HPA (1013.25)
#define DELAYTIME (1000)

// Inicializácia premenných
Adafruit_BME280 bme;
int inputPinPIR = D7; 
const char* ssid = "Syslia 15";
const char* password = "71740874";
ESP8266WebServer server(80); 

// Vytvorenie HTML stránky na webserveri 
void handleRoot(){
  String s = MAIN_page;
  server.send(200,"text/html",s);
}

/* 
  Vytvorí JSON štruktúru data, naplní ju požadovanými dátami 
  pomocou volaní funkcií z knižnice Adafrut_BME280 a funkcie pirDetection a
  odošle dáta na nakonfigurovaný webserver.
*/
void handleADC(){
  String data = "{\"Teplota\":\"" + String(bme.readTemperature())
              + "\",\"Tlak\":\"" + String(bme.readPressure() / 100)
              + "\", \"Nadmorská výška\":\"" + String(bme.readAltitude(SEALEVELPRESSURE_HPA)) 
              + "\", \"Vlhkosť\":\"" + String(bme.readHumidity()) 
              + "\",\"Pohyb\":\"" + String(pirDetection())+"\"}";

  server.send(200,"text/plane",data);
  bmeValuesPrint();
}

/* Provtná konfigurácia vstupov, nastavenie boudrate a otestovanie funkčnosti
   pripojených snímačov. 
   Inicializácia Wi-Fi pripojenia a po úspešnom pripojení spustenie web servera.  
*/
void setup() {
  pinMode(inputPinPIR, INPUT);
  Serial.begin(9600); // nastavenie boudrate
  bool status;

  status = bme.begin(0x77);
  if (!status) {
    Serial.println("Senzor BME280 nebol najdeny! Skontroluj pripojenie...");
  }
  delay(DELAYTIME*5);

  Serial.println();
  Serial.println("------------------------------------");
  Serial.println("Test pred spustenim...");
  delay(DELAYTIME); //delay pred spustenim hlavneho loopu
  Serial.println();
  
  Serial.println("------------------------------------");
  Serial.print("Spustanie Wi-Fi.....");
  WiFi.begin(ssid,password);
  Serial.print("Pripajam sa na: ");
  Serial.print(ssid);
  Serial.print(" ...");

  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(DELAYTIME);
    Serial.print(++i);
    Serial.print(" ");
  }

  Serial.println();
  Serial.println("Wi-Fi pripojene...");
  Serial.println("IP adresa zariadenia: ");
  Serial.print(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/readADC", handleADC);
  
  server.begin();
  Serial.println("HTTP server spustený");
}

/*
  Hlavný loop starajúci sa o spracovávanie HTTP požiadaviek servera,
  nastavenie času oneskorenia pred spracovaním ďalšej požiadavky. 
*/
void loop(){
  server.handleClient();
  delay(DELAYTIME * 5); 
}

/* 
  Prečíta hodnotu na digitálnom vstupe a podľa úrovne určí, 
  či bol alebo nebol zaznamenaný pohyb. 
*/
String pirDetection(){
  int val = digitalRead(inputPinPIR); 
  if (val == HIGH) {
    Serial.println("Pohyb v miestnosti!");
    return ("Pohyb v miestnosti!");
    }
  else {
    Serial.println("Bez pohybu...");
    return ("Bez pohybu...");
    }
}

// Vypíše všetky zbierané atmosférické dáta na sériovú linku 
void bmeValuesPrint(){
  //Meranie teploty
  Serial.print("Teplota = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  //Meranie tlaku
  Serial.print("Tlak = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  /*
  //Priblizny odhad nadm. vysky
  Serial.print("Nadmorská výška = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  */

  //Meranie vlhkosti vzduchu
  Serial.print("Vlhkosť = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  Serial.println();
}
