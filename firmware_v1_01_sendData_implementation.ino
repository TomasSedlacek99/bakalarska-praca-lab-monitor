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

// Aktuálne využívané knižnice
#include <ESP8266WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>
#include "index.h"

// Makrá 
#define SEALEVELPRESSURE_HPA (1013.25)
#define DELAYTIME (1000)
#define CONVERSION_FACTOR (0.00812037)

// Blynk definitions
#define BLYNK_TEMPLATE_ID           "TMPL496i5WJDA"
#define BLYNK_TEMPLATE_NAME         "LabMonitor"
#define BLYNK_AUTH_TOKEN            "d4_5AMI1M8hUsIwRpst6orH3uLcQy0lW"
#define NOTIFICATION_PIN V1
#define TEMP_PIN V2
#define HUM_PIN V3
#define PRESS_PIN V4
#define RAD_PIN V5

// Struktura pre uchovavanie zozbieravanych udajov 
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float radiationLevel;
};

// Inicializácia premenných
Adafruit_BME280 bme;
int inputPinPIR = D7; 

// Syslia 15
// 71740874
// Pixel_3012
// Kecaldes1999
const char* ssid = "Syslia 15";
const char* password = "71740874";
ESP8266WebServer server(80); 
int radiationPin = D8;
volatile int radiationCounter = 0;

Ticker pirSensorTicker;
Ticker sensorsDataTicker;
Ticker blynkRegularTicker;

// Stanovene tresholdy
const int NOTIFICATION_THRESHOLD_TEMP = 30; // Teplotný prah pre notifikácie (upravte podľa potreby)
const int NOTIFICATION_THRESHOLD_HUM = 70; // Prah vlhkosti pre notifikácie (upravte podľa potreby)
const int NOTIFICATION_THRESHOLD_RAD = 5; // Prah radiácie pre notifikácie (upravte podľa potreby)
const int HISTORY_SIZE = 5;

// Relevantne hodnoty rastu pri velicinach
const float MIN_RELEVANT_TEMP_INCREASE = 1.0; // Minimálny relevantný rast teploty (°C)
const float MIN_RELEVANT_HUM_INCREASE = 5.0; // Minimálny relevantný rast vlhkosti (%)
const float MIN_RELEVANT_RAD_INCREASE = 0.3; // Minimálny relevantný rast úrovne žiarenia (mikroSievert)

const unsigned long SEND_INTERVAL = 300000; // 5 minút v milisekundách
unsigned long lastSendTime = 0;

const unsigned long tempHumRadCheckInterval = 10000;  // 10 sekúnd
unsigned long lastTempHumRadCheck = 0; 
unsigned long lastRadiationCheck = 0;

const int MIN_MEASUREMENT_INTERVAL = 10000; // 10 sekúnd v milisekundách
const int MED_MEASUREMENT_INTERVAL = 30000; // 30 sekúnd v milisekundách
const int MAX_MEASUREMENT_INTERVAL = 60000; // 60 sekúnd v milisekundách
const int MIN_CPM_THRESHOLD = 25; // Práh pre najnižší interval merania
const int MED_CPM_THRESHOLD = 125; // Stredny prah pre zmenu intervalu
const int MAX_CPM_THRESHOLD = 1200; // Najvyssi prah pre zmenu intervalu

int adaptiveMeasurementInterval = MIN_MEASUREMENT_INTERVAL;
int nextAdaptiveMeasurementInterval = 0;

int cpmToMin;
int countsPerMinute = 0;

bool measurementInProgress = false;
unsigned long measurementStartMillis = 0;
bool intervalUpdated = false;

// Polia pre 5 poslednych nameranych hodnot
float tempHistory[HISTORY_SIZE];
float humHistory[HISTORY_SIZE];
float radHistory[HISTORY_SIZE];
int historyIndex = 0;

// Vytvorenie HTML stránky na webserveri 
void handleRoot(){
  String s = MAIN_page;
  server.send(200,"text/html",s);
}

SensorData data;

void readSensors() {
  // Read atmospheric sensor data
  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure();
  
  // Read radiation board data
  data.radiationLevel = readRadiationSensor();
}

void printData(SensorData data){
  //Meranie teploty
  Serial.print("Teplota = ");
  Serial.print(data.temperature);
  Serial.println(" *C");

  //Meranie tlaku
  Serial.print("Tlak = ");
  Serial.print(data.pressure / 100.0F);
  Serial.println(" hPa");

  //Meranie vlhkosti vzduchu
  Serial.print("Vlhkosť = ");
  Serial.print(data.humidity);
  Serial.println(" %");

  //Data z radiacnej dosky
  Serial.print("Uroven radiacie = ");
  Serial.print(data.radiationLevel, 8);
  Serial.println(" mikroSievert");
}

/* Prvotná konfigurácia vstupov, nastavenie boudrate a otestovanie funkčnosti
   pripojených snímačov. 
   Inicializácia Wi-Fi pripojenia a po úspešnom pripojení spustenie web servera.  
*/
void setup() {

  // Inicializacia PIR snimaca
  pinMode(inputPinPIR, INPUT);
  pirSensorTicker.attach(2, handlePirSensor);

  // Posle kazdych 5 minut data do Blynk
  // Neskor premenovat na sendDataRegularTicker!!!!
  // Vytvorit metodu sendData() zdruzujucu posielanie udajov do Blynk a na Meshlium!!!
  blynkRegularTicker.attach(300, sendData);

  Serial.begin(9600); // nastavenie boudrate
  bool status;

  status = bme.begin(0x77);
  if (!status) {
    Serial.println("Senzor BME280 nebol najdeny! Skontroluj pripojenie...");
  }
  delay(DELAYTIME*5);

  sensorsDataTicker.attach(10, handleSensors);

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
  Serial.println(WiFi.localIP());
  Serial.println();

  // Setup sluzby Blynk pre posielanie notifikacii a jednoduchy dashboard
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);

  //server.on("/", handleRoot);
  //server.on("/readADC", handleADC);
  
  //server.begin();
  Serial.println("HTTP server spustený");
}

/*
  Hlavný loop starajúci sa o spracovávanie HTTP požiadaviek servera,
  nastavenie času oneskorenia pred spracovaním ďalšej požiadavky. 
*/
void loop(){
  Blynk.run();
  //server.handleClient();
}

// Zbera udaje z BME senzoru a radiacnej dosky
void handleSensors(){
  readSensors();
  Serial.println();
  Serial.println("Handling senzorov...");
  Serial.println();
  printData(data);

  // Aktualizacia historie hodnot
  tempHistory[historyIndex] = data.temperature;
  humHistory[historyIndex] = data.humidity;
  radHistory[historyIndex] = data.radiationLevel;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE;

  if (analyzeData(data)) {
    Serial.println();
    Serial.println("Abnormalita! Odosielam data na server a do Blynk...");
    Serial.println();
    sendNotification();
    sendData();
  }
}

/* 
  Prečíta hodnotu na digitálnom vstupe a podľa úrovne určí, 
  či bol alebo nebol zaznamenaný pohyb. 
*/
void handlePirSensor() {
  int pirState = digitalRead(inputPinPIR);  // Získaj len PIR hodnotu
  if (pirState == HIGH) {
    Serial.println();
    Serial.println("POZOR! Zaznamenaný pohyb...");
    sendPirNotification();
    sendData();
  } else {
    Blynk.virtualWrite(NOTIFICATION_PIN, 0);    
  }
}

/*
  Odošle notifikáciu o pohybe v miestnosti do Blynk
*/
void sendPirNotification() {
  if (Blynk.connected()) {
    String message = "Pohyb v laboratoriu!";
    Blynk.virtualWrite(NOTIFICATION_PIN, 2);
  }
}

volatile unsigned long lastRadiationPulseTime = 0;
const unsigned long debounceInterval = 10; // Odbudzaci interval v milisekundach

void ICACHE_RAM_ATTR onRadiationPulse() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastRadiationPulseTime > debounceInterval){
    radiationCounter++;
    lastRadiationPulseTime = currentMillis;
    Serial.println("IMPULZ ZAZNAMENANY");
    Serial.println("Aktualny pocet:");
    Serial.println(radiationCounter);
  }
}

void updateRadiationSensor() {
  unsigned long currentMillis = millis();

  if (!measurementInProgress) {
    Serial.println("SPUSTAM MERANIE....");
    measurementInProgress = true;
    radiationCounter = 0;
    attachInterrupt(digitalPinToInterrupt(radiationPin), onRadiationPulse, RISING);
    measurementStartMillis = currentMillis;
    intervalUpdated = false;
    return;
  }

  if (measurementInProgress) {
    if (!intervalUpdated) {
      int testCPM = radiationCounter * 6;
      Serial.println("DYNAMICKY PREDLZUJEM MERANIE...");

      // Dynamická zmena adaptívneho intervalu merania
      if (testCPM < MED_CPM_THRESHOLD) {
        adaptiveMeasurementInterval = MAX_MEASUREMENT_INTERVAL;
      } else if (testCPM >= MED_CPM_THRESHOLD && testCPM < MAX_CPM_THRESHOLD) {
        adaptiveMeasurementInterval = MED_MEASUREMENT_INTERVAL;
      } else if (testCPM >= MAX_CPM_THRESHOLD) {
        adaptiveMeasurementInterval = MIN_MEASUREMENT_INTERVAL;
      }

      intervalUpdated = true;
    }

    // Ukončenie merania, ak uplynul adaptívny interval
    if (currentMillis - measurementStartMillis >= adaptiveMeasurementInterval && intervalUpdated) {
      measurementInProgress = false;
      detachInterrupt(digitalPinToInterrupt(radiationPin));
      // Nastavenie konverzie na minutu podla aktualneho intervalu
      if(adaptiveMeasurementInterval == MIN_MEASUREMENT_INTERVAL){
        cpmToMin = 6;
      }
      if(adaptiveMeasurementInterval == MED_MEASUREMENT_INTERVAL){
        cpmToMin = 2;
      }
      if(adaptiveMeasurementInterval == MAX_MEASUREMENT_INTERVAL){
        cpmToMin = 1;
      }
      countsPerMinute = radiationCounter * cpmToMin;
      Serial.println("MERANIE UKONCENE...");
      Serial.println("Pocet preruseni: ");
      Serial.println(radiationCounter);
      Serial.println("CPM: ");
      Serial.println(countsPerMinute);
    }
  }
}

/*
  Spusti dalsie meranie a hodnotu z posledneho merania
*/
float readRadiationSensor(){
  updateRadiationSensor();
  // Prepocet CPM na minutu podla intervalu
  float uSvPerHour = countsPerMinute * CONVERSION_FACTOR;
  return uSvPerHour;
}
 
// Predpriprava pre funkciu posielajucu notifikacie prostrednictvom sluzby Blynk
void sendNotification(){
  if (Blynk.connected()) {
    Blynk.virtualWrite(NOTIFICATION_PIN, 1);
  }
}

void sendData(){
  sendDataToBlynk();
  sendDataToMeshlium();
}

// Funkcia pre odosielanie nameraných hodnôt do služby Blynk
void sendDataToBlynk() {
  if (Blynk.connected()) {
    Blynk.virtualWrite(TEMP_PIN, data.temperature);
    Blynk.virtualWrite(HUM_PIN, data.humidity);
    Blynk.virtualWrite(PRESS_PIN, data.pressure / 100.0F);
    Blynk.virtualWrite(RAD_PIN, data.radiationLevel);    
  }
}

// Funkcia pre odosielanie udajov do zariadenia Meshlium
void sendDataToMeshlium() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClientSecure client;
    
    // Nastavenie certifikátu pre overenie servera (ak je to potrebné)
    // client.setCACert(certifikat); 
    
    // URL vášho Meshlium zariadenia s HTTPS
    const char* meshliumURL = "https://your.meshlium.device.url/path";

    // Vytvorenie objektu JSON a pridanie údajov do neho
    StaticJsonDocument<200> jsonDocument;
    jsonDocument["temperature"] = data.temperature;
    jsonDocument["humidity"] = data.humidity;
    jsonDocument["pressure"] = data.pressure;
    jsonDocument["radiationLevel"] = data.radiationLevel;

    // Serializácia JSON objektu do reťazca
    String jsonString;
    serializeJson(jsonDocument, jsonString);

    // Odoslanie HTTPS POST požiadavky s JSON údajmi na Meshlium
    http.begin(client, meshliumURL);
    http.addHeader("Content-Type", "application/json");
    int httpResponseCode = http.POST(jsonString);

    if (httpResponseCode > 0) {
      Serial.print("Úspešne odoslané údaje na Meshlium, HTTP kód: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Chyba pri odosielaní údajov na Meshliu, HTTP kód: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Chyba: Nie je pripojené k Wi-Fi.");
  }
}

// Funkcia pre analyzu dat a rozhodovanie, ci sa ma odoslat notifikacia do sluzby Blynk
bool analyzeData(SensorData data) {
  int tempIncreasing = 0;
  int humIncreasing = 0;
  int radIncreasing = 0;
  bool tempOverThreshold = false;
  bool humOverThreshold = false;
  bool radOverThreshold = false;

  // Kontrola, ci sa kontinualne zvysuje merana velicina
  for (int i = 0; i < HISTORY_SIZE - 1; i++) {
    if (tempHistory[i] + MIN_RELEVANT_TEMP_INCREASE <= tempHistory[i + 1]) {
      tempIncreasing++;
    }
    if (humHistory[i] + MIN_RELEVANT_HUM_INCREASE <= humHistory[i + 1]) {
      humIncreasing++;
    }
    if (radHistory[i] + MIN_RELEVANT_RAD_INCREASE <= radHistory[i + 1]) {
      radIncreasing++;
    }
  }

  // Skontrolovať posledný prvok poľa
  int lastIndex = HISTORY_SIZE - 1;
  if (tempHistory[lastIndex] >= NOTIFICATION_THRESHOLD_TEMP) {
    tempOverThreshold = true;
  }
  if (humHistory[lastIndex] >= NOTIFICATION_THRESHOLD_HUM) {
    humOverThreshold = true;
  }
  if (radHistory[lastIndex] >= NOTIFICATION_THRESHOLD_RAD) {
    radOverThreshold = true;
  }

  // Ak aspoň jedna z hodnôt prekročí prah alebo hodnota kontinuálne narastá, vráti true
  return tempOverThreshold || humOverThreshold || radOverThreshold 
  || (tempIncreasing >= 4) || (humIncreasing >= 4) || (radIncreasing >= 4);
}



