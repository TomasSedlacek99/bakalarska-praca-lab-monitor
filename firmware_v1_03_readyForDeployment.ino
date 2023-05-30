/*
  @author Tomáš Sedláček  
*/

#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <ESP8266HTTPClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Ticker.h>

/* -------- Definícia makier -------- */
#define DELAYTIME (1000)
// Faktor pre konverziu CPM na mikroSieverty za hodinu.
#define CONVERSION_FACTOR (0.00812037)

/* -------- Definícia makier sluzby Blynk -------- */
#define BLYNK_TEMPLATE_ID           "TMPL496i5WJDA"
#define BLYNK_TEMPLATE_NAME         "LabMonitor"
#define BLYNK_AUTH_TOKEN            "d4_5AMI1M8hUsIwRpst6orH3uLcQy0lW"
#define NOTIFICATION_PIN V1
#define TEMP_PIN V2
#define HUM_PIN V3
#define PRESS_PIN V4
#define RAD_PIN V5

/* -------- Hlavna struktura pre uchovavanie udajov -------- */
struct SensorData {
  float temperature;
  float humidity;
  float pressure;
  float radiationLevel;
};

/* -------- Definícia a inicializácia hardvérových komponentov a senzorov -------- */
// Inicializácia senzoru BME280 pre meranie teploty, vlhkosti a tlaku.
Adafruit_BME280 bme;
// PIR senzor je pripojený na pin D7.
int inputPinPIR = D7; 
// Detektor radiácie je pripojený na pin D8.
int radiationPin = D8;
// Počítadlo pre detekciu pulzov radiácie.
volatile int radiationCounter = 0;

/* -------- Wi-Fi nastavenia -------- */
// Wi-Fi SSID a heslo pre pripojenie k sieti.
const char* ssid = "Pixel_3012";
const char* password = "Kecaldes1999";

/* -------- Práhové hodnoty a rastové limity -------- */
const int NOTIFICATION_THRESHOLD_TEMP = 30; // Teplotný prah pre notifikácie
const int NOTIFICATION_THRESHOLD_HUM = 70; // Prah vlhkosti pre notifikácie 
const int NOTIFICATION_THRESHOLD_RAD = 5; // Prah radiácie pre notifikácie 

// Relevantne hodnoty rastu pri velicinach
const float MIN_RELEVANT_TEMP_INCREASE = 1.0; // Minimálny relevantný rast teploty (°C)
const float MIN_RELEVANT_HUM_INCREASE = 5.0; // Minimálny relevantný rast vlhkosti (%)
const float MIN_RELEVANT_RAD_INCREASE = 0.3; // Minimálny relevantný rast úrovne žiarenia (mikroSievert)

// Veľkosť histórie pre uchovávanie posledných nameraných hodnôt.
const int HISTORY_SIZE = 5;
// História meraní pre jednotlivé senzory.
float tempHistory[HISTORY_SIZE];
float humHistory[HISTORY_SIZE];
float radHistory[HISTORY_SIZE];
// Index do histórie.
int historyIndex = 0;

/* -------- Premenné pre časovanie a intervaly -------- */
// Cas poslednej odoslanej notifikacie.
unsigned long lastNotificationTime = 0;  // Vytvorenie globálnej premennej
// Prahy a intervaly pre adaptívne meranie.
const int MIN_MEASUREMENT_INTERVAL = 10000; // 10 sekúnd v milisekundách
const int MED_MEASUREMENT_INTERVAL = 30000; // 30 sekúnd v milisekundách
const int MAX_MEASUREMENT_INTERVAL = 60000; // 60 sekúnd v milisekundách
const int MIN_CPM_THRESHOLD = 25; // Práh pre najnižší interval merania
const int MED_CPM_THRESHOLD = 125; // Stredny prah pre zmenu intervalu
const int MAX_CPM_THRESHOLD = 1200; // Najvyssi prah pre zmenu intervalu

// Aktuálny interval merania a plánovaný interval pre ďalšie meranie.
int adaptiveMeasurementInterval = MIN_MEASUREMENT_INTERVAL;
int nextAdaptiveMeasurementInterval = 0;
// Počet pulzov za minútu a počet meraní za minútu.
int cpmToMin;
int countsPerMinute = 0;

// Flag označujúci, či je práve prebiehajúce meranie.
bool measurementInProgress = false;
// Flag označujúci, či bol práve aktualizovaný interval merania.
bool intervalUpdated = false;
// Čas začiatku posledného merania.
unsigned long measurementStartMillis = 0;

// Čas posledného zaznamenaného pulzu radiácie.
volatile unsigned long lastRadiationPulseTime = 0;
// Odbudzaci interval pre detekciu pulzov radiácie.
const unsigned long debounceInterval = 10; 

/* -------- Premenná pre uchovávanie údajov -------- */
SensorData data;

/* -------- Tickery pre asynchrónne spúšťanie úloh -------- */
// Ticker pre kontrolu PIR senzora.
Ticker pirSensorTicker;
// Ticker pre získavanie údajov zo senzorov.
Ticker sensorsDataTicker;
// Ticker pre pravidelné odosielanie údajov na server.
Ticker sendDataRegularTicker;

/*
  Prvotná konfigurácia vstupov, nastavenie boudrate a otestovanie funkčnosti
  pripojených snímačov a inicializácia Wi-Fi pripojenia.
*/
void setup(){
  // Nastavenie rýchlosti sériového portu (baud rate) na 9600 bps.
  Serial.begin(9600); 

  // Test funkčnosti senzoru BME280.
  bool status = bme.begin(0x77);
  if (!status) {
    // Ak sa senzor BME280 nenašiel, vypíše sa chybové hlásenie.
    Serial.println("Senzor BME280 nebol najdeny! Skontroluj pripojenie...");
  }

  delay(DELAYTIME*5);
  
  // Inicializácia PIR senzora.
  pinMode(inputPinPIR, INPUT); // Nastavenie pinu PIR senzora ako vstupu.
  pirSensorTicker.attach(2, handlePirSensor); // Pripojenie funkcie handlePirSensor k PIR senzoru.

  // Pravidelné posielanie dát do aplikácie Blynk a na server každých 5 minút.
  sendDataRegularTicker.attach(300, sendData); // Pripojenie funkcie sendData k odosielateľu dát.

  // Pravidelné načítanie dát zo senzorov každých 10 sekúnd.
  sensorsDataTicker.attach(10, handleSensors); // Pripojenie funkcie handleSensors k čítaču senzorov.

  // Začiatok série testov pred spustením hlavnej slučky.
  Serial.println();
  Serial.println("------------------------------------");
  Serial.println("Test pred spustenim...");
  delay(DELAYTIME); 
  Serial.println();
  
  // Inicializácia Wi-Fi pripojenia.
  Serial.println("------------------------------------");
  Serial.print("Spustanie Wi-Fi.....");
  WiFi.begin(ssid,password);
  Serial.print("Pripajam sa na: ");
  Serial.print(ssid);
  Serial.print(" ...");

  // Kontrola stavu pripojenia.
  int i = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(DELAYTIME);
    Serial.print(++i);
    Serial.print(" ");
  }

  // Potvrdenie úspešného pripojenia.
  Serial.println();
  Serial.println("Wi-Fi pripojene...");
  Serial.println("IP adresa zariadenia: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  // Spustenie komunikácie s aplikáciou Blynk.
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
}

/* -------- Hlavná slučka, ktorá neustále beží počas chodu programu -------- */
void loop(){
  // Spustenie funkcie, ktorá neustále komunikuje s aplikáciou Blynk.
  Blynk.run();
}

/* -------- Funkcia pre načítanie údajov zo všetkých senzorov -------- */
void readSensors(){
  // Čítanie údajov z atmosferického senzora.
  data.temperature = bme.readTemperature();
  data.humidity = bme.readHumidity();
  data.pressure = bme.readPressure();
  // Čítanie údajov z radiačnej dosky.
  data.radiationLevel = readRadiationSensor();
}

/* -------- Funkcia pre výpis údajov zo senzorov na sériový port -------- */
void printData(SensorData data){
  // Meranie teploty
  Serial.print("Teplota = ");
  Serial.print(data.temperature);
  Serial.println(" *C");
  // Meranie tlaku
  Serial.print("Tlak = ");
  Serial.print(data.pressure / 100.0F);
  Serial.println(" hPa");
  // Meranie vlhkosti vzduchu
  Serial.print("Vlhkosť = ");
  Serial.print(data.humidity);
  Serial.println(" %");
  // Data z radiacnej dosky
  Serial.print("Uroven radiacie = ");
  Serial.print(data.radiationLevel, 8);
  Serial.println(" mikroSievert");
}

/* -------- Funkcia na spracovanie údajov zo senzorov -------- */
void handleSensors(){
  // Čítanie údajov zo senzorov BME280 a radiacnej dosky.
  readSensors();

  // Výpis stavu pre lepšiu sledovateľnosť procesu spracovania údajov.
  Serial.println();
  Serial.println("Handling senzorov...");
  Serial.println();
  // Výpis načítaných údajov na sériový port pre kontrolu.
  printData(data);

  // Aktualizácia histórie meraných hodnôt. Index histórie sa posúva v cykle, čím sa dosiahne prekrývanie starých hodnôt novými.
  tempHistory[historyIndex] = data.temperature;
  humHistory[historyIndex] = data.humidity;
  radHistory[historyIndex] = data.radiationLevel;
  historyIndex = (historyIndex + 1) % HISTORY_SIZE; // Udržiava index v rámci rozsahu histórie.

  // Analýza získaných údajov, či neprekračujú stanovené prahy a či sa nevyskytli abnormality.
  if (analyzeData(data)) {
    // Ak boli detekované abnormálne hodnoty, vypíše sa hlásenie a údaje sa odošlú na server a do Blynk.
    Serial.println();
    Serial.println("Abnormalita! Odosielam data na server a do Blynk...");
    Serial.println();
    sendNotification();
    sendData();
  }
}

/*
  Funkcia zaznamenáva stav PIR senzora (detektoru pohybu).
  Ak je zaznamenaný pohyb (PIR senzor vráti hodnotu HIGH),
  vypíše sa upozornenie a odošle sa notifikácia.
*/
void handlePirSensor() {
  // Načítanie stavu PIR senzora.
  int pirState = digitalRead(inputPinPIR);
  // Ak senzor zaznamenal pohyb (stav je HIGH).
  if (pirState == HIGH) {
    Serial.println();
    Serial.println("POZOR! Zaznamenaný pohyb...");
    // Odošleme notifikáciu o zaznamenanom pohybe a aktuálne údaje.
    sendPirNotification();
    sendData();
  } else {
    // Ak ubehlo 5 minút od poslednej notifikácie, resetujeme notifikačný pin
    if (millis() - lastNotificationTime >= 300000) {
      Blynk.virtualWrite(NOTIFICATION_PIN, 0);
    }    
  }
}

/*
  Funkcia odošle notifikáciu, ak je zariadenie pripojené a ak ubehlo
  viac ako 5 minút od poslednej odoslanej notifikácie.
*/
void sendPirNotification() {
  // Ak je zariadenie pripojené ku službe Blynk.
  if (Blynk.connected()) {
    unsigned long currentTime = millis();
    // Ak už ubehlo 5 minút od poslednej notifikácie
    if (currentTime - lastNotificationTime >= 300000) {  
      String message = "Pohyb v laboratoriu!";
      // Nastavenie hodnoty notifikačného pinu na 2 (čím sa odošle notifikácia).
      Blynk.virtualWrite(NOTIFICATION_PIN, 2);
      // Aktualizácia času poslednej notifikácie.
      lastNotificationTime = currentTime; 
    }
  }
}

/*
  Funkcia je vyvolaná pri prerušení vtedy, keď je zaznamenaný impulz z radiačnej dosky.
  Aktualizuje počítadlo rádiácie a čas posledného impulzu.
*/
void ICACHE_RAM_ATTR onRadiationPulse() {
  unsigned long currentMillis = millis();
  if(currentMillis - lastRadiationPulseTime > debounceInterval){
    radiationCounter++;
    lastRadiationPulseTime = currentMillis;
  }
}

/*
  Funkcia spustí meranie rádiácie a aktualizuje adaptívny interval merania
  na základe počtu impulzov za minútu (CPM).
*/
void updateRadiationSensor() {
  unsigned long currentMillis = millis();
  // Ak neprebieha meranie, inicializuj ho.
  if (!measurementInProgress) {
    Serial.println("SPUSTAM MERANIE....");
    measurementInProgress = true;
    radiationCounter = 0;
    attachInterrupt(digitalPinToInterrupt(radiationPin), onRadiationPulse, RISING);
    measurementStartMillis = currentMillis;
    intervalUpdated = false;
    return;
  }

  // Ak meranie prebieha.
  if (measurementInProgress) {
    if (!intervalUpdated) {
      int testCPM = radiationCounter * 6;
      Serial.println("DYNAMICKY PREDLZUJEM MERANIE...");

      // Dynamická zmena adaptívneho intervalu merania na základe počtu impulzov CPM
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
  Funkcia číta hodnotu radiácie a vráti ju ako uSv za hodinu.
  Spustí aktualizáciu hodnoty radiácie a vráti hodnotu z posledného merania.
*/
float readRadiationSensor(){
  updateRadiationSensor();
  // Konverzia počtu impulzov za minútu (CPM) na uSv/hod.
  float uSvPerHour = countsPerMinute * CONVERSION_FACTOR;
  return uSvPerHour;
}
 
/*
  Funkcia odošle notifikáciu prostredníctvom služby Blynk, ak je zariadenie pripojené.
  Notifikácia sa odošle na definovaný NOTIFICATION_PIN.
*/
void sendNotification(){
  if (Blynk.connected()) {
    Blynk.virtualWrite(NOTIFICATION_PIN, 1);
  }
}

/*
  Funkcia sprostredkováva odosielanie nameraných údajov do služby Blynk a do zariadenia Meshlium.
*/
void sendData(){
  sendDataToBlynk();
  sendDataToMeshlium();
}

/*
  Funkcia odošle namerané hodnoty do služby Blynk, ak je zariadenie pripojené.
  Hodnoty (teplota, vlhkosť, tlak a úroveň rádiácie) sa odošlú na príslušné virtuálne piny.
*/
void sendDataToBlynk() {
  if (Blynk.connected()) {
    Blynk.virtualWrite(TEMP_PIN, data.temperature);
    Blynk.virtualWrite(HUM_PIN, data.humidity);
    Blynk.virtualWrite(PRESS_PIN, data.pressure / 100.0F);
    Blynk.virtualWrite(RAD_PIN, data.radiationLevel);    
  }
}

/*
  Funkcia na konverziu reťazca na jeho hexadecimálnu reprezentáciu.
*/
String toHexString(String str) {
  // Inicializácia výstupného hexadecimálneho reťazca.
  String hexString = "";
  
  // Získanie dĺžky vstupného reťazca a vytvorenie char pola s adekvátnou dĺžkou.
  unsigned int str_len = str.length() + 1;
  char str_array[str_len];
  
  // Kopírovanie reťazca do char pola.
  str.toCharArray(str_array, str_len);
  
  // Iterácia cez všetky znaky v poli.
  for (int i = 0; i < str_len - 1; i++) {
    char buffer[3];
    // Konverzia aktuálneho znaku na hexadecimálne číslo.
    sprintf(buffer, "%02X", str_array[i]);
    // Pripojenie konvertovaného znaku k reťazcu.
    hexString += buffer;
  }
  
  // Vrátenie výsledného reťazce.
  return hexString;
}

/*
  Funkcia pre generovanie ASCII rámcu.
*/
String generateAsciiFrame(String serialID, String waspmoteID, uint8_t sequence, String sensorData1, String sensorData2, String sensorData3) {
  // Inicializácia ASCII rámcu s úvodným oddeľovačom a typom rámcu (A pre ASCII).
  String frame = "<=>A";
  // Pripojenie počtu polí (3 pre tento príklad).
  frame += String(3);
  frame += "#";
  // Pripojenie ID sériového rozhrania.
  frame += serialID;
  frame += "#";
  // Pripojenie Waspmote ID.
  frame += waspmoteID;
  frame += "#";
  // Pripojenie sekvencie rámcu.
  frame += String(sequence);
  frame += "#";
  // Pripojenie dát z senzoru.
  frame += sensorData1;
  frame += "#";
  // Pripojenie dát z senzoru.
  frame += sensorData2;
  frame += "#";
  // Pripojenie dát z senzoru.
  frame += sensorData3;
  frame += "#";
  // Konverzia ASCII rámca do hexadecimálnej podoby.
  frame = toHexString(frame);
  
  // Vrátenie výsledného rámca v hexadecimálnej reprezentácii.
  return frame;
}

/*
  Funkcia pre odosielanie údajov do zariadenia Meshlium. !! Je dôležité poznamenať, 
  že táto funkcia bola navrhnutá len pre testovacie účely a skutočná implementácia 
  nebola dokončená, keďže zariadenie Meshlium nespolupracovalo so zariadením tak, 
  ako bolo očakávané pred začatím vývoja zariadenia. !!
*/
void sendDataToMeshlium() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClientSecure client;
    
    String meshliumURL = "http://87.197.145.246:80";

    // Tieto reťazce boli určené pre testovanie a neodrážajú skutočné hodnoty senzorov. 
    // Konečná implementácia nebola dokončená z vyššie uvedených dôvodov.
    String sensorData1 = "RAD:10";
    String sensorData2 = "BAT:90";
    String sensorData3 = "STR:this_is_a_string";
    String frame = generateAsciiFrame("4F591CE819623CB1", "node_01", 1, sensorData1, sensorData2, sensorData3);

    String url = meshliumURL + "/getpost_frame_parser.php?frame=" + frame;
    Serial.println(url);
    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      Serial.print("Úspešne odoslané údaje na Meshlium, HTTP kód: ");
      Serial.println(httpResponseCode);
    } else {
      Serial.print("Chyba pri odosielaní údajov na Meshlium, HTTP kód: ");
      Serial.println(httpResponseCode);
    }
    http.end();
  } else {
    Serial.println("Chyba: Nie je pripojené k Wi-Fi.");
  }
}

/*
  Funkcia analyzuje dáta zo senzorov a rozhoduje, či je potrebné poslať notifikáciu do služby Blynk. 
  Notifikácia sa odošle, ak aspoň jedna z hodnôt prekročí definovaný prah alebo ak hodnota kontinuálne narastá.
*/
bool analyzeData(SensorData data) {
  int tempIncreasing = 0;
  int humIncreasing = 0;
  int radIncreasing = 0;
  bool tempOverThreshold = false;
  bool humOverThreshold = false;
  bool radOverThreshold = false;

  // Kontroluje, či hodnoty z meraní (teploty, vlhkosti a úrovne rádiácie) kontinuálne rastú.
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

  // Kontroluje posledný prvok histórie, či prekročil určený prah.
  int lastIndex = HISTORY_SIZE - 1;
  if (tempHistory[lastIndex] >= NOTIFICATION_THRESHOLD_TEMP) {
    Serial.println("Teplota prekrocila threshold...");
    tempOverThreshold = true;
  }
  if (humHistory[lastIndex] >= NOTIFICATION_THRESHOLD_HUM) {
    Serial.println("Vlhkost prekrocila threshold...");
    humOverThreshold = true;
  }
  if (radHistory[lastIndex] >= NOTIFICATION_THRESHOLD_RAD) {
    Serial.println("Radiacia prekrocila threshold...");
    radOverThreshold = true;
  }

  // Vracia true, ak aspoň jedna z hodnôt prekročila prah alebo ak aspoň 3 z 5 posledných meraní ukázali kontinuálny rast.
  return tempOverThreshold || humOverThreshold || radOverThreshold 
  || (tempIncreasing >= 3) || (humIncreasing >= 3) || (radIncreasing >= 3);
}



