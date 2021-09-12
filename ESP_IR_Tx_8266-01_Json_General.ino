/*
  IRremoteESP8266: IRrecvDumpV2 - dump details of IR codes with IRrecv
  An IR detector/demodulator must be connected to the input RECV_PIN.

  Copyright 2009 Ken Shirriff, http://arcfn.com
  Copyright 2017 David Conran

  Example circuit diagram:
   https://github.com/markszabo/IRremoteESP8266/wiki#ir-receiving

  Changes:
    Version 0.3 November, 2017
      - Support for A/C decoding for some protcols.
    Version 0.2 April, 2017
      - Decode from a copy of the data so we can start capturing faster thus
        reduce the likelihood of miscaptures.
  Based on Ken Shirriff's IrsendDemo Version 0.1 July, 2009,
*/

//#ifndef UNIT_TEST
//#include <Arduino.h>
//#endif
//Define el Tiempo de cada Interrupcion del Timer 0
//#define Time_1    1
#define Time_100  1
//Se define se se utiliza Debug por UART
//#define DEBUG 1
#define Time_60_Seg             600
//
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <IRutils.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <ArduinoJson.h>
#include <DHTesp.h>
#if DECODE_AC
#include <ir_Daikin.h>
#include <ir_Fujitsu.h>
#include <ir_Gree.h>
#include <ir_Haier.h>
#include <ir_Kelvinator.h>
#include <ir_Midea.h>
#include <ir_Toshiba.h>
#endif  // DECODE_AC
#define EEPROM_SIZE 512
//
//Configuracion DHT
DHTesp dht;
#define dhtPin 2        //Colocar D2 (GPIO4) o D5 (GPIO14) alli es que funciona
//Constantes que se Definen en el Programa
#define Addr_Init_Wifi          511
#define Data_Wifi_Prog          0
#define Data_Wifi_NProg         0xFF
//
#define AP_STA            0x01       //Estado: Inicial
#define STATION           0x00       //Estado: Inicial
ESP8266WebServer server(80);
// Variables para IR
const char* ssid_AP = "Home_IoT_";
String ssid_ap;
String deviceId;
String ID_ESP;
const char* paswr_ap = "0123456789";
IPAddress ip, gateway, maskSubnet;
//String  CMD_Rx;
//char    CMD_Rx[900];
uint16_t  Count_DHT;
uint8_t Count_DHT_Fail;
float   temperatura, humedad;
char    Caracter;
int     Cont_Rx;
int     Cont_IR;
uint8_t Cont_LedStatus;
uint8_t Count_Botton;
//String st;
String s_ini, s_home, s_wifi, s_fin;
String req;
String ssid, password;
bool   Hab_AP;                //1-> ESP Habilitado como Acces Point 0-> ESP DesHabilitado como AP
bool    Conect_AP;            //0-> ESP no Conectado al Router, 1-> ESP Conectado al Router;
bool  OTA_ENABLE;
bool  Modo_AP_STATION;          //0 - Modo Station.  1- Modo Acces Poinnt
bool  Flag_Prog_AP;
bool  Wait_INT_Start;
uint16_t  IR_Comand[250];
bool    Fin_CMD_Rx;
int Estado_LED;
int Cont_1S;
bool  Time_100mS_L;
bool  Time_100mS_B;
bool  Prim_V = true;
bool  ifTemp = false;
bool  ifHume = false;
bool  ifIr = true;
bool  ifLuz = false;
bool  ifMotor = false;
//Banderas para ver el Funcionamiento del equipo mediante el LED
bool  flagProg = false;
bool  flagConect = false;
#define Nro_CMD_Rx      900
//#define LED      1
#define LED_ON     0
#define LED_OFF     1
#define Tam_IR      220
#define BUTTON          0
#define Addr_Init_Wifi          511
#define Data_Wifi_AP_STA          0
#define Data_Wifi_STA         0xFF
#define Time_Rx   10
#ifndef DEBUG
  #define LedStatus  1
#endif
#define LED_OFF   1
#define LED_ON    0
#define Time_4000_uSeg          40
// ==================== start of TUNEABLE PARAMETERS ====================
// An IR detector/demodulator is connected to GPIO pin 14
#define IR_LED 3  // ESP8266 GPIO pin to use. Recommended: 4 (D2).
IRsend irsend(IR_LED);  // Set the GPIO to be used to sending the message.
// The Serial connection baud rate.
// i.e. Status message will be sent to the PC at this baud rate.
// Try to avoid slow speeds like 9600, as you will miss messages and
// cause other problems. 115200 (or faster) is recommended.
// NOTE: Make sure you set your Serial Monitor to the same speed.
//#define BAUD_RATE 115200

// As this program is a special purpose capture/decoder, let us use a larger
// than normal buffer so we can handle Air Conditioner remote codes.
#define CAPTURE_BUFFER_SIZE 1024

// TIMEOUT is the Nr. of milli-Seconds of no-more-data before we consider a
// message ended.
// This parameter is an interesting trade-off. The longer the timeout, the more
// complex a message it can capture. e.g. Some device protocols will send
// multiple message packets in quick succession, like Air Conditioner remotes.
// Air Coniditioner protocols often have a considerable gap (20-40+ms) between
// packets.
// The downside of a large timeout value is a lot of less complex protocols
// send multiple messages when the remote's button is held down. The gap between
// them is often also around 20+ms. This can result in the raw data be 2-3+
// times larger than needed as it has captured 2-3+ messages in a single
// capture. Setting a low timeout value can resolve this.
// So, choosing the best TIMEOUT value for your use particular case is
// quite nuanced. Good luck and happy hunting.
// NOTE: Don't exceed MAX_TIMEOUT_MS. Typically 130ms.
#if DECODE_AC
#define TIMEOUT 50U  // Some A/C units have gaps in their protocols of ~40ms.
// e.g. Kelvinator
// A value this large may swallow repeats of some protocols
#else  // DECODE_AC
#define TIMEOUT 15U  // Suits most messages, while not swallowing many repeats.
#endif  // DECODE_AC
// Alternatives:
// #define TIMEOUT 90U  // Suits messages with big gaps like XMP-1 & some aircon
// units, but can accidentally swallow repeated messages
// in the rawData[] output.
// #define TIMEOUT MAX_TIMEOUT_MS  // This will set it to our currently allowed
// maximum. Values this high are problematic
// because it is roughly the typical boundary
// where most messages repeat.
// e.g. It will stop decoding a message and
//   start sending it to serial at precisely
//   the time when the next message is likely
//   to be transmitted, and may miss it.

// Set the smallest sized "UNKNOWN" message packets we actually care about.
// This value helps reduce the false-positive detection rate of IR background
// noise as real messages. The chances of background IR noise getting detected
// as a message increases with the length of the TIMEOUT value. (See above)
// The downside of setting this message too large is you can miss some valid
// short messages for protocols that this library doesn't yet decode.
//
// Set higher if you get lots of random short UNKNOWN messages when nothing
// should be sending a message.
// Set lower if you are sure your setup is working, but it doesn't see messages
// from your device. (e.g. Other IR remotes work.)
// NOTE: Set this value very high to effectively turn off UNKNOWN detection.
#define MIN_UNKNOWN_SIZE 12
// ==================== end of TUNEABLE PARAMETERS ====================
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///// Interrupcion del Timer 0, cada 100 uSeg
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void timer0_ISR (void)
{
  //One_Seg = true;
  #ifdef  Time_1
    timer0_write(ESP.getCycleCount() + 80000000L);  // 80000000/80MHz == 1 Sec
  #endif
  #ifdef Time_100
    timer0_write(ESP.getCycleCount() + 8000000L);   // 8000000/80MHz == 100 uSec
  #endif
  Time_100mS_L = true;
  Time_100mS_B = true;
  Wait_INT_Start = true;
}




//*******************************************************************************
//Rutina Init_OTA_Arduino()
//  Inicializa y configura la actualizacion mediante OTA
//*******************************************************************************
void Init_OTA_Arduino(void)
{
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    noInterrupts();
    timer0_write(ESP.getCycleCount() + 80000000000L);
  });
  ArduinoOTA.onEnd([]() {
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  });
  ArduinoOTA.onError([](ota_error_t error) {
  });
  ArduinoOTA.begin();
  #ifdef DEBUG
    //Serial.println("Arduino OTA Activo");
  #endif
  OTA_ENABLE = true;

}
//void  Mode_Prog_AP(void)
//{
//      //Estado = Est_Prog;
//      delay(500);
//      WiFi.disconnect();
//      WiFi.persistent(false);
//      WiFi.mode(WIFI_OFF);
//      //WiFi.mode(WIFI_AP);
//      delay(100);
//      WiFi.mode(WIFI_AP_STA);
//      //WiFi.softAP(ssid_ap,paswr_ap);
//      delay(400);
//      WiFi.softAP(ssid_ap.c_str(), paswr_ap);
//      //WiFi.mode(WIFI_ON);
//      //server.begin();
//      conectarServer();
//      #ifdef DEBUG
//        Serial.println("");
//        Serial.println("Modo Programacion....");
//      #endif
//}
void  Mode_Prog_AP(void)
{
    String ID_ESP = String(ESP.getChipId(), HEX);
    ID_ESP.toUpperCase();
    //  Serial.println(ID_ESP.c_str());
    //  Serial.println("");
    //ChipID = String(ESP.getChipId());
    ssid_ap = ssid_AP;
    ssid_ap += ID_ESP;
      //Estado = Est_Prog;
      delay(500);
      WiFi.disconnect();
      WiFi.persistent(false);
      WiFi.mode(WIFI_OFF);
      delay(100);
      //WiFi.mode(WIFI_AP);
      WiFi.mode(WIFI_AP_STA);
      delay(400);
      //WiFi.softAP(ssid_ap,paswr_ap);
      Serial.println("SSID como Acces Point: "+ssid_ap);
      Serial.println("Pasword como Acces Point: "+String(paswr_ap));
      WiFi.softAP(ssid_ap.c_str(), paswr_ap);
      int Intentos = 0;

        server.on("/cmdo", cmdoJson);
        server.begin();
      //server.begin();
      //Serial.println("Modo Programacion...");
      //WiFi.printDiag(Serial);
}
//*******************************************************************************
//Rutina Function_Botton():
// Rutina monitores el Estado del Botton
//*******************************************************************************
void Function_Botton(void)
{
  if (!digitalRead(BUTTON))
  {
    if(Prim_V)
    {
      delay(20);
      if (!digitalRead(BUTTON))
      {
        Prim_V = false;
        Count_Botton = 0;
        #ifndef DEBUG
          digitalWrite(LedStatus,!digitalRead(LedStatus));
        #endif
      }
    }
  }
  else
  {
    Count_Botton = 0;
    Prim_V = true;
  }
  if(Time_100mS_B)
  {
    Count_Botton++;
    Count_DHT++;
    Time_100mS_B = false;
    //Function_LED();
    if(Count_Botton > Time_4000_uSeg)
    {
      Count_Botton = 0;
      #ifndef DEBUG
        digitalWrite(LedStatus, !digitalRead(LedStatus));
      #endif
      Mode_Prog_AP();
      flagProg = true;
    }
  }
}




// The section of code run only once at start-up.
void setup() {
  Param_ESP();
  #ifdef DEBUD
    Serial.begin(19200);
  #endif
  pinMode(IR_LED, INPUT);
  pinMode(BUTTON, INPUT);
  #ifndef DEBUG
    pinMode(LedStatus, OUTPUT);
    digitalWrite(LedStatus,LED_OFF);
  #endif
  EEPROM. begin(EEPROM_SIZE);
  char    Caracter;
  Caracter = EEPROM.read(Addr_Init_Wifi);
  ifIr = true;
  Cont_IR = 0;
  Cont_LedStatus = 0;
  Flag_Prog_AP = false;
  Wait_INT_Start = false;
  Count_DHT_Fail = 0;
  Init_OTA_Arduino();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
  interrupts();
  conectarAP();
  conectarServer();
  pinMode(IR_LED, INPUT);
  dht.setup(dhtPin, DHTesp::DHT11);
  irsend.begin();
}
// The repeating section of the code
//
void loop() {

  server.handleClient();
  ArduinoOTA.handle();
  Function_Botton();
  #ifndef DEBUG
    Function_Led();
  #endif
  Funcion_DHT();

}
////////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para ver el funcionamiento del Equipo
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//<>
#ifndef DEBUG
void Function_Led(){
  if(!Time_100mS_L){
    return;
  }
  Time_100mS_L = false;
  if(WiFi.status() != WL_CONNECTED){
    flagConect = false;
  }else{
    flagConect = true;
  }
  if(flagProg){
    if(Cont_LedStatus == 1 || Cont_LedStatus == 5 || Cont_LedStatus == 9){
      digitalWrite(LedStatus,LED_ON);
    }else{
      digitalWrite(LedStatus,LED_OFF);
    }
  }else{
    if(flagConect){
      if(Cont_LedStatus == 1){
        digitalWrite(LedStatus,LED_ON);
      }else{
        digitalWrite(LedStatus,LED_OFF);
      }
    }else{
      if(Cont_LedStatus == 1 || Cont_LedStatus == 5){
        digitalWrite(LedStatus,LED_ON);
      }else{
        digitalWrite(LedStatus,LED_OFF);
      }
    }
  }
  Cont_LedStatus++;
  if(Cont_LedStatus > 19)
    Cont_LedStatus = 0;
  
}
#endif
///////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Leer la Temp y Humedad del DHT
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Funcion_DHT(void){
  String jsonTx = "";
  if(Count_DHT > Time_60_Seg){
    Count_DHT = 0;
    delay(dht.getMinimumSamplingPeriod());
    
    if(dht.getStatusString() == "OK"){
      Count_DHT_Fail = 0;
      ifHume = true;
      ifTemp = true;
      float humidity = dht.getHumidity();
      float temperature = dht.getTemperature();
      temperatura = temperature;
      humedad = humidity;
      #ifdef  DEBUG
        Serial.print(dht.getStatusString());
        Serial.print("\t");
        Serial.print("Humedad: ");
        Serial.print(humidity, 1);
        Serial.print(" %");
        Serial.print("\t\t");
        Serial.print("Temperatura: ");
        Serial.print(temperature, 1);
        Serial.println(" °C");
      #endif
//      jsonTx ="{\"temp\":\""+(String)temperatura+"\",";
//      jsonTx +="\"hume\":\""+(String)humedad+"\"}";
//      webSocket.broadcastTXT(jsonTx);
    }
    else{
      Count_DHT_Fail++;
      if(Count_DHT_Fail > 9){
        Count_DHT_Fail = 0;
        ifHume = false;
        ifTemp = false;
      }
      #ifdef DEBUG
        Serial.println("Error al Leer DHT");
      #endif
    }
  }
}
///////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Configurar el ESP a conectarse al Router con el SSID y Password de la EEPROM
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void conectarAP(void){
  delay(500);
  WiFi.disconnect();
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  delay(100);
  //WiFi.mode(WIFI_AP);
  WiFi.mode(WIFI_STA);
  delay(400);
  if(Leer_EEPROM()==false){
    #ifdef DEBUG
      Serial.println("Se conecta sin EEPROM");
    #endif
//    WiFi.begin("Linksys24_AP", "sadamr-9295352");
      return;
  }else{
    #ifdef DEBUG
      Serial.println("Se conecta con EEPROM");
      Serial.println("SSID: "+ssid);
      Serial.println("Pass: "+password);
   #endif
   if(Leer_EEPROM_ip()==true){
    WiFi.config(ip, gateway, maskSubnet);
   }
   WiFi.begin(ssid.c_str(), password.c_str());
  }
  int Intentos = 0;
  #ifdef DEBUG
    Serial.println("*");
  #endif
  while (WiFi.status() != WL_CONNECTED && Intentos < 50) {
    delay(500);
    Intentos++;
    #ifdef DEBUG
      Serial.print(Intentos);
      Serial.print("*");
    #endif
  }
  #ifdef DEBUG
    Serial.println("");
  #endif
  if (WiFi.status() != WL_CONNECTED){
    Conect_AP = false;
    #ifdef DEBUG
      Serial.println("Error Conectando al Router");
    #endif
  }
  else{
    Conect_AP = true;
    #ifdef DEBUG
      Serial.println("Conectado al Rourer");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
    #endif
  }
  flagProg = false;
    
}
////////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Configurar el Server, Rutina que ejecutara el recibir el http
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void conectarServer(void){
  server.on("/cmdo", cmdoJson);
  server.begin();
}
////////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Determinar el Comando Recibido en el Json
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void cmdoJson(void){
  //StaticJsonDocument<1000> jsonBuffer;
  DynamicJsonDocument jsonBuffer(8196);
  DeserializationError error = deserializeJson(jsonBuffer, server.arg("plain"));
  String pay = server.arg("plain");   //Colocando la palabra "plain" guarda todo lo Recibido
  #ifdef DEBUG
    Serial.println("Payload Json Rx: ");
    Serial.println(pay);
  #endif
  if(error){
    #ifdef DEBUG
      Serial.println("Error al Rx Json");
    #endif
    String SJson = "{\"error\":1,\"data\":{\"deviceid\":\""+deviceId+"\"}}";
    #ifdef DEBUG
      Serial.print("Error al Rx Json: ");
      Serial.println(SJson);
    #endif
    server.send(200, "application/json", SJson);
    return;
  }
  String comando = jsonBuffer["cmdo"];
  if(comando == "status")
    statusJson();
  else if(comando == "reset")
    resetJson();
  else if(comando == "toggle")
    toggleJson();
  else if(comando == "accPoint")
    accPointJson();
  else if(comando == "conectAP")
    conectAPJson(jsonBuffer["data"]["ssid"], jsonBuffer["data"]["pass"]);
  else if(comando == "ctir")
  {
    String formatIR = jsonBuffer["data"]["format"];
    unsigned int frecIR = jsonBuffer["data"]["frec"];
    unsigned int longIR = jsonBuffer["data"]["long"];
    unsigned int repetIR = jsonBuffer["data"]["repet"];
    if(formatIR == "raw")
    {
      unsigned int i;
      for(i=0; i < longIR; i++){
        IR_Comand[i] = jsonBuffer["data"]["dataIR"][i];
      }
    }
    ctirJson(formatIR, frecIR, longIR, repetIR);
  }
  else
    errorJson();
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para enviar el Status del equipo en Json
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool statusJson(void){
  StaticJsonDocument<200> jsonBuffer;
  String json;
  //JsonObject json = jsonBuffer.createNestedObject();
  jsonBuffer["error"] = 0;
  jsonBuffer["deviceid"] = deviceId;
  JsonObject obj = jsonBuffer.createNestedObject("data");
  #ifndef DEBUG
    obj["switch"] = digitalRead(LedStatus) ? "ON" : "OFF";
  #endif
  obj["ifTemp"] = ifTemp;
  obj["ifHume"] = ifHume;
  obj["ifIr"] = ifIr;
  obj["ifLuz"] = ifLuz;
  obj["ifMotor"] = ifMotor;
  obj["temp"] = temperatura;
  obj["hume"] = humedad;
  serializeJson(jsonBuffer, json);
  #ifdef DEBUG
    Serial.print("cmdo Status, se Tx: ");
    Serial.print(json);
  #endif
  server.send(200, "application/json", json);
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para enviar hacer Reset del Equipo
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool resetJson(void){
  StaticJsonDocument<200> jsonBuffer;
  String json;
  //JsonObject json = jsonBuffer.createNestedObject();
  jsonBuffer["error"] = 0;
  jsonBuffer["deviceid"] = deviceId;
  serializeJson(jsonBuffer, json);
  server.send(200, "application/json", json);
  conectarAP();  
  conectarServer();
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para toggle del LED
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool toggleJson(void){
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para enviar la lista de Routers (AccesPoint) que ve el equipo
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool accPointJson(void){
  String SJson;
  StaticJsonDocument<1000> jsonBuffer;
  JsonObject s1 = jsonBuffer.createNestedObject();
  JsonObject json = jsonBuffer.createNestedObject();
  json["error"] = 0;
  json["deviceid"] = deviceId;
  JsonArray datosS = json.createNestedArray("wifi");
  String aux;
  int qualit;
  int n = WiFi.scanNetworks();
  if(n!=0){
    for(int i = 0; i < n; ++i)
    {
      int Senal_RSSI = WiFi.RSSI(i);
      if (Senal_RSSI <= -100)
        qualit = 0;
      else if (Senal_RSSI >= -50)
        qualit = 100;
      else
      {
        qualit = 2 * (Senal_RSSI + 100);
      }
      s1["SSID"]= WiFi.SSID(i);
      s1["QoS"]= qualit;
      s1["Encryption"]= WiFi.encryptionType(i);
      datosS.add(s1);
    }
    serializeJson(json, SJson);
    server.send(200, "application/json", SJson);
  }else{
    #ifdef DEBUG
      Serial.println("Cero redes Encontradas");
    #endif
    s1["SSID"]= "Sin Redes Encontradas";
    s1["QoS"]= 0;
    s1["Encryption"]= 0;
    datosS.add(s1);
    serializeJson(json, SJson);
    server.send(200, "application/json", SJson);
  }
  
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para conectarse el equipo al Router seleccionado
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool conectAPJson(String ssidRx, String passwRx){
  StaticJsonDocument<200> jsonBuffer;
  String json;
  //JsonObject json = jsonBuffer.createNestedObject();
  jsonBuffer["error"] = 0;
  jsonBuffer["deviceid"] = deviceId;
  #ifdef DEBUG
    Serial.println("SSID: "+ssidRx);
    Serial.println("Pass: "+passwRx);
  #endif
  WiFi.begin(ssidRx.c_str(), passwRx.c_str());
  //WiFi.begin("Linksys24_AP", "sadamr-9295352");
  int Intentos = 0;
  #ifdef DEBUG
      Serial.println("*");
  #endif
  while (WiFi.status() != WL_CONNECTED && Intentos < 25) {
    delay(500);
    Intentos++;
    #ifdef DEBUG
      Serial.print(Intentos);
      Serial.print("*");
    #endif
  }
  if(WiFi.status() != WL_CONNECTED){
    jsonBuffer["error"] = 9;
  }else{
    IPAddress ipSubNet = WiFi.subnetMask();
    IPAddress ipGateway = WiFi.gatewayIP();
    IPAddress ipF = WiFi.localIP();
    String ipStr = String(ipF[0]) + '.' + String(ipF[1]) + '.' + String(ipF[2]) + '.' + String(ipF[3]);
    JsonObject obj = jsonBuffer.createNestedObject("data");
    obj["dirIP"] = ipStr;
    
    if ( ssidRx.length() > 1 )
    {
      for (int i = 0; i < 96; ++i) {
        EEPROM.write(i, 0);
      }
      for (int i = 0; i < ssidRx.length(); ++i)
      {
        EEPROM.write(i, ssidRx[i]);
      }
    }
    if ( passwRx.length() > 1 )
    {
      for (int i = 0; i < passwRx.length(); ++i)
      {
        EEPROM.write(32 + i, passwRx[i]);
      }
      //EEPROM.commit();
    }
    for (int i = 0; i < 4; ++i){
      EEPROM.write(200 + i, ipF[i]);
    }
    for (int i = 0; i < 4; ++i){
      EEPROM.write(204 + i, ipGateway[i]);
    }
    for (int i = 0; i < 4; ++i){
      EEPROM.write(208 + i, ipSubNet[i]);
    }
    EEPROM.commit();
  }
  serializeJson(jsonBuffer, json);
  server.send(200, "application/json", json);
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para enviar el codigo IR recibido
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool ctirJson(String formatIR, unsigned int frecIR, unsigned int longIR, unsigned int repetIR){
  #ifndef DEBUG
    digitalWrite(LedStatus,LED_ON);
  #endif
  irsend.sendRaw(IR_Comand, longIR, frecIR);
  StaticJsonDocument<200> jsonBuffer;
  String json;
  //JsonObject json = jsonBuffer.createNestedObject();
  jsonBuffer["error"] = 0;
  jsonBuffer["deviceid"] = deviceId;
  JsonObject obj = jsonBuffer.createNestedObject("data");
  obj["ifTemp"] = ifTemp;
  obj["ifHume"] = ifHume;
  obj["temp"] = temperatura;
  obj["hume"] = humedad;
  serializeJson(jsonBuffer, json);
  server.send(200, "application/json", json);
  #ifndef DEBUG
    digitalWrite(LedStatus,LED_OFF);
  #endif
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para enviar eror por no recibir Json Valido
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool errorJson(void){
  StaticJsonDocument<200> jsonBuffer;
  String json;
  //JsonObject json = jsonBuffer.createNestedObject();
  jsonBuffer["error"] = 1;
  jsonBuffer["deviceid"] = deviceId;
  serializeJson(jsonBuffer, json);
  server.send(200, "application/json", json);
  
}



///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Leer de la EPROM el SSID y Password
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Leer_EEPROM(void)
{
  String essid;
  int c = 0;
  for (int i = 0; i < 32; ++i)
  {
    if (EEPROM.read(i) == 0 || EEPROM.read(i) == 0xFF)
      i = 32;
    else
      essid += char(EEPROM.read(i));
  }
  ssid = essid;
  //WiFi.softAP(ssid_ap,paswr_ap);
  String epass = "";
  for (int i = 32; i < 96; ++i)
  {
    if (EEPROM.read(i) == 0 || EEPROM.read(i) == 0xFF)
      i = 96;
    else
      epass += char(EEPROM.read(i));
  }
  password = epass;
  if (ssid == ""){
    return (false);
  }
  else{
    return (true);
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Leer de la EPROM la IP, gateway y MaskSubNet
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void erraseEEprom(){
  for (int i = 0; i < EEPROM_SIZE ; i++){
      EEPROM.write(i, 0xFF);
  }
  EEPROM.commit();
}
///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Rutina para Leer de la EPROM la IP, gateway y MaskSubNet
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool Leer_EEPROM_ip(void){
  uint8_t ipFix[4];
  uint8_t ipGat[4];
  uint8_t ipMask[4];
  for (int i = 0; i < 4; ++i)
  {
    ipFix[i] = EEPROM.read(i+200);
  }
  for (int i = 0; i < 4; ++i)
  {
    ipGat[i] = EEPROM.read(i+204);
  }
  for (int i = 0; i < 4; ++i)
  {
    ipMask[i] = EEPROM.read(i+208);
  }
  if(ipFix[0] == 0 || ipFix[0] == 0xFF)
  {
    return (false);
  }
  else{
    for (int i = 0; i < 4; ++i)
    {
      ip[i] = ipFix[i];
    }
    for (int i = 0; i < 4; ++i)
    {
      gateway[i] = ipGat[i];
    }
    for (int i = 0; i < 4; ++i)
    {
      maskSubnet[i] = ipMask[i];
    }
    return(true);
  }
}



///////////////////////////////////////////////////////////////////////////////////////////////////77/////////////////////////
////////////// Parametros del ESP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Param_ESP(void)
{
  ID_ESP = String(ESP.getChipId(), HEX);
  ID_ESP.toUpperCase();
  deviceId = ID_ESP.c_str();
  ssid_ap = ssid_AP;
  ssid_ap += ID_ESP;

//    Serial.print("El Chip ID  es: ");
//    Serial.println(ID_ESP.c_str());
//    Serial.println("");
//    Serial.print("El reset fue por: ");
//    Serial.println(ESP.getResetReason());
//    Serial.print("La Version del Core es: ");
//    Serial.println(ESP.getCoreVersion());
//    Serial.print("La Version del SDK es: ");
//    Serial.println(ESP.getSdkVersion());
//    Serial.print("La Frecuencia del CPU es: ");
//    Serial.println(ESP.getCpuFreqMHz());
//    Serial.print("El Tamaño del Programa es: ");
//    Serial.println(ESP.getSketchSize());
//    Serial.print("El actual Scketch es: ");
//    Serial.println(ESP.getSketchMD5());
//    Serial.print("El ID de la FLASH es: ");
//    Serial.println(ESP.getFlashChipId());
//    Serial.print("La Frecuencia de la FLASH es: ");
//    Serial.println(ESP.getFlashChipSpeed());
}
