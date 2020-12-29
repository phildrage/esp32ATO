
//up to date as of 17/7/20
//logic to complete functionality.
  
//  remember to save as intended filename before making changes

#include <WiFi.h> //add libraries
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>


const char* ssid = "ASUS";
const char* password = "9KU62NP9";
IPAddress server(192, 168, 1, 90);

#define ONE_WIRE_BUS 4     //  D4 on physical board
#define EEPROM_SIZE 500
#define waterInPin 25
#define membraneOutPin 26
#define dumpPin 33
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);
float prevTemp = 0;

//IO assignments
int TDS1            = 34;
int TDS2            = 35;
int tds1value       = 666;
int tds2value       = 666;
int Lev1            = 14;
int Lev2            = 13;
int LowState        = 0;
int HighState       = 0;
int action        = 0;   // 0 = waiting, 1 = dumping, 2 = filling
int levelState    = 0;  // 0 = low level, 1 = mid, 2 = high
char mqttAction[1];
char mqtt[4];
int handAuto        = 1;     // 1= auto, 0 = manual

float tdsLimitM;
float tdsLimitDI;
float aRef;
float adcRange;
float K1            = 1;
float K2            = 1;
double ecValue      = 0;
double ecValue25    = 0;
float TDSfactor     = 0.5;
double voltage1     = 0;
double voltage2     = 0;
double TDS1calc;
double TDS2calc;
double temp1;
double temp2;
float tRound;
WiFiClient espClient;
PubSubClient client(espClient);
const char* willTopic             = "LWT/ATO";
const char* willMessage           = "ATO Power failure";
boolean willRetain                = true;
byte willQos                      = 0;

unsigned long previousMillis      = 0; 
const long interval               = 5000;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  EEPROM.begin(EEPROM_SIZE);
  setup_wifi();
  client.setServer(server, 1883);
  client.setCallback(callback);  
  pinMode(Lev1,INPUT);
  pinMode (waterInPin,OUTPUT);
pinMode (membraneOutPin,OUTPUT);
pinMode (dumpPin,OUTPUT);
aRef = EEPROM.readFloat(15);
adcRange = EEPROM.readFloat(10);
K1 = EEPROM.readFloat(20);
K2 = EEPROM.readFloat(25);
tdsLimitM = EEPROM.readFloat(30);
tdsLimitDI = EEPROM.readFloat(35);
Serial.println(ESP.getFreeHeap());

}

void loop() {
  // put your main code here, to run repeatedly:
  if (!client.connected()) {
    reconnect();
  }
  client.loop();  
 
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time 
    previousMillis = currentMillis;
    itoa (ESP.getFreeHeap(),mqtt,10);
    client.publish("ATO/heap",mqtt);
 if (handAuto == 1){
  client.publish("ATO/auto","AUTO");
  WaterLevels();
  measureSend();
   if (levelState == 0){    // low level
     BeginATO();            // start filling routine
 }
 action = 0;
 itoa (action,mqttAction,10);
client.publish("ATO/STATE", mqttAction);
                                      //close all valves
                                      
digitalWrite(waterInPin,HIGH);
digitalWrite(dumpPin, HIGH);
digitalWrite(membraneOutPin, HIGH);

client.publish("ATO/auto/feedback/waterIn","green");
client.publish("ATO/auto/feedback/dump","green");
client.publish("ATO/auto/feedback/membrane","green");
                                   
 }
                                      //Logic if controller is in manual
                                      
if (handAuto == 0){

   client.publish("ATO/auto","HAND");
   Serial.println("hand");
   WaterLevels();
   measureSend();
}
}
}

void measureSend(){
                                      //grab temperatures from both sensors
                                      
  DS18B20.requestTemperatures(); 
  temp1 = DS18B20.getTempCByIndex(0); // first temperature sensor
  //char buff[100];
  //dtostrf(temp1,0,2,buff);
  temp1 = temp1 + 0.5;
  tRound = int(temp1);   //round the temperature to the nearest degree C
  temp2 = DS18B20.getTempCByIndex(1); // first temperature sensor
  //dtostrf(temp2,0,2,buff);
  temp2 = temp2 + 0.5;
  tRound = int(temp2);   //round the temperature to the nearest degree C
 //long now = millis();

 //publish to MQTT
 
  char pubTemp1[100];
  dtostrf(temp1,2,2,pubTemp1);
  client.publish("ATO/Temp1",pubTemp1);
   char pubTemp2[100];
  dtostrf(temp2,2,2,pubTemp2);
  client.publish("ATO/Temp2",pubTemp2);

 // grab analog values for TDS
 tds1value = analogRead(TDS1);
 tds2value = analogRead(TDS2);

 // wierd calculation of TDS based on maths ripped from the library. putting it here
 // lets me manipulate it more easily for calibration, and allows multiple TDS meters

 //TDS1 calculation
 
voltage1 = (tds1value/adcRange)*aRef;
ecValue = (((133.42*pow(voltage1,3)))-((255.86*pow(voltage1,2)))+(857.39*voltage1))*K1;
ecValue25 = ecValue/(1+(0.02*(temp1-25)));
TDS1calc = ecValue25*TDSfactor;

// TDS2 calculation

voltage2 = (tds2value/adcRange)*aRef;
ecValue = (((133.42*pow(voltage2,3)))-((255.86*pow(voltage2,2)))+(857.39*voltage2))*K2;
ecValue25 = ecValue/(1+(0.02*(temp2-25)));
TDS2calc = ecValue25*TDSfactor;


//MQTT publish

  char pubTDS1[100];
  dtostrf(TDS1calc,2,2,pubTDS1);
  client.publish("ATO/TDS1",pubTDS1);


  char pubTDS2[100];
  dtostrf(TDS2calc,2,2,pubTDS2);
  client.publish("ATO/TDS2",pubTDS2);



  char publev1[100];
  dtostrf(Lev1,2,2,publev1);
  client.publish("ATO/Lev1",publev1);
  char publev2[100];
  dtostrf(Lev2,2,2,publev2);
  client.publish("ATO/Lev2",publev2);


// warning if DI resin no good

if (TDS2calc >= tdsLimitDI){

client.publish("ATO/WARNING","HIGH OPUTPUT TDS");
   }
}

void callback(char* topic, byte* payload, unsigned int length) {

  for (int i = 0; i < length; i++) {
  }

  float load = atof((char*)payload);

//  }
 if(strcmp(topic, "ATO/Cal/2") == 0){
  EEPROM.write(5,load);
  EEPROM.commit();
  }
   if(strcmp(topic, "ATO/ADC") == 0){
  EEPROM.writeFloat(10,load);
  EEPROM.commit();
  adcRange =  EEPROM.readFloat(10);
  }
   if(strcmp(topic, "ATO/aRef") == 0){
  EEPROM.writeFloat(15,load);
  EEPROM.commit();
  aRef =  EEPROM.readFloat(15);
  }
     if(strcmp(topic, "ATO/K1") == 0){
  //  load=load/10;
  EEPROM.writeFloat(20,load);
  EEPROM.commit();
  K1 =  EEPROM.readFloat(20);
  }
       if(strcmp(topic, "ATO/K2") == 0){
  EEPROM.writeFloat(25,load);
  EEPROM.commit();
  K2 =  EEPROM.readFloat(25);
  }
         if(strcmp(topic, "ATO/tdsLimitM") == 0){
  EEPROM.writeFloat(30,load);
  EEPROM.commit();
  tdsLimitM =  EEPROM.readFloat(30);
  }
           if(strcmp(topic, "ATO/tdsLimitDI") == 0){
  EEPROM.writeFloat(35,load);
  EEPROM.commit();
  tdsLimitDI =  EEPROM.readFloat(35);
  }

   if(strcmp(topic, "ATO/handAuto") == 0){
handAuto = load;

   }
                                                          //Manual mode
if (handAuto == 0){
  if(strcmp(topic, "ATO/relays/Dumping") == 0){
    Serial.print("dump");
    client.publish("ATO/auto/feedback/man","dump acknowledge");
    digitalWrite(waterInPin,LOW);
    digitalWrite(dumpPin,LOW);
    client.publish("ATO/auto/feedback/waterIn","red");
    client.publish("ATO/auto/feedback/dump","red");  
  }
  if(strcmp(topic, "ATO/relays/Filling") == 0){
    client.publish("ATO/auto/feedback/man","fill acknowledge");
    digitalWrite(waterInPin,LOW);
    digitalWrite(membraneOutPin,LOW);
    client.publish("ATO/auto/feedback/waterIn","red");
    client.publish("ATO/auto/feedback/membrane","red");
  }

  if(strcmp(topic, "ATO/relays/Waiting") == 0){
    client.publish("ATO/auto/feedback/man","wait acknowledge");
    digitalWrite(waterInPin,HIGH);
    digitalWrite(dumpPin,HIGH);
    digitalWrite(membraneOutPin,HIGH);
    client.publish("ATO/auto/feedback/waterIn","green");
    client.publish("ATO/auto/feedback/dump","green");
    client.publish("ATO/auto/feedback/membrane","green");
  }
}
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  randomSeed(micros());
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    // Create a random client ID
    String clientId = "ESP8266Client";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    
    if (client.connect("12345")) {

      // Once connected, publish an announcement...
      client.publish("AliveRegister", "0");   // change depending on device
      // ... and resubscribe// change to subscribed topics
      client.subscribe("ATO/handAuto");
      client.subscribe("ATO/relays/Waiting");  
      client.subscribe("ATO/relays/Dumping");
      client.subscribe("ATO/relays/Filling");  
      client.subscribe("ATO/Cal/1");  
      client.subscribe("ATO/Cal/2");  
      client.subscribe("ATO/aRef");  
      client.subscribe("ATO/ADC"); 
      client.subscribe("ATO/K1"); 
      client.subscribe("ATO/K2");
      client.subscribe("ATO/tdsLimitM");  
      client.subscribe("ATO/tdsLimitDI");   
    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void BeginATO(){

digitalWrite(waterInPin,LOW); // open supply valve
client.publish("ATO/auto/feedback/waterIn","red");
digitalWrite(dumpPin, LOW);   // open dump valve
client.publish("ATO/auto/feedback/dump","red");
action = 1;
itoa (action,mqttAction,10);
client.publish("ATO/STATE", mqttAction);
delay(5000);
measureSend();
if (TDS2calc >= tdsLimitM){
  BeginATO();
}

while (levelState < 2){     // loop until high level switch is active
WaterLevels();
measureSend();
if (TDS2calc >= tdsLimitM){
  BeginATO();
}
digitalWrite(waterInPin,LOW);   // open supply valve
client.publish("ATO/auto/feedback/waterIn","red");
digitalWrite(dumpPin, HIGH);   // close dump valve
client.publish("ATO/auto/feedback/dump","green");
digitalWrite(membraneOutPin, LOW); // open feed valve
client.publish("ATO/auto/feedback/membrane","red");
action = 2;
itoa (action,mqttAction,10);
client.publish("ATO/STATE", mqttAction);
delay(5000);
}

}

void WaterLevels(){
  
  // grab level switch info
  
 LowState = digitalRead(Lev1);
 HighState = digitalRead(Lev2);

if (HighState == HIGH){     // high level out of water
  if (LowState == HIGH){    //  low level out of water
    levelState = 0;
  }
}
if (HighState == HIGH){     // high level out of water
  if (LowState == LOW){    //  low level under water
    levelState = 1;
  }
}
if (HighState == LOW){     // high level under water
  if (LowState == LOW){    //  low level under water
    levelState = 2;
  }
}
  char pubLev[100];
  dtostrf(levelState,2,2,pubLev);
client.publish("ATO/auto/feedback/levelState",pubLev);


 if(HighState == HIGH){
client.publish("ATO/auto/feedback/HighState","red");
 }
  if(HighState == LOW){
client.publish("ATO/auto/feedback/HighState","green");
 }
  if(LowState == HIGH){
client.publish("ATO/auto/feedback/LowState","red");
 }
  if(LowState == LOW){
client.publish("ATO/auto/feedback/LowState","green");
 }
}
