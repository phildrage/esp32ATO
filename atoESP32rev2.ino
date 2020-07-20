
//up to date as of 17/7/20
//logic to complete functionality.
  
//  remember to save as intended filename before making changes

#include <WiFi.h> //add libraries
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

const char* ssid = "ASUS";
const char* password = "********";
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
char* action        = "0";   // 0 = waiting, 1 = dumping, 2 = filling
int handAuto        = 1;

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
  // put your setup code here, to run once:
  Serial.begin(115200);
  SerialBT.begin("ESP32ATO"); //Bluetooth device name
  SerialBT.println("The device started, now you can pair it with bluetooth!");
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
 LowState = digitalRead(Lev1);
 HighState = digitalRead(Lev2);
 
 if (handAuto == 1){
   if (LowState == HIGH){
     BeginATO();
 }
 action = "0";
 
                                      //close all valves
                                      
digitalWrite(waterInPin,HIGH);
digitalWrite(dumpPin, HIGH);
digitalWrite(membraneOutPin, HIGH);

                                      //send values to MQTT
                                      
client.publish("ATO/auto","AUTO");
client.publish("ATO/STATE", action);

                                      //send values to bluetooth client
                                      
SerialBT.println("ATO is in Auto");
SerialBT.println("waiting");
SerialBT.print("LOW LEVEL SWITCH - ");
SerialBT.println(LowState);
SerialBT.print("HIGH LEVEL SWITCH - ");
SerialBT.println(HighState);

                                       //send values to serial connection
                                       
Serial.println("ATO is in Auto");
Serial.println("waiting");
Serial.print("LOW LEVEL SWITCH - ");
Serial.println(LowState);
Serial.print("HIGH LEVEL SWITCH - ");
Serial.println(HighState);
}

                                      //Logic if controller is in manual
                                      
if (handAuto == 0){
   Serial.println("ATO is in Hand");
   client.publish("ATO/auto","HAND");
   measureSend();
}
}
}
void measureSend(){

  // grab level switch info
  
 LowState = digitalRead(Lev1);
 HighState = digitalRead(Lev2);

// bluetooth

 SerialBT.print("level switch 1- ");
 SerialBT.println(LowState);
 SerialBT.print("level switch 2- ");
 SerialBT.println(HighState);
 
 if(HighState == HIGH){
client.publish("ATO/auto/feedback/HighState","red");
 }
  if(HighState == LOW){
client.publish("ATO/feedback/HighState","green");
 }
  if(LowState == HIGH){
client.publish("ATO/auto/feedback/LowState","red");
 }
  if(LowState == LOW){
client.publish("ATO/feedback/LowState","green");
 }
 
 // serial
 Serial.print("level switch 1- ");
 Serial.println(LowState);
 Serial.print("level switch 2- ");
 Serial.println(HighState);

                                      //grab temperatures from both sensors
                                      
  DS18B20.requestTemperatures(); 
  temp1 = DS18B20.getTempCByIndex(0); // first temperature sensor
  char buff[100];
  dtostrf(temp1,0,2,buff);
  temp1 = temp1 + 0.5;
  tRound = int(temp1);   //round the temperature to the nearest degree C
  temp2 = DS18B20.getTempCByIndex(1); // first temperature sensor
  dtostrf(temp2,0,2,buff);
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

//bluetooth comms
SerialBT.print("K1 value - ");
SerialBT.println(K1);
SerialBT.print("ADC - ");
SerialBT.println(adcRange);
SerialBT.print("aRef - ");
SerialBT.println(aRef);
SerialBT.println(" ");
SerialBT.print ("analog 1 read value - ");
SerialBT.println ( tds1value);
SerialBT.print ("Volage calculated - ");
SerialBT.println(voltage1);
SerialBT.print("ecvalue - ");
SerialBT.println(ecValue);
SerialBT.print ("Temp1 - ");
SerialBT.println (temp1);
SerialBT.print("ec25 - ");
SerialBT.println(ecValue25);
SerialBT.println(" ");
SerialBT.print("TDS1calc - ");
SerialBT.println (TDS1calc);

//Serial comms

Serial.print("TDS1calc - ");
Serial.println (TDS1calc);
SerialBT.println(" ");

// TDS2 calculation

voltage2 = (tds2value/adcRange)*aRef;
ecValue = (((133.42*pow(voltage2,3)))-((255.86*pow(voltage2,2)))+(857.39*voltage2))*K2;
ecValue25 = ecValue/(1+(0.02*(temp2-25)));
TDS2calc = ecValue25*TDSfactor;

// Bluetooth comms

SerialBT.print("K2 value - ");
SerialBT.println(K2);
SerialBT.print("ADC - ");
SerialBT.println(adcRange);
SerialBT.print("aRef - ");
SerialBT.println(aRef);
SerialBT.println(" ");
SerialBT.print ("analog 2 read value - ");
SerialBT.println ( tds2value);
SerialBT.print ("Volage 2 calculated - ");
SerialBT.println(voltage2);
SerialBT.print("ecvalue - ");
SerialBT.println(ecValue);
SerialBT.print ("Temp2 - ");
SerialBT.println (temp2);
SerialBT.print("ec25 - ");
SerialBT.println(ecValue25);
SerialBT.println(" ");
SerialBT.print("TDS2calc - ");
SerialBT.println (TDS2calc);

//Serial comms

Serial.print("TDS2calc - ");
Serial.println (TDS2calc);


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
  SerialBT.println(" ");
  SerialBT.print("membrane TDS limit - ");
  SerialBT.println(tdsLimitM);
  SerialBT.print("DI TDS limit - ");
  SerialBT.println(tdsLimitDI);
  SerialBT.println(" ");

// warning if DI resin no good

if (TDS2calc >= tdsLimitDI){
SerialBT.println("Output TDS high warning") ;
client.publish("ATO/WARNING","HIGH OPUTPUT TDS");
   }
}

void callback(char* topic, byte* payload, unsigned int length) {
  SerialBT.print("Message arrived [");
  SerialBT.print(topic);
  SerialBT.println("] ");
  for (int i = 0; i < length; i++) {
    SerialBT.print(char(payload[i]));
  }
 
  SerialBT.println();
//  payload[length];
//  String s = String((char*)payload);
//  SerialBT.println ("String payload - ");
//  SerialBT.println (s);
  float load = atof((char*)payload);
  SerialBT.println ("Float payload - ");
  SerialBT.println (load);

//this is broken.fix this bit
  
// if(strcmp(topic, "ATO/Cal/1") == 0){
//  float raw = (load/TDSfactor)*(1+(0.02*(temp1-25)));
//  float cal1 = (raw/(133.42*(pow(voltage1,3)))-(255.86*(pow(voltage1,2)))+(857.39*voltage1));
//EEPROM.writeFloat (20, cal1);
//EEPROM.commit;
//SerialBT.println(EEPROM.readFloat(20));

//  }
 if(strcmp(topic, "ATO/Cal/2") == 0){
  EEPROM.write(5,load);
  SerialBT.println ("eeprom saved ");
  SerialBT.println (load);
  EEPROM.commit();
  }
   if(strcmp(topic, "ATO/ADC") == 0){
  EEPROM.writeFloat(10,load);
  EEPROM.commit();
  SerialBT.println ("eeprom saved ");
  adcRange =  EEPROM.readFloat(10);
  SerialBT.println (EEPROM.readFloat(10));
  }
   if(strcmp(topic, "ATO/aRef") == 0){
  EEPROM.writeFloat(15,load);
  EEPROM.commit();
  SerialBT.println ("eeprom saved ");
  aRef =  EEPROM.readFloat(15);
  SerialBT.println (EEPROM.readFloat(15));
  }
     if(strcmp(topic, "ATO/K1") == 0){
  //  load=load/10;
  EEPROM.writeFloat(20,load);
  EEPROM.commit();
  SerialBT.println ("K1 saved to eeprom");
  Serial.print ("K1 saved to eeprom - ");
  Serial.println (load);
  K1 =  EEPROM.readFloat(20);
  SerialBT.println (EEPROM.readFloat(20));
  }
       if(strcmp(topic, "ATO/K2") == 0){
  EEPROM.writeFloat(25,load);
  EEPROM.commit();
  SerialBT.println ("K2 saved to eeprom");
  Serial.print ("K2 saved to eeprom - ");
  Serial.println (load);
  K2 =  EEPROM.readFloat(25);
  SerialBT.println (EEPROM.readFloat(25));
  }
         if(strcmp(topic, "ATO/tdsLimitM") == 0){
  EEPROM.writeFloat(30,load);
  EEPROM.commit();
  SerialBT.println ("TDS limit for membrane output saved ");
  tdsLimitM =  EEPROM.readFloat(30);
  SerialBT.println (EEPROM.readFloat(30));
  }
           if(strcmp(topic, "ATO/tdsLimitDI") == 0){
  EEPROM.writeFloat(35,load);
  EEPROM.commit();
  SerialBT.println ("TDS limit for DI output saved ");
  tdsLimitDI =  EEPROM.readFloat(35);
  SerialBT.println (EEPROM.readFloat(35));
  }

   if(strcmp(topic, "ATO/handAuto") == 0){
handAuto = load;
Serial.println(load);
   }
                                                          //Manual mode
if (handAuto == 0){
  if(strcmp(topic, "ATO/relays/waterIn") == 0){
    if (load == 1){
    digitalWrite(waterInPin,HIGH);
    Serial.println("Water in relay - OFF");
    client.publish("ATO/auto/feedback/waterIn","green");  
    }
    if (load == 0){
    digitalWrite(waterInPin,LOW);
    Serial.println("Water in relay - ON");
    client.publish("ATO/auto/feedback/waterIn","red");  
    }
  }
  if(strcmp(topic, "ATO/relays/dump") == 0){
    if (load == 1){
    digitalWrite(dumpPin,HIGH);
    Serial.println("dump relay - OFF");
    client.publish("ATO/auto/feedback/dump","green");  
    }
    if (load == 0){
    digitalWrite(dumpPin,LOW);
    Serial.println("dump relay - ON");
    client.publish("ATO/auto/feedback/dump","red");  
    }
  }

  if(strcmp(topic, "ATO/relays/membraneOut") == 0){
    if (load == 1){
    digitalWrite(membraneOutPin,HIGH);
    Serial.println("membrane out relay - OFF");
    client.publish("ATO/auto/feedback/membrane","green");  
    }
    if (load == 0){
    digitalWrite(membraneOutPin,LOW);
    Serial.println("membrane out relay - ON");  
    client.publish("ATO/auto/feedback/membrane","red");
    }
  }
}
}

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  SerialBT.println();
  SerialBT.print("Connecting to ");
  SerialBT.println(ssid);
  Serial.print("Connecting to ");
  Serial.println(ssid);


  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    SerialBT.print(".");
    Serial.print(".");
  }

  randomSeed(micros());

  SerialBT.println("");
  SerialBT.println("WiFi connected");
  SerialBT.println("IP address: ");
  SerialBT.println(WiFi.localIP());
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    SerialBT.print("Attempting MQTT connection...");
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    
    if (client.connect("12345")) {
      SerialBT.println("connected");
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("AliveRegister", "0");   // change depending on device
      // ... and resubscribe// change to subscribed topics
      client.subscribe("ATO/handAuto");
      client.subscribe("ATO/relays/waterIn");  
      client.subscribe("ATO/relays/dump");
      client.subscribe("ATO/relays/membraneOut");  
      client.subscribe("ATO/Cal/1");  
      client.subscribe("ATO/Cal/2");  
      client.subscribe("ATO/aRef");  
      client.subscribe("ATO/ADC"); 
      client.subscribe("ATO/K1"); 
      client.subscribe("ATO/K2");
      client.subscribe("ATO/tdsLimitM");  
      client.subscribe("ATO/tdsLimitDI");   
    } else {
      SerialBT.print("failed, rc=");
      SerialBT.print(client.state());
      SerialBT.println(" try again in 5 seconds");
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
void BeginATO(){
SerialBT.println("ATO engaged");
while (HighState == HIGH){
digitalWrite(waterInPin,LOW);
digitalWrite(membraneOutPin, LOW);
measureSend();
while (TDS2calc >= tdsLimitM){
digitalWrite(dumpPin,LOW);
action = "1";
delay(5000);
client.publish("ATO/STATE", action);
Serial.println("Dumping");
SerialBT.println("Dumping");
delay(5000);
measureSend();
  }
digitalWrite(dumpPin,HIGH);
Serial.println("Filling");
SerialBT.println("Filling") ;
action = "2";
client.publish("ATO/STATE", action);
delay(5000);
}
digitalWrite(waterInPin,HIGH);
digitalWrite(dumpPin,HIGH);
digitalWrite(membraneOutPin,HIGH);
Serial.println("ATO complete");
SerialBT.println("ATO Complete") ;
action = "0";
client.publish("ATO/STATE", action);
delay(5000);
}
