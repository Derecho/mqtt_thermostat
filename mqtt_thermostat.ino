// vim: syntax=c:autoindent:sw=2:ts=2

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include <OpenTherm.h>

#include "mqtt_config.h"

const unsigned long OT_INTERVAL = 1000;  // 1 second
const unsigned long RECONNECT_INTERVAL = 5000;  // 5 seconds

//OpenTherm input and output wires connected to 4 and 5 pins on the OpenTherm Shield
const int inPin = 4;
const int outPin = 5;

//Data wire is connected to 14 pin on the OpenTherm Shield
#define ONE_WIRE_BUS 14

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
OpenTherm ot(inPin, outPin);
WiFiClient espClient;
PubSubClient client(espClient);
char buf[10];

float sp = 23, //set point
pv = 0, //current temperature
pv_last = 0, //prior temperature
ierr = 0, //integral error
dt = 0, //time between measurements
op = 0; //PID controller output
unsigned long ts = 0, new_ts = 0, disconnected_ts = 0; //timestamp


void ICACHE_RAM_ATTR handleInterrupt() {
  ot.handleInterrupt();
}

float getTemp() {
  return sensors.getTempCByIndex(0);
}

float pid(float sp, float pv, float pv_last, float& ierr, float dt) {
  float Kc = 10.0; // K / %Heater
  float tauI = 50.0; // sec
  float tauD = 1.0;  // sec
  // PID coefficients
  float KP = Kc;
  float KI = Kc / tauI;
  float KD = Kc*tauD; 
  // upper and lower bounds on heater level
  float ophi = 100;
  float oplo = 0;
  // calculate the error
  float error = sp - pv;
  // calculate the integral error
  ierr = ierr + KI * error * dt;  
  // calculate the measurement derivative
  float dpv = (pv - pv_last) / dt;
  // calculate the PID output
  float P = KP * error; //proportional contribution
  float I = ierr; //integral contribution
  float D = -KD * dpv; //derivative contribution
  float op = P + I + D;
  // implement anti-reset windup
  if ((op < oplo) || (op > ophi)) {
    I = I - KI * error * dt;
    // clip output
    op = max(oplo, min(ophi, op));
  }
  ierr = I; 
  Serial.println("sp="+String(sp) + " pv=" + String(pv) + " dt=" + String(dt) + " op=" + String(op) + " P=" + String(P) + " I=" + String(I) + " D=" + String(D));
  return op;
}

void setup_wifi() {
  delay(10);
  //Connect to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup(void) {  
  Serial.begin(115200);
  setup_wifi();

  //Init DS18B20 Sensor
  sensors.begin();
  sensors.requestTemperatures();
  sensors.setWaitForConversion(false); //switch to async mode
  pv, pv_last = sensors.getTempCByIndex(0);
  ts = millis();

  //Init OpenTherm Controller
  ot.begin(handleInterrupt);

  //Init MQTT Client
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void publish_temperature() {
  String data;
  data += "{\"pv\": \"";
  data += pv;
  data += "\", \"sp\": \"";
  data += sp;
  data += "\"}";
  Serial.println(data);
  char data_char[64];
  data.toCharArray(data_char, sizeof(data_char));
  client.publish(mqtt_topic_update, data_char);
}

void callback(char* topic, byte* payload, unsigned int length) {
  if(strcmp(topic, mqtt_topic_sp) != 0) return;
  String str = String();    
  for (int i = 0; i < length; i++) {
    str += (char)payload[i];
  }
  Serial.println("sp=" + str);  
  sp = str.toFloat();
}

void reconnect() {  
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
#ifdef MQTT_AUTH
    if (client.connect(mqtt_id, mqtt_user, mqtt_password)) {
#else
    if (client.connect(mqtt_id)) {
#endif
      Serial.println("connected");
      // Once connected, publish an announcement...
      publish_temperature();
      // ... and resubscribe
      client.subscribe(mqtt_topic_sp);
    } else {
      Serial.print("failed, rc=");
      Serial.println(client.state());
    }
}

void loop(void) { 
  new_ts = millis();
  if (new_ts - ts > OT_INTERVAL) {   
    //Set/Get Boiler Status
    bool enableCentralHeating = true;
    bool enableHotWater = true;
    bool enableCooling = false;
    unsigned long response = ot.setBoilerStatus(enableCentralHeating, enableHotWater, enableCooling);
    OpenThermResponseStatus responseStatus = ot.getLastResponseStatus();
    if (responseStatus != OpenThermResponseStatus::SUCCESS) {
      Serial.println("Error: Invalid boiler response " + String(response, HEX));
    }   

    pv = getTemp();
    dt = (new_ts - ts) / (float)OT_INTERVAL;
    ts = new_ts;
    if (responseStatus == OpenThermResponseStatus::SUCCESS) {
      op = pid(sp, pv, pv_last, ierr, dt);
      //Set Boiler Temperature
      ot.setBoilerTemperature(op);
    }
    pv_last = pv;
    
    sensors.requestTemperatures(); //async temperature request
    
    publish_temperature();
  }
  
  //MQTT Loop
  if (!client.connected()) {
    if (disconnected_ts == 0) {
      // Just got disconnected, save timestamp
      disconnected_ts = new_ts;
    }
    else if(new_ts - disconnected_ts > RECONNECT_INTERVAL) {
      // RECONNECT_INTERVAL passed since disconnection, try to reconnect
      reconnect();
      disconnected_ts = 0;  // Reset after connection attempt
    }
  }
  client.loop();
}

