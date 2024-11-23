//*******************************
//This is the implementation of the nodes for communicating and exchanging info with each other.
//*******************************


//importing painlessMesh library and defining passwords and port for mesh networking
#include "painlessMesh.h"
#include <ESP8266WiFi.h>    //For esp8266 wifi
#include <ArduinoOTA.h>     //For OTA support
#include <ArduinoJson.h>    //For JSON message

const char* ssid = "GODGIFTED";  // Change to your WiFi Network name
const char* password = "ssgrp1234";

#define  MESH_PREFIX    "Fire_Gas_Detector"
#define  MESH_PASSWORD  "saveslifefromfire"
#define  MESH_PORT      5555

const int firepin = 2;
const int gaspin = A0;
const int outpin = 4;

// Adding Scheduler for personal tasks
Scheduler myScheduler;
painlessMesh mesh;

// Variable to store gateway node ID
uint32_t gatewayNodeId = 0;

// Creating tasks for scheduled duration

void sendData();
void readSensor();

Task taskSendData(TASK_SECOND * 1 , TASK_FOREVER, &sendData);
Task taskReadSensors(TASK_MILLISECOND * 100 , TASK_FOREVER, &readSensor);

// Setting the task which includes reading sensors
void sendData(){
  if (gatewayNodeId != 0) {
    //Read IR sensor
    short firesensor = digitalRead(firepin);

    //Read Gas sensor
    short gassensor = analogRead(outpin);

    //Create JSON object
    StaticJsonDocument<200> jsonDat;
    jsonDat["firesensor"] = firesensor;
    jsonDat["gassensor"] = gassensor;

    //Serializing JSON to string
    String jsonString;
    serializeJson(jsonDat, jsonString);

    // Send data to gateway node
    mesh.sendSingle(gatewayNodeId, jsonString);
    taskSendData.setInterval( random( TASK_SECOND * 1, TASK_SECOND * 3 ));
  }
}

//Setting the task for reading sensors
void readSensor(){
  //For turning buzzer and led
  if(digitalRead(firepin) == HIGH || analogRead(gaspin) > 200){
    digitalWrite(outpin, HIGH); 
  }
}

//To receive reset signal
void receivedCallback(uint32_t from, String &msg) { 
  if (msg.startsWith("GatewayID:")) { 
    gatewayNodeId = msg.substring(10).toInt(); 
    Serial.printf("Gateway node ID received: %u\n", gatewayNodeId);
  }

  else if(msg == "Reset"){
    Serial.println("Reset button triggered.");
    digitalWrite(outpin, LOW);
  }

  else{
    // Do nothing.
  }; 
}

//To check new connection
void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

//To see change in connection
void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void setup(){
  Serial.begin(115200);

  // Connects to WiFi
  OTAhandle();
  //Initialise mesh network and user
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &myScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);

  myScheduler.addTask(taskSendData);
  myScheduler.addTask(taskReadSensors);
  taskSendData.enable();

}

void loop(){
  ArduinoOTA.handle();    // Handles a code update request
  mesh.update();          //Update the mesh network
  myScheduler.execute();  // Executes the task as per the user
}

void OTAhandle(){
  WiFi.begin(ssid, password);

  // Ensure WiFi is connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  ArduinoOTA.begin();  
}