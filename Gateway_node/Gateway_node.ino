#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL386asCXQ1"
#define BLYNK_TEMPLATE_NAME "New Test"
#define BLYNK_AUTH_TOKEN "BB9ncucYKEf1jreyQlk0oyyJLzYLl8Va"

#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoJson.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

#define WIFI_SSID "GODGIFTED"
#define WIFI_PASSWORD "ssgrp1234"

#define RESET_VPIN V1
#define CLEAR_RESET_VPIN V2

const short room = 2;      //Assign room number for the room you want the sensor to be installed

const short firepin = D5;  // Fire sensor pin
const short gaspin = D2;   // Gas sensor pin
const short outpin = D6;   // Alert output pin

bool manualReset = false;    // Manual reset flag

Scheduler userScheduler;
painlessMesh mesh;

uint32_t nodeId;
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 5; // Check every 5 seconds

std::vector<uint32_t> nodeList;    // List of connected nodes

// Variables to track button states
bool resetTriggered = false;
bool clearResetTriggered = false;

void readSensor();
void checkConnections(); 

Task taskReadSensors(TASK_MILLISECOND * 100, TASK_FOREVER, &readSensor);
Task taskCheckConnections(TASK_SECOND * 5, TASK_FOREVER, &checkConnections);
Task taskSendMessage(TASK_MILLISECOND * 50, TASK_FOREVER, []() {
  if (resetTriggered) {
    // Broadcast "RESET" to the mesh network
    mesh.sendBroadcast("RESET");
    Serial.println("Broadcasting RESET");
    resetTriggered = false; // Reset flag
  } 
  else if (clearResetTriggered) {
    // Broadcast "CLEAR_RESET" to the mesh network
    mesh.sendBroadcast("CLEAR_RESET");
    Serial.println("Broadcasting CLEAR_RESET");
    clearResetTriggered = false; // Reset flag
  }
});

BLYNK_WRITE(RESET_VPIN) {
  short buttonState = param.asInt();  // Read button value (0 or 1)
  if (buttonState == 1) {           // Button pressed
    resetTriggered = true;          // Set flag to send RESET
    manualReset = true;
    digitalWrite(outpin, LOW);
    Serial.println("Manual reset activated.");
  }
  Blynk.virtualWrite(RESET_VPIN, 0);
}

BLYNK_WRITE(CLEAR_RESET_VPIN) {
  short buttonState = param.asInt();  // Read button value (0 or 1)
  if (buttonState == 1) {             // Button pressed
    clearResetTriggered = true;       // Set flag to send CLEAR_RESET
    manualReset = false;
    Serial.println("Manual reset cleared.");
  }
  Blynk.virtualWrite(CLEAR_RESET_VPIN, 0);
}

void checkConnections() {
    auto nodes = mesh.getNodeList();  // Get a list of connected nodes
    if (nodes.size() == 0) {
      Serial.println("No active nodes found in the mesh network.");
    } else {
      Serial.printf("Active nodes: %u\n", nodes.size());
    }
  }

void readSensor() {
    if (!manualReset) {
      if (digitalRead(firepin) == HIGH || digitalRead(gaspin) == HIGH) {
        digitalWrite(outpin, HIGH);
      } 
      else {
        digitalWrite(outpin, LOW);
      }
    } else {
      digitalWrite(outpin, LOW);
    }
  }

void receivedCallback( uint32_t from, String &msg ) {
  // Parse the JSON data
  StaticJsonDocument<50> doc;

  DeserializationError error = deserializeJson(doc, msg);
  
  if (!error){
    short room = doc["room"];
    bool fireSensor = doc["firesensor"];
    bool gasSensor = doc["gassensor"];

    Serial.printf("Room %d, Fire: %d, Gas: %d\n", room, fireSensor, gasSensor);

    //Handles JSON and Blynk
    handleReceivedData(from, msg);
  }
  else{
    Serial.println("Received no json data.");
  }

}
void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection: NodeID = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(),offset);
}

void handleReceivedData(uint32_t from, String &msg){
  //Parse the JSON data
  StaticJsonDocument<100> doc;
  DeserializationError error = deserializeJson(doc, msg);
  if (error) {
    Serial.print("Failed to parse JSON: ");
    Serial.println(error.c_str());
    return;
  }

  if (doc.containsKey("room") && doc.containsKey("firesensor") && doc.containsKey("gassensor")) {
    short room = doc["room"];
    short firesensor = doc["firesensor"];
    short gassensor = doc["gassensor"];

    // Send data to Blynk
    if (room == 1) {
      Blynk.virtualWrite(V1, firesensor); // Fire status to Virtual Pin V1
      Blynk.virtualWrite(V2, gassensor); // Gas status to Virtual Pin V2
    } else if (room == 2) {
      Blynk.virtualWrite(V3, firesensor); // Fire status to Virtual Pin V3
      Blynk.virtualWrite(V4, gassensor); // Gas status to Virtual Pin V4
    }

    Serial.printf("Sent to Blynk -> room: %d, firesensor: %d, gassensor: %d\n", room, firesensor, gassensor);
  } else {
    Serial.println("JSON does not contain expected keys.");
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  pinMode(outpin, OUTPUT);

  // Initialize mesh network
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

  nodeId = mesh.getNodeId();
  Serial.printf("Node ID: %u\n", nodeId);

  // Connect to WiFi with timeout
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if (millis() - startTime > 20000) {
      Serial.println("Failed to connect to WiFi!");
      break;
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected to WiFi");
  }

  // Initialize Blynk without blocking
  Blynk.config(BLYNK_AUTH_TOKEN);
  while (!Blynk.connect()) {
    delay(1000);
    Serial.println("Connecting to Blynk...");
  }
  Serial.println("Connected to Blynk");

  // Add and enable tasks
  userScheduler.addTask(taskSendMessage);
  userScheduler.addTask(taskReadSensors);
  userScheduler.addTask(taskCheckConnections);
  
  taskSendMessage.enable();
  taskReadSensors.enable();
  taskCheckConnections.enable();
}

void loop(){
  mesh.update();
  Blynk.run();
  userScheduler.execute();
}