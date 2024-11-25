#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <ArduinoJson.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

const int firepin = D5;  // Fire sensor pin
const int gaspin = D2;   // Gas sensor pin 
const int outpin = D6;   // Alert output pin

Scheduler myScheduler;
painlessMesh mesh;
uint32_t gatewayNodeId = 0;  // Gateway node ID
bool manualReset = false;    // Manual reset flag

void sendData();
void readSensor();
void checkConnections();

Task taskSendData(TASK_SECOND * 2, TASK_FOREVER, &sendData);
Task taskReadSensors(TASK_MILLISECOND * 100, TASK_FOREVER, &readSensor);
Task taskCheckConnections(TASK_SECOND * 5, TASK_FOREVER, &checkConnections);

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP_STA);    // Required for painlessMesh
  pinMode(firepin, INPUT);
  pinMode(gaspin, INPUT);
  pinMode(outpin, OUTPUT);
  
  // Initialize mesh network
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &myScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);

  myScheduler.addTask(taskSendData);
  myScheduler.addTask(taskReadSensors);
  myScheduler.addTask(taskCheckConnections);

  taskSendData.enable();
  taskReadSensors.enable();
  taskCheckConnections.enable();
}

void loop() {
  mesh.update();
  myScheduler.execute();
}

void sendData() {
  if (gatewayNodeId != 0) {
    short firesensor = digitalRead(firepin);
    short gassensor = digitalRead(gaspin);

    StaticJsonDocument<200> jsonDat;
    jsonDat["room"] = 1;
    jsonDat["firesensor"] = firesensor;
    jsonDat["gassensor"] = gassensor;

    String jsonString;
    serializeJson(jsonDat, jsonString);
    mesh.sendSingle(gatewayNodeId, jsonString);

    Serial.println("Data sent: " + jsonString);
  }
}

void readSensor() {
  if (!manualReset) {
    if (digitalRead(firepin) == HIGH || digitalRead(gaspin) == HIGH) {
      digitalWrite(outpin, HIGH);
    } else {
      digitalWrite(outpin, LOW);
    }
  }
}

void checkConnections() {
  auto nodes = mesh.getNodeList();  // Get a list of connected nodes
  if (nodes.size() == 0) {
    Serial.println("No active nodes found in the mesh network.");
  } else {
    Serial.printf("Active nodes: %u\n", nodes.size());
  }
}

void receivedCallback(uint32_t from, String &msg) {
  if (msg.startsWith("GatewayID:")) {
    gatewayNodeId = msg.substring(10).toInt();
    Serial.printf("Gateway node ID: %u\n", gatewayNodeId);
  } else if (msg == "Reset") {
    manualReset = true;
    digitalWrite(outpin, LOW);
    Serial.println("Manual reset activated.");
  } else if (msg == "ClearReset") {
    manualReset = false;
    Serial.println("Manual reset cleared.");
  }
}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New Connection: NodeID = %u\n", nodeId);
}

void changedConnectionCallback() {
  Serial.println("Mesh connections changed.");
}
