#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL3AuGYNuFp"
#define BLYNK_TEMPLATE_NAME "Node Test"
#define BLYNK_AUTH_TOKEN "mkI3iO_k8aWNfvEh447bDvvKG4CEa5tJ"

#include "painlessMesh.h"
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <ArduinoJson.h>

#define MESH_PREFIX "Fire_Gas_Detector"
#define MESH_PASSWORD "saveslifefromfire"
#define MESH_PORT 5555

#define WIFI_SSID "GODGIFTED"
#define WIFI_PASSWORD "ssgrp1234"

Scheduler userScheduler;
painlessMesh mesh;

uint32_t nodeId;
unsigned long lastCheckTime = 0;
const unsigned long checkInterval = 10000; // Check every 10 seconds

void broadcastGatewayID();

Task taskBroadcastGateway(TASK_SECOND * 5, TASK_FOREVER, &broadcastGatewayID);

void broadcastGatewayID() {
  String msg = "GatewayID:" + String((unsigned long)nodeId);
  mesh.sendBroadcast(msg);
  Serial.println("Broadcasting Gateway ID...");
}

void receivedCallback(uint32_t from, String &msg) {
  Serial.printf("Message from %u: %s\n", from, msg.c_str());

  // Parse the JSON data

  StaticJsonDocument<256> doc;
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
    if(room == 1){
      Blynk.virtualWrite(V1, firesensor); // Send Fire status to Virtual Pin V1
      Blynk.virtualWrite(V2, gassensor);    // Send gas status to Virtual Pin V2
    }
    
    else if(room == 2){
      Blynk.virtualWrite(V3, firesensor); // Send Fire status to Virtual Pin V3
      Blynk.virtualWrite(V4, gassensor);    // Send gas status to Virtual Pin V4
    }

    Serial.printf("Sent to Blynk -> room: %d, firesensor: %d, gassensor: %d\n", room, firesensor, gassensor);
  } else {
    Serial.println("JSON does not contain expected keys.");
  }

}

void newConnectionCallback(uint32_t nodeId) {
  Serial.printf("New connection: Node ID = %u\n", nodeId);
}

void changedConnectionsCallback() {
  Serial.println("Connections changed.");
  size_t connectedNodes = mesh.getNodeList().size();
  if (connectedNodes == 0) {
    Serial.println("No nodes are connected.");
  } else {
    Serial.printf("%u node(s) connected.\n", connectedNodes);
  }
}

void setup() {
  Serial.begin(115200);

  // Initialize mesh network
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionsCallback);

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
  Blynk.connect();

  // Add and enable the broadcast task
  userScheduler.addTask(taskBroadcastGateway);
  taskBroadcastGateway.enable();
}

void loop() {
  mesh.update();
  userScheduler.execute();
  Blynk.run();

  // Periodically check if nodes are connected
  if (millis() - lastCheckTime > checkInterval) {
    lastCheckTime = millis();
    size_t connectedNodes = mesh.getNodeList().size();
    if (connectedNodes == 0) {
      Serial.println("No nodes are connected.");
    } else {
      Serial.printf("%u node(s) connected.\n", connectedNodes);
    }
  }
}
