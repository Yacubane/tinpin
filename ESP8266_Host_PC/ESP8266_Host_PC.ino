#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "commands.h"
#include "controller.h"

//#define DEBUG
//#define DEBUG_CONTROLLER_CONNECTION
//#define DEBUG_CONTROLLER_DATA


#define MAX_CONTROLLER_TIMEOUT 5000
#define CONTROLLER_NUM 4

WiFiUDP Udp;
unsigned int localUdpPort = 1000;
char incomingPacket[255];  // buffer for incoming packets

IPAddress ip(192, 168, 1, 1);

Controller controllers[CONTROLLER_NUM] {{0}, {1}, {2}, {3}};

const char *SSID = "MoveController";
const char *WIFI_PASSWORD = "MoveController";
int lastMouseX = 0;
int lastMouseY = 0;
uint8_t mousePress = 0;
float yaw;
float pitch;
float roll;
float lastRoll;
float lastYaw;
float lastPitch;

char buffer[128];

unsigned long lastTimeReceived;
unsigned long lastTimeSent;


void setup() {
  noInterrupts();
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Testing serial...");
  WiFi.disconnect(true);
  delay(100);

#ifdef DEBUG
  Serial.println("DEBUG enabled");
#endif
#ifdef DEBUG_CONTROLLER_CONNECTION
  Serial.println("DEBUG_CONTROLLER_CONNECTION enabled");
#endif
#ifdef DEBUG_CONTROLLER_DATA
  Serial.println("DEBUG_CONTROLLER_DATA enabled");
#endif

  WiFi.mode(WIFI_AP);
  delay(100);
  WiFi.softAP(SSID, WIFI_PASSWORD);
  delay(100);
  WiFi.softAPConfig(ip, ip, IPAddress(255, 255, 255, 0));
  delay(100);
  Serial.println(WiFi.softAPIP()); // Confirm AP IP address
  delay(1);
  Serial.println("Begining UDP");
  Udp.begin(localUdpPort);
  Serial.println("Beginned UDP");
  delay(500);
  Serial.println("Started");

}

int lastControllerSendToArduino = -1;

void loop() {
  for (int i = 0; i < CONTROLLER_NUM; i++) {
    if (controllers[i].isActive() &&  millis() - controllers[i].getLastTimeGotData() > MAX_CONTROLLER_TIMEOUT) {
#ifdef DEBUG_CONTROLLER_CONNECTION
      Serial.print("Disconnecting controller ");
      Serial.println(i);
#endif
      controllers[i].setActive(false);
    }
  }


  int packetSize = Udp.parsePacket();
  if (packetSize)
  {

    //Serial.printf("Received %d %d bytes from %s, port %d\n", millis(), packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
#ifdef DEBUG
    Serial.print("Receiving: ");
    Serial.println((millis() - lastTimeReceived));
    lastTimeReceived = millis();
#endif
    IPAddress controllerIPAddress = Udp.remoteIP();
    int len = Udp.read(incomingPacket, 255);

    if (len > 0) {
      bool found = false;
      for (int i = 0; i < CONTROLLER_NUM; i++) {
        if (controllers[i].isActive() && controllers[i].getIPAddress() == controllerIPAddress) {
          controllers[i].parseControllerData(incomingPacket, len);
#ifdef DEBUG_CONTROLLER_DATA
          controllers[i].printData();
#endif
          found = true;
          break;
        }
      }

      if (!found) {
        for (int i = 0; i < CONTROLLER_NUM; i++) {
          if (!controllers[i].isActive()) {
#ifdef DEBUG_CONTROLLER_CONNECTION
            Serial.print("Connected controller ");
            Serial.println(i);
#endif
            controllers[i].setActive(true);
            controllers[i].setIPAddress(controllerIPAddress);
            controllers[i].parseControllerData(incomingPacket, len);
#ifdef DEBUG_CONTROLLER_DATA
            controllers[i].printData();
#endif
            break;
          }
        }

      }

    }
  }





  if (Serial.available() > 0) {

    bool found = false;
    for (int i = 0; i < CONTROLLER_NUM; i++) {
      lastControllerSendToArduino++;
      lastControllerSendToArduino %= CONTROLLER_NUM;
      if (controllers[lastControllerSendToArduino].isActive()) {
        found = true;
        break;
      }
    }

    if (!found) {
      lastControllerSendToArduino = -1;
    }

    if (lastControllerSendToArduino != -1) {
      int size = controllers[lastControllerSendToArduino].generateControllerPacket(buffer);
      buffer[size] = 0;
#ifdef DEBUG
      Serial.print("Sending to Arduino: ");
      Serial.println((millis() - lastTimeSent));
      lastTimeSent = millis();
#endif
      while (Serial.read() >= 0);
      Serial.write(buffer, 64);
    }
  }
  yield();
}
