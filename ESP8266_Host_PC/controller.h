#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ESP8266WiFi.h>

class Controller {
  private:
    char* data;

    uint32_t lastUpdateTime;
    bool active;
    uint8_t id;
    uint32_t lastTimeGotData;
    IPAddress ipAddress;

  public:
    Controller(int id) {
      this->id = id;
      active = false;
      lastUpdateTime = 0;
      this->data = (char*)malloc(sizeof(char)*26);
    }

    int generateControllerPacket(char* buffer);
    void parseControllerData(char* data, int size);
    void setActive(bool active);
    bool isActive();
    uint32_t getLastTimeGotData();
    void setIPAddress(IPAddress ipAddress);
    IPAddress getIPAddress();
};

#endif
