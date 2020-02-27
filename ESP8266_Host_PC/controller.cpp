#include "controller.h"
#include "commands.h"

int Controller::generateControllerPacket(char* buffer) {

  buffer[0] = 0x44;
  buffer[1] = this->id;

  memcpy(buffer + 2, this->data, 26);
  return 28;
}

void Controller::parseControllerData(char* buffer, int size) {
  if (size == 28) {
    memcpy(this->data, buffer + 1, 26);
  }
  lastTimeGotData = millis();
}

void Controller::setActive(bool active) {
  this->active = active;
}

uint32_t Controller::getLastTimeGotData() {
  return this->lastTimeGotData;
}

bool Controller::isActive() {
  return this->active;
}

void Controller::setIPAddress(IPAddress ipAddress) {
  this->ipAddress = ipAddress;
}

IPAddress Controller::getIPAddress() {
  return this->ipAddress;
}
