#ifndef TINPIN_H
#define TINPIN_H

#include "hton.h"

#define CONTROLLERS_NUM 4
#define CONTROLLER_CONNECTION_TIMEOUT 5000
#define CONTROLLER_PREAMBLE 0x44

//#define DEBUG

typedef struct RawControllerData {
  int controllerNum;

  int16_t quaternion[4];
  int16_t acceleration[3];
  int16_t gyroscope[3];

  int16_t joystickAxis[2];

  int buttonJoystick;
  int buttonA;
  int buttonB;
  int buttonX;
  int buttonY;
  int buttonRightBumper;
  int buttonLeftBumper;
  int buttonRightTrigger;
  int buttonLeftTrigger;
} RawControllerData;

class TinPin {
  private:
    char buffer[64];
    long lastReceiveTime;
    long controllersLastReceiveTime[CONTROLLERS_NUM];
    int controllerActiveLedPins[CONTROLLERS_NUM];
    int esp8266ConnectionActiveLed;

    void (*handleControllerDataFunction)(RawControllerData& rawControllerData);
  public:
    TinPin() {
      esp8266ConnectionActiveLed = 6;

      for (int i = 0; i < CONTROLLERS_NUM; i++) {
        controllersLastReceiveTime[i] = 0;
        controllerActiveLedPins[i] = 2 + i;
      };
    }
    begin(void (*handleControllerDataFunction)(RawControllerData & rawControllerData) ) {
      pinMode(17, OUTPUT);
      for (int i = 2; i < 7; i++) {
        pinMode(i, OUTPUT);
      }
      Serial1.begin(115200);
      this->handleControllerDataFunction = handleControllerDataFunction;
    }
    update() {
      for (int i = 0; i < CONTROLLERS_NUM; i++) {
        if (millis() - controllersLastReceiveTime[i] > CONTROLLER_CONNECTION_TIMEOUT) {
          digitalWrite(controllerActiveLedPins[i], LOW);
        }
      };

      unsigned long time = millis();

      int iterator = 0;

      while (true) {
        if (millis() - time > 1000) {
          break;
        }

        while (Serial1.available() > 0) {
          time = millis();
          char input = Serial1.read();
          buffer[iterator++] = input;
        }

        if (iterator >= 64) {
          break;
        }
      }
      Serial1.print('R');
      while (Serial1.read() >= 0);
      readBuffer(iterator);
    }

  private:
    void handleControllerData(RawControllerData & data) {
      handleControllerDataFunction(data);
    }
    int16_t readInt(char* ptr) {
      int16_t tmp;
      memcpy(&tmp, ptr, sizeof(int16_t));
      return ntohs(tmp);
    }

    int readNextCommand(int start) {
      if (buffer[start] == CONTROLLER_PREAMBLE) {
        int controllerNum = buffer[start + 1];

        digitalWrite(controllerActiveLedPins[controllerNum], HIGH);
        controllersLastReceiveTime[controllerNum] = millis();

        int16_t quaternion[4];
        int16_t accel[3];
        int16_t gyro[3];

        RawControllerData data;

        data.controllerNum = controllerNum;

        data.quaternion[0] = readInt(buffer + 2);
        data.quaternion[1] = readInt(buffer + 4);
        data.quaternion[2] = readInt(buffer + 6);
        data.quaternion[3] = readInt(buffer + 8);

        data.acceleration[0] = readInt(buffer + 10);
        data.acceleration[1] = readInt(buffer + 12);
        data.acceleration[2] = readInt(buffer + 14);

        data.gyroscope[0] = readInt(buffer + 16);
        data.gyroscope[1] = readInt(buffer + 18);
        data.gyroscope[2] = readInt(buffer + 20);

        uint16_t buttons = readInt(buffer + 22);
        data.buttonJoystick = buttons >> 8 & 1;
        data.buttonX = buttons >> 7 & 1;
        data.buttonA = buttons >> 6 & 1;
        data.buttonY = buttons >> 5 & 1;
        data.buttonB = buttons >> 4 & 1;
        data.buttonRightTrigger = buttons >> 3 & 1;
        data.buttonRightBumper = buttons >> 2 & 1;
        data.buttonLeftTrigger = buttons >> 1 & 1;
        data.buttonLeftBumper = buttons >> 0 & 1;

        data.joystickAxis[0] = readInt(buffer + 24);
        data.joystickAxis[1] = readInt(buffer + 26);

        handleControllerData(data);

        return start + 28;
      }
      else {
        return -1;
      }
    }

    void readBuffer(int size) {
#ifdef DEBUG
      Serial.print("Difference: ");
      Serial.print((millis() - lastReceiveTime));
      Serial.print(" Size: ");
      Serial.println(size);
#endif
      lastReceiveTime = millis();
      int start = 0;
      while (start < size && start != -1) {
        start = readNextCommand(start);
      }
    }

};

#endif
