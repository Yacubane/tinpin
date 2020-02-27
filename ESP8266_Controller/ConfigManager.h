#ifndef CONFIGMANAGER_H
#define CONFIGMANAGER_H

#include <ESP8266WiFi.h>


class ConfigManager {
  private:
    int16_t xAccelOffset;
    int16_t yAccelOffset;
    int16_t zAccelOffset;

    int16_t xGyroOffset;
    int16_t yGyroOffset;
    int16_t zGyroOffset;

    bool loaded;

    ConfigManager();
  public:
    static ConfigManager& getInstance();
    bool load();
    bool save();
    void setXAccelOffset(int16_t value) {
      this->xAccelOffset = value;
    }

    void setYAccelOffset(int16_t value) {
      this->yAccelOffset = value;
    }

    void setZAccelOffset(int16_t value) {
      this->zAccelOffset = value;
    }

    void setXGyroOffset(int16_t value) {
      this->xGyroOffset = value;
    }

    void setYGyroOffset(int16_t value) {
      this->yGyroOffset = value;
    }

    void setZGyroOffset(int16_t value) {
      this->zGyroOffset = value;
    }

    int16_t getXAccelOffset() {
      return xAccelOffset;
    }


    int16_t getYAccelOffset() {
      return yAccelOffset;
    }

    int16_t getZAccelOffset() {
      return zAccelOffset;
    }

    int16_t getXGyroOffset() {
      return xGyroOffset;
    }

    int16_t getYGyroOffset() {
      return yGyroOffset;
    }

    int16_t getZGyroOffset() {
      return zGyroOffset;
    }

    bool isLoaded() {
      return loaded;
    }

};

#endif /* CONFIGMANAGER_H */
