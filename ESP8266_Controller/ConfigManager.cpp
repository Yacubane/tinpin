#include "ConfigManager.h"
#include <ArduinoJson.h>
#include "FS.h"

ConfigManager::ConfigManager() {
  SPIFFS.begin();
  this->loaded = false;
  xAccelOffset = 0;
  yAccelOffset = 0;
  zAccelOffset = 0;
  xGyroOffset = 0;
  yGyroOffset = 0;
  zGyroOffset = 0;
}

ConfigManager& ConfigManager::getInstance() {
  static ConfigManager instance;
  return instance;
}

bool ConfigManager::load() {
  File configFile = SPIFFS.open("/config.json", "r");
  if (!configFile) {
    Serial.println("LOADING FALSE");
    this->loaded = false;
    return false;
  }
  Serial.println("LOADING TRUE");

  size_t size = configFile.size();

  char *buf = new char[size];

  configFile.readBytes(buf, size);

  const size_t capacity = 2 * JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(6) + 220;
  DynamicJsonDocument doc(capacity);

  deserializeJson(doc, buf);

  const char* network_ssid = doc["network"]["ssid"];
  const char* network_password = doc["network"]["password"];

  JsonObject mpu = doc["mpu"];
  xAccelOffset = mpu["xAccelOff"]; // -32768
  yAccelOffset = mpu["yAccelOff"]; // -32768
  zAccelOffset = mpu["zAccelOff"]; // -32768
  xGyroOffset = mpu["xGyroOff"]; // -32768
  yGyroOffset = mpu["yGyroOff"]; // -32768
  zGyroOffset = mpu["zGyroOff"]; // -32768

  //  if (!doc.success()) {
  //    delete buf;
  //    return false;
  //  }

  delete buf;
  configFile.close();

  this->loaded = true;
  return true;
}

bool ConfigManager::save() {
  const size_t capacity = 2 * JSON_OBJECT_SIZE(2) + JSON_OBJECT_SIZE(6);
  DynamicJsonDocument doc(capacity);

  JsonObject network = doc.createNestedObject("network");
  network["ssid"] = "12345678901234567890123456789012";
  network["password"] = "12345678901234567890123456789012";

  JsonObject mpu = doc.createNestedObject("mpu");
  mpu["xAccelOff"] = xAccelOffset;
  mpu["yAccelOff"] = yAccelOffset;
  mpu["zAccelOff"] = zAccelOffset;
  mpu["xGyroOff"] = xGyroOffset;
  mpu["yGyroOff"] = yGyroOffset;
  mpu["zGyroOff"] = zGyroOffset;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile) {
    return false;
  }

  serializeJson(doc, configFile);
  configFile.close();
  return true;

}
