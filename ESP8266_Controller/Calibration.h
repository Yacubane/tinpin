#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <ESP8266WiFi.h>
#include "MPU6050.h"
#include "ConfigManager.h"
#include <Adafruit_MCP23017.h>


void calibrate(MPU6050* mpu6050, Adafruit_MCP23017* mcp, ConfigManager* configManager, bool force);

#endif /* CALIBRATION_H */
