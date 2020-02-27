#include "Calibration.h"
#define MCP_LED2_PIN 2 //pin 23

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int max_calibration_iterations = 10;
int iterations_after_success = 4;

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;



// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.

/* ==========  LICENSE  ==================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2011 Jeff Rowberg

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
  =========================================================
*/


///////////////////////////////////   SETUP   ////////////////////////////////////
void setup(MPU6050* accelgyro) {
  accelgyro->setXAccelOffset(0);
  accelgyro->setYAccelOffset(0);
  accelgyro->setZAccelOffset(0);
  accelgyro->setXGyroOffset(0);
  accelgyro->setYGyroOffset(0);
  accelgyro->setZGyroOffset(0);
}

///////////////////////////////////   LOOP   ////////////////////////////////////


///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(MPU6050* accelgyro, Adafruit_MCP23017* mcp) {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  int ledState = 1;
  while (i < (buffersize + 101)) {
    if (i % 20 == 0) {
      mcp->digitalWrite(MCP_LED2_PIN, ledState = 1 - ledState);
    }
    // read raw accel/gyro measurements from device
    accelgyro->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;

    delay(2); //Needed so we don't get repeated measures
  }
}

bool calibrationIteration(MPU6050* accelgyro, Adafruit_MCP23017* mcp) {
  int ready = 0;
  accelgyro->setXAccelOffset(ax_offset);
  accelgyro->setYAccelOffset(ay_offset);
  accelgyro->setZAccelOffset(az_offset);

  accelgyro->setXGyroOffset(gx_offset);
  accelgyro->setYGyroOffset(gy_offset);
  accelgyro->setZGyroOffset(gz_offset);

  meansensors(accelgyro, mcp);
  Serial.println("...");
  Serial.print(ready);
  Serial.print(" ");
  Serial.print(ax_offset);
  Serial.print(" ");
  Serial.print(ay_offset);
  Serial.print(" ");
  Serial.print(gx_offset);
  Serial.print(" ");
  Serial.println(gy_offset);

  if (abs(mean_ax) <= acel_deadzone) ready++;
  else ax_offset = ax_offset - mean_ax / acel_deadzone;

  if (abs(mean_ay) <= acel_deadzone) ready++;
  else ay_offset = ay_offset - mean_ay / acel_deadzone;

  if (abs(16384 - mean_az) <= acel_deadzone) ready++;
  else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

  if (abs(mean_gx) <= giro_deadzone) ready++;
  else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

  if (abs(mean_gy) <= giro_deadzone) ready++;
  else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

  if (abs(mean_gz) <= giro_deadzone) ready++;
  else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

  if (ready == 6) return true;
  return false;
}

void calibration(MPU6050* accelgyro,  Adafruit_MCP23017* mcp) {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;

  int i = 0;

  while (i < max_calibration_iterations) {
    if (calibrationIteration(accelgyro, mcp)) {
      break;
    }
    i++;
  }

  if (i == max_calibration_iterations) {
    Serial.println("Failed to calibrate");
  }

  for (i = 0; i < iterations_after_success; i++) {
    calibrationIteration(accelgyro, mcp);
  }

}


void calibrate(MPU6050* accelgyro, Adafruit_MCP23017* mcp, ConfigManager* configManager, bool force) {
  if (force) {
    setup(accelgyro);
    if (state == 0) {
      Serial.println("\nReading sensors for first time...");
      meansensors(accelgyro, mcp);
      state++;
      delay(1000);
    }

    if (state == 1) {
      Serial.println("\nCalculating offsets...");
      calibration(accelgyro, mcp);
      state++;
      delay(1000);
    }

    if (state == 2) {
      meansensors(accelgyro, mcp);
      Serial.println("\nFINISHED!");
      Serial.print("\nSensor readings with offsets:\t");
      Serial.print(mean_ax);
      Serial.print("\t");
      Serial.print(mean_ay);
      Serial.print("\t");
      Serial.print(mean_az);
      Serial.print("\t");
      Serial.print(mean_gx);
      Serial.print("\t");
      Serial.print(mean_gy);
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset);
      Serial.print("\t");
      Serial.print(ay_offset);
      Serial.print("\t");
      Serial.print(az_offset);
      Serial.print("\t");
      Serial.print(gx_offset);
      Serial.print("\t");
      Serial.print(gy_offset);
      Serial.print("\t");
      Serial.println(gz_offset);

      configManager->setXAccelOffset(ax_offset);
      configManager->setYAccelOffset(ay_offset);
      configManager->setZAccelOffset(az_offset);

      configManager->setXGyroOffset(gx_offset);
      configManager->setYGyroOffset(gy_offset);
      configManager->setZGyroOffset(gz_offset);

      configManager->save();

      Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    }
  }
  accelgyro->setXAccelOffset(configManager->getXAccelOffset());
  accelgyro->setYAccelOffset(configManager->getYAccelOffset());
  accelgyro->setZAccelOffset(configManager->getZAccelOffset());

  accelgyro->setXGyroOffset(configManager->getXGyroOffset());
  accelgyro->setYGyroOffset(configManager->getYGyroOffset());
  accelgyro->setZGyroOffset(configManager->getZGyroOffset());
}
