#include <Mouse.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "TinPin.h"

TinPin tinpin;

MPU6050 mpu;

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


float prevYpr[3];       // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
bool lastButtonB;
void handleControllerData(RawControllerData& data) {
  if (data.controllerNum == 0) {
    q.w = (float)data.quaternion[0] / 16384.0f;
    q.x = (float)data.quaternion[1] / 16384.0f;
    q.y = (float)data.quaternion[2] / 16384.0f;
    q.z = (float)data.quaternion[3] / 16384.0f;

    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    float multiplier = 1000.0;
    Mouse.move((ypr[0] - prevYpr[0])*multiplier,
               (prevYpr[2] - ypr[2])*multiplier, 0);

    prevYpr[0] = ypr[0];
    prevYpr[1] = ypr[1];
    prevYpr[2] = ypr[2];

    if (data.buttonB && !lastButtonB) {
      Mouse.click();
    }
    lastButtonB = data.buttonB;
  }
}


void setup() {
  Serial.begin(115200);
  Mouse.begin();
  tinpin.begin(handleControllerData);
}

void loop() {
  tinpin.update();
}
