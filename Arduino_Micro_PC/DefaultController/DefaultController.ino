#include <Joystick.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "TinPin.h"

TinPin tinpin;

MPU6050 mpu;

Joystick_ joysticks[2] {
  {0x03, JOYSTICK_TYPE_GAMEPAD, 9, 0, true, true, true, true, true, true, true, true, false, false, false, false, false},
  {0x04, JOYSTICK_TYPE_JOYSTICK, 9, 0, true, true, true, true, true, true, true, true, false, false, false, false, false}
};


Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


uint16_t getOrientationData(float data) {
  return (uint16_t) (1024 * (data + PI) / (2 * PI));
}

uint16_t getAccelerationData(int16_t data) {
  float percentage = data / 20000.0f;
  if (percentage < -1.0) percentage = -1.0;
  if (percentage > 1.0) percentage = 1.0;
  uint16_t result = 511 + (511 * percentage);
  return result;
}

void handleControllerData(RawControllerData& data) {
  if (data.controllerNum > 1) return;
  
  Joystick_ joystick = joysticks[data.controllerNum];

  joystick.setButton(0, data.buttonA);
  joystick.setButton(1, data.buttonB);
  joystick.setButton(2, data.buttonX);
  joystick.setButton(3, data.buttonY);
  joystick.setButton(4, data.buttonLeftTrigger);
  joystick.setButton(5, data.buttonLeftBumper);
  joystick.setButton(6, data.buttonRightBumper);
  joystick.setButton(7, data.buttonRightTrigger);
  joystick.setButton(8, data.buttonJoystick);

  joystick.setXAxis(data.joystickAxis[0]);
  joystick.setYAxis(data.joystickAxis[1]);

  q.w = (float)data.quaternion[0] / 16384.0f;
  q.x = (float)data.quaternion[1] / 16384.0f;
  q.y = (float)data.quaternion[2] / 16384.0f;
  q.z = (float)data.quaternion[3] / 16384.0f;

  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  VectorInt16 accel;
  accel.x = data.acceleration[0];
  accel.y = data.acceleration[1];
  accel.z = data.acceleration[2];

  mpu.dmpGetLinearAccel(&aaReal, &accel, &gravity);

  joystick.setRxAxis(getOrientationData(ypr[0]));
  joystick.setRyAxis(getOrientationData(ypr[1]));
  joystick.setRzAxis(getOrientationData(ypr[2]));

  joystick.setZAxis(getAccelerationData(aaReal.x));
  joystick.setSliderAxis(getAccelerationData(-aaReal.y));
  joystick.setDialAxis(getAccelerationData(aaReal.z));

  joystick.sendState();
}


void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 2; i++) {
    joysticks[i].begin(false);
  }
  tinpin.begin(handleControllerData);
}

void loop() {
  tinpin.update();
}
