#include <Joystick.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "TinPin.h"

TinPin tinpin;

MPU6050 mpu;

Joystick_ joysticks[2] {
  {0x03, JOYSTICK_TYPE_GAMEPAD, 9, 0, true, true, true, true, true, true, false, false, false, false, false, false, false},
  {0x04, JOYSTICK_TYPE_JOYSTICK, 9, 0, true, true, true, true, true, true, false, false, false, false, false, false, false}
};

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

uint16_t getOrientationData(float data) {
  return (uint16_t) (1024 * (data + PI) / (2 * PI));
}

void handleControllerData(RawControllerData data) {
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

  float forzaOutput = gravity.y;
  float threshold = 0.24;
  if (gravity.y > 0) {
    forzaOutput = (1 - threshold) * gravity.y + threshold;
  } else {
    forzaOutput = (1 - threshold) * gravity.y - threshold;
  }

  joystick.setRxAxis(getOrientationData(forzaOutput * PI));
  joystick.setRyAxis(0);
  joystick.setRzAxis(0);

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
