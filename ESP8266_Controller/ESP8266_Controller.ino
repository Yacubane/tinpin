/* ============================================
  I2Cdev device library code is placed under the MIT license
  Copyright (c) 2012 Jeff Rowberg

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
  ===============================================
*/

/* This driver reads quaternion data from the MPU6060 and sends
   Open Sound Control messages.

  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   Lolin         Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin

*/

#if defined(ESP8266)
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#else
#include <WiFi.h>
#endif
#include <DNSServer.h>
#include <WiFiClient.h>
#include "commands.h"
#include <Adafruit_MCP23017.h>
#include "ConfigManager.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Calibration.h"

//#define PROTOTYPE_BOARD

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

Adafruit_MCP23017 mcp;

const int analogInPin = A0;  // ESP8266 Analog Pin ADC0 = A0
int sensorValue = 0;  // value read from the pot

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 5/3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the ESP8266 GPIO15
   pin.
   ========================================================================= */

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_TEAPOT_OSC
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#define INTERRUPT_PIN 14 // use pin 14 on ESP8266

#define MCP_BUTTON1_PIN 8
#define MCP_BUTTON2_PIN 10
#define MCP_BUTTON3_PIN 11
#define MCP_BUTTON4_PIN 12
#define MCP_BUTTON5_PIN 13
#define MCP_BUTTON6_PIN 14
#define MCP_BUTTON7_PIN 15
#define MCP_BUTTON8_PIN 7
#define MCP_BUTTON9_PIN 6

#define MCP_LED1_PIN 4 //pin 25
#define MCP_LED2_PIN 2 //pin 23

#define AXIS_X_ADJUSTMENT_VALUE 0
#define AXIS_Y_ADJUSTMENT_VALUE 0

#define DEBUG false

#define BATTERY_LOW_BLINK_TIME 1000
#define BATTERY_CRITICAL_BLINK_TIME 400
float lastBatteryLevel;

unsigned long lastTimeBatteryBlink;
int led1state = 1;

unsigned long lastTimeSent;

const char DEVICE_NAME[] = "mpu6050";

WiFiUDP Udp;
unsigned int localUdpPort = 1000;
char incomingPacket[256];

static ConfigManager configManager = ConfigManager::getInstance();

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void ICACHE_RAM_ATTR dmpDataReady() {
  mpuInterrupt = true;
}

Quaternion create_from_axis_angle(const float &xx, const float &yy, const float &zz, const float &a)
{
  // Here we calculate the sin( theta / 2) once for optimization
  double factor = sin( a / 2.0 );

  // Calculate the x, y and z of the quaternion
  double x = xx * factor;
  double y = yy * factor;
  double z = zz * factor;

  // Calcualte the w value by cos( theta / 2 )
  double w = cos( a / 2.0 );

  Quaternion quaternion(x, y, z, w);
  quaternion.normalize();

  return quaternion;
}

void die(int signals) {
  mcp.digitalWrite(MCP_LED1_PIN, HIGH);
  mcp.digitalWrite(MCP_LED2_PIN, HIGH);
  delay(1000);
  mcp.digitalWrite(MCP_LED2_PIN, LOW);
  for (int i = 0; i < signals; i++) {
    mcp.digitalWrite(MCP_LED1_PIN, LOW);
    delay(500);
    mcp.digitalWrite(MCP_LED1_PIN, HIGH);
    delay(500);
  }
  mcp.digitalWrite(MCP_LED1_PIN, LOW);
  delay(10000);
}

void mpu_setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Serial.println("Wire");
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


  if (!mpu.testConnection()) {
    die(4);
  }



  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  calibrate(&mpu, &mcp, &configManager, !mcp.digitalRead(MCP_BUTTON9_PIN));
  if (!mcp.digitalRead(MCP_BUTTON9_PIN)) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
  }



  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

const char* SSID = "MoveController";
const char* WIFI_PASSWORD = "MoveController";
const char* HOST_IP = "192.168.1.1";
const int HOST_UDP_PORT = 1000;

void sendToHostViaUdp(char* message, int size) {
  if ( DEBUG) {
    Serial.print("Sending to host: ");
    Serial.println((millis() - lastTimeSent));
  }
  lastTimeSent = millis();
  Udp.beginPacket(HOST_IP, HOST_UDP_PORT);
  Udp.write(message, size);
  Udp.endPacket();
}

void setup(void)
{
  Serial.begin(115200);
  configManager.load();

  mcp.begin();

  mcp.pinMode(MCP_BUTTON1_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON1_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON2_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON2_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON3_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON3_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON4_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON4_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON5_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON5_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON6_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON6_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON7_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON7_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON8_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON8_PIN, HIGH);
  mcp.pinMode(MCP_BUTTON9_PIN, INPUT);
  mcp.pullUp(MCP_BUTTON9_PIN, HIGH);

  mcp.pinMode(MCP_LED1_PIN, OUTPUT);
  mcp.pinMode(MCP_LED2_PIN, OUTPUT);
  mcp.digitalWrite(MCP_LED1_PIN, HIGH);
  mcp.digitalWrite(MCP_LED2_PIN, HIGH);

  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);

  mpu_setup();

  Serial.println("Connecting to wifi network");
  WiFi.begin(SSID, WIFI_PASSWORD);
  int lastState = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
    mcp.digitalWrite(MCP_LED2_PIN, lastState = 1 - lastState);
  }
  mcp.digitalWrite(MCP_LED2_PIN, HIGH);

  Udp.begin(localUdpPort);



  for (int i = 0; i < 29; i++) {
    incomingPacket[28] = 0;
  }
}

void mpu_loop()
{
  if (lastBatteryLevel > 3.25) {
    mcp.digitalWrite(MCP_LED1_PIN, HIGH);
  } else if (lastBatteryLevel > 3.05) {
    //battery level lower than 3.25 (real around 3.2)
    if (millis() - lastTimeBatteryBlink > BATTERY_LOW_BLINK_TIME) {
      mcp.digitalWrite(MCP_LED1_PIN, led1state = 1 - led1state);
      lastTimeBatteryBlink = millis();
    }
  } else {
    //battery level lower than 3.05 (real around 3.0)
    if (millis() - lastTimeBatteryBlink > BATTERY_CRITICAL_BLINK_TIME) {
      mcp.digitalWrite(MCP_LED1_PIN, led1state = 1 - led1state);
      lastTimeBatteryBlink = millis();
    }
  }

  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  if (!mpuInterrupt && fifoCount < packetSize) return;

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


#ifdef OUTPUT_READABLE_YAWPITCHROLL
    Quaternion quaternion;
    mpu.dmpGetQuaternion(&quaternion, fifoBuffer);

    mpu.dmpGetGravity(&gravity, &quaternion);
    mpu.dmpGetYawPitchRoll(ypr, &quaternion, &gravity);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    VectorInt16 accel;
    mpu.dmpGetAccel(&accel, fifoBuffer);
    
    VectorInt16 gyro;
    mpu.dmpGetGyro(&gyro, fifoBuffer);

#ifdef PROTOTYPE_BOARD
    Quaternion rotation = create_from_axis_angle(1, 0, 0, PI / 2.0);
    quaternion = rotation.getProduct(quaternion).getProduct(rotation.getConjugate());
    accel = accel.getRotated(&rotation);
    gyro = gyro.getRotated(&rotation);
#endif

    if (DEBUG) {
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / M_PI);
    }

    float ypr_tmp[3] = {ypr[0], ypr[1], ypr[2]};
    ypr[1] = ypr_tmp[2];
    ypr[2] = -ypr_tmp[1];

    int16_t aaReal_tmp[3] = {aaReal.x, aaReal.y, aaReal.z};
    aaReal.x = aaReal_tmp[1];
    aaReal.y = aaReal_tmp[0];

    incomingPacket[0] = 33;

    int16_t tmp16;
    tmp16 = htons((int16_t)(quaternion.w * 16384.0f));
    memcpy(incomingPacket + 1, &tmp16, sizeof(int16_t));
    tmp16 = htons((int16_t)(quaternion.x * 16384.0f));
    memcpy(incomingPacket + 3, &tmp16, sizeof(int16_t));
    tmp16 = htons((int16_t)(quaternion.y * 16384.0f));
    memcpy(incomingPacket + 5, &tmp16, sizeof(int16_t));
    tmp16 = htons((int16_t)(quaternion.z * 16384.0f));
    memcpy(incomingPacket + 7, &tmp16, sizeof(int16_t));
    tmp16 = htons(accel.x);
    memcpy(incomingPacket + 9, &tmp16, sizeof(int16_t));
    tmp16 = htons(accel.y);
    memcpy(incomingPacket + 11, &tmp16, sizeof(int16_t));
    tmp16 = htons(accel.z);
    memcpy(incomingPacket + 13, &tmp16, sizeof(int16_t));
    tmp16 = htons(gyro.x);
    memcpy(incomingPacket + 15, &tmp16, sizeof(int16_t));
    tmp16 = htons(gyro.y);
    memcpy(incomingPacket + 17, &tmp16, sizeof(int16_t));
    tmp16 = htons(gyro.z);
    memcpy(incomingPacket + 19, &tmp16, sizeof(int16_t));


    uint16_t buttons = 0;

    buttons |= (mcp.digitalRead(MCP_BUTTON1_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON2_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON3_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON4_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON5_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON6_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON7_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON8_PIN) == 1 ? 0 : 1);
    buttons <<= 1;
    buttons |= (mcp.digitalRead(MCP_BUTTON9_PIN) == 1 ? 0 : 1);

    tmp16 = htons(buttons);
    memcpy(incomingPacket + 21, &tmp16, sizeof(int16_t));


    uint16_t minAnalogReadValue = 27; //0.026V
    uint16_t maxAnalogReadValue = 800; //0.784V

    digitalWrite(12, LOW);
    digitalWrite(13, HIGH);
    uint16_t axisX  = clamp(map(analogRead(analogInPin), minAnalogReadValue, maxAnalogReadValue, 0, 1023), 0, 1023);
    if (axisX < 0)

      if (DEBUG) {
        Serial.print("sensor = ");
        Serial.print(axisX);
      }

    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    uint16_t axisY  = clamp(map(analogRead(analogInPin), minAnalogReadValue, maxAnalogReadValue, 0, 1023), 0, 1023);

    if (DEBUG) {
      Serial.print(" ");
      Serial.print(axisY);
    }

    digitalWrite(12, HIGH);
    digitalWrite(13, LOW);

    if (DEBUG) {
      Serial.print(" RAW BAT: ");
    }
    uint16_t raw_battery = analogRead(analogInPin);
    float battery = raw_battery / (1.2 / (4.7 + 1.2)) / 1024;
    lastBatteryLevel = battery;

    if (DEBUG) {
      Serial.print(analogRead(analogInPin));
      Serial.print(" BAT: ");
      Serial.print(battery);
    }

    tmp16 = htons(axisX);
    memcpy(incomingPacket + 23, &tmp16, sizeof(int16_t));
    tmp16 = htons(axisY);
    memcpy(incomingPacket + 25, &tmp16, sizeof(int16_t));

    incomingPacket[27] = '\0';

    sendToHostViaUdp(incomingPacket, 28);
    yield();
#endif
  }
}

int clamp(int const value, int const lhs, int const rhs)
{
  return ((value < lhs) ? lhs : ((value > rhs) ? rhs : value));
}

void loop(void)
{
  mpu_loop();
  yield();
  delay(1);
}
