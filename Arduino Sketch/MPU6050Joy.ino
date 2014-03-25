//
//  Head Tracker Sketch
//
// 22/03/2014 by Rob James  (pocketmoon@gmail.com)
//
// Changelog:
//     2014-03-01 Initial Version
//     2014-03-21 Ensured License Info included
//     2014-03-22 HD version. Axis range now -32767 to 32767

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

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUTTON_PIN 10

bool blinkState = false;

MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion 
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll

// packet structure for InvenSense teapot demo
unsigned long lastMillis;
unsigned long last_update;
unsigned long lastReport;

float cx , cy, cz =0.0;

//Running count of samples - used when recalibrating
float   sampleCount = 0.0;
boolean calibrated = false;

//Allows the MPU6050 to settle for 10 seconds.
//There should be no drift after this time
unsigned long calibrateTime     = 10000;

//Number of samples to take when recalibrating
unsigned int  recalibrateSamples =  500;

// Holds the time since sketch stared
unsigned long  nowMillis;

//comment this in if you want so see output to the serial monitor
//#define DEBUGOUTPUT 1

TrackState_t joySt;

// Interrupt Routine - called when MPU interrupt pin gones high
// i.e. it has some juicy data for us
volatile bool mpuInterrupt = false;     
void dmpDataReady() {
  mpuInterrupt = true;
}

ISR(INT6_vect) {
  dmpDataReady();
}


// Minimal Seup.
void setup()
{
#ifdef DEBUGOUTPUT
  Serial.begin(57600);
  Serial.println("Hello world");
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  lastMillis = millis();
  lastReport = 0;
  
  // wait a couple of seconds before we initialise everything
  // avoids problems with some Pro Micro clones.
  delay(1000);

  lastMillis = millis();

  // Begin talking on I2C
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

  // initialize device
#ifdef DEBUGOUTPUT
  Serial.println(F("Initializing MPU6050"));
#endif

  mpu.initialize();

  // verify connection
#ifdef DEBUGOUTPUT
  Serial.println(F("Testing Connection"));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initialising DMP"));
#endif

  devStatus = mpu.dmpInitialize();

#ifdef DEBUGOUTPUT
  Serial.println(F("DMP Initiaised"));
#endif

  // To avoid drift, please enter your OWN gyro offsets. 
  // Running the master calibration sketch at 
  //http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
  //
  mpu.setXGyroOffset(45);
  mpu.setYGyroOffset(24);
  mpu.setZGyroOffset(5);
  mpu.setZAccelOffset(1234); 

  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
#ifdef DEBUGOUTPUT
    Serial.println(F("Enabling DMP"));
#endif
    mpu.setDMPEnabled(true);

    // enable interrupt detection
#ifdef DEBUGOUTPUT
    Serial.println(F("Enabling interrupt"));
#endif
    //  attachInterrupt(4, dmpDataReady, RISING);
    EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
    EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef DEBUGOUTPUT
    Serial.println(F("DMP ready"));
#endif
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
#ifdef DEBUGOUTPUT
    Serial.print(F("DMP Initialisation failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
#endif
  }
}

void blink()
{ 
  // Blink the LED, slowly if we're calibrating
  // Fast if we're not.
  unsigned long delta = 100;
  
  if (calibrated)
    delta = 300;

  if (nowMillis > lastMillis + delta)
  {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    lastMillis = nowMillis;
  }
}

// Main Loop
void loop() {

  nowMillis = millis();

  // flash the LED
  blink();

  // if DMP not ready do nothing
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    delay(0);
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow - will get some at start of sketch
  // while we wait for system to settle
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();

#ifdef DEBUGOUTPUT
    Serial.println(F("FIFO overflow!"));
#endif
    // otherwise, check for DMP data ready interrupt 
  }
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    // Get the nice, gimbal lock free quaternion
    mpu.dmpGetQuaternion(&q, fifoBuffer);

    // Use some code to convert to R P Y
    ypr[2] =  atan2(2.0*(q.y*q.z + q.w* q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    ypr[1] = -asin(-2.0*(q.x*q.z - q.w* q.y));
    ypr[0] = -atan2(2.0*(q.x*q.y + q.w* q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);

    // scale to range -1 to 1
    float newX = (ypr[0])*0.32; 
    float newY = (ypr[1])*0.32; 
    float newZ = (ypr[2])*0.32; 

   // nowMillis = millis();
    // if we're still in the initial 'settling' period do nothing else
    if (nowMillis < calibrateTime)
    {
      return;
    }

    if (!calibrated)
    {
      if (sampleCount < recalibrateSamples)
      { // accumulate samples
        cx += newX;
        cy += newY;
        cz += newZ;
        sampleCount ++;
      }
      else
      {
        //sampleCount--;
        calibrated = true;
        cx = cx / sampleCount;
        cy = cy / sampleCount;
        cz = cz / sampleCount;
        recalibrateSamples = 200;// reduce calibrate next time around
      }
      return;
    }

    // apply calibration offsets
    newX = newX - cx;
    newY = newY - cy;
    newZ = newZ - cz;
    
    float xSensitivity = 5.0;
    float ySensitivity = 4.0;
    float zSensitivity = 6.0;
    float zFudge       = 50.0;
    
    // and scale to out target range plus a 'sensitivity' factor;
     int   iX = (int)(newX * 32767.0 * xSensitivity );//side mount = yaw  *255
     int   iY = (int)(newY * 32767.0 * ySensitivity ); // side mount = pich
     int   iZ = (int)(newZ * 32767.0 * zSensitivity + zFudge);//side mount = roll

    // Have we been asked to recalibrate ?
    if (digitalRead(BUTTON_PIN) == LOW)
    {
#ifdef DEBUGOUTPUT
      Serial.print("Recalibrate");
#endif
      sampleCount = 0;
      cx = cy = cz = 0;
      calibrated = false;
      return;
    }

    // Should match the 100Hz sampling frequency of the MPU6050
    // but just to be sure we don't flood the PC with data...
    if (nowMillis > lastReport)
    {
      // Only send data if we have moved.
      if (( (joySt.xAxis - iX)*(joySt.xAxis - iX) +
            (joySt.yAxis - iY)*(joySt.yAxis - iY) +
            (joySt.zAxis - iZ)*(joySt.zAxis - iZ)) > 3)
      {
        joySt.xAxis = iX;
        joySt.yAxis = iY;
        joySt.zAxis = iZ;

        // Do it to it.
        Tracker.setState(&joySt);

        // limit updates to once every 10ms , i.e. 100Hz
        lastReport = nowMillis + 10;

#ifdef DEBUGOUTPUT
        Serial.print("\t\t");
        Serial.print(iX );
        Serial.print("\t\t");
        Serial.print(iY);
        Serial.print("\t\t");
        Serial.print(iZ);
        Serial.println(" ");
#endif
      }
    }

  }
}


