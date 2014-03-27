//
//  Head Tracker Sketch
//
// 22/03/2014 by Rob James  (pocketmoon@gmail.com)
//
// Changelog:
//     2014-03-01 Initial Version
//     2014-03-21 Ensured License Info included
//     2014-03-22 HD version. Axis range now -32767 to 32767
//     2014-03-25 Clip axis values on limits
//     2014-03-26 Button press during 1st 10 seconds will initiate full calibrate
//     2014-03-27 Read/Write Calibrated values to EEPROM
//

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

//comment this in if you want so see output to the serial monitor
//#define DEBUGOUTPUT 1

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

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

float cx , cy, cz = 0.0;
int   lastX, lastY, lastZ;

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
unsigned long frame;


// To avoid drift, please enter your OWN gyro offsets.
// Running the master calibration sketch at
//http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
//
int xGyroOffset = 50;
int yGyroOffset = 20;
int zGyroOffset = 2;
int xAccelOffset = -1086;
int yAccelOffset = -807;
int zAccelOffset = 1263;

//These are used to keep track of when we view the -/+ boundary
//which causes a sudden 180 flip in-game.
//No we can prevent wrapping
int xBorder = 0;
int yBorder = 0;
int zBorder = 0;

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

//Need some helper funct to read/write integers
void writeIntEE(int value, int address)
{
  EEPROM.write(address, value & 0xff); // write lower byte
  EEPROM.write(address + 1, value >> 8); //upper byte
}

int readIntEE(int address)
{
  int val;
  val = EEPROM.read(address);
  val |= EEPROM.read(address + 1) << 8;
  return val;
}

void saveOffsets()
{
#ifdef DEBUGOUTPUT
  Serial.println(F("Save Stored offsets"));
#endif
  EEPROM.write(0, 99);
  writeIntEE (xGyroOffset, 1);
  writeIntEE (yGyroOffset, 3);
  writeIntEE (zGyroOffset, 5);
  writeIntEE (xAccelOffset, 7);
  writeIntEE (yAccelOffset, 9);
  writeIntEE (zAccelOffset, 11);
}

void readOffsets()
{
  byte valid = EEPROM.read(0) ;
  if (valid == 99) // We have some stored offsets
  {
#ifdef DEBUGOUTPUT
    Serial.print(F("Read Stored offsets :"));
#endif
    xGyroOffset = readIntEE(1);
    yGyroOffset = readIntEE(3);
    zGyroOffset = readIntEE(5);
    xAccelOffset = readIntEE(7);
    yAccelOffset = readIntEE(9);
    zAccelOffset = readIntEE(11);

#ifdef DEBUGOUTPUT
    Serial.println(xAccelOffset);
#endif
  }
  else
  {
    saveOffsets();
  }
}



// Minimal Seup.
void setup()
{
#ifdef DEBUGOUTPUT
  delay(5000);
  Serial.begin(115200);
  Serial.println("Hello world");
#endif

  // On first run will write the initial offsets to EEPROM
  // on subsequent runs will use offsets in EEPROM
  readOffsets();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  lastMillis = millis();
  lastReport = 0;

  // wait a couple of seconds before we initialise everything
  // avoids problems with some Pro Micro clones.
  delay(1000);

  lastMillis = millis();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

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

  mpu.setXGyroOffset(xGyroOffset);   //45
  mpu.setYGyroOffset(yGyroOffset);   //24
  mpu.setZGyroOffset(zGyroOffset);    //5
  mpu.setXAccelOffset(xAccelOffset);   //1234
  mpu.setYAccelOffset(yAccelOffset);   //1234
  mpu.setZAccelOffset(zAccelOffset);   //1234

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

unsigned long latency;
boolean full_calib = false;

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

  unsigned long go = millis();

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
    ypr[2] =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    ypr[1] = -asin(-2.0 * (q.x * q.z - q.w * q.y));
    ypr[0] = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

    // scale to range -1 to 1
    float newX = (ypr[0]) * 0.32;
    float newY = (ypr[1]) * 0.32;
    float newZ = (ypr[2]) * 0.32;

    // nowMillis = millis();
    // if we're still in the initial 'settling' period do nothing ...
    if (nowMillis < calibrateTime)
    {
#ifdef DEBUGOUTPUT
      Serial.println(F("Settle"));
#endif
      // unless we have been asked for a full blow auto calibrate!
      if (digitalRead(BUTTON_PIN) == LOW)
      {
#ifdef DEBUGOUTPUT
        Serial.println(F("Full Calibration Requested."));
#endif
        full_calib = true;
      }
      return;
    }

    if (full_calib == true)
    {
      //deactivate interupt
      EIMSK &= ~(1 << INT6); // activates the interrupt. 6 for 6, etc
      full_calib = false;
#ifdef DEBUGOUTPUT
      Serial.print("Pre  Gyro XYZ Accel XYZ:");
      Serial.print(xGyroOffset);
      Serial.print("\t");
      Serial.print(yGyroOffset);
      Serial.print("\t");
      Serial.print(zGyroOffset);
      Serial.print("\t");
      Serial.print(xAccelOffset);
      Serial.print("\t");
      Serial.print(yAccelOffset);
      Serial.print("\t");
      Serial.println(zAccelOffset);
#endif
      full_calibrate();
#ifdef DEBUGOUTPUT
      Serial.print("Post Gyro XYZ Accel XYZ:");
      Serial.print(xGyroOffset);
      Serial.print("\t");
      Serial.print(yGyroOffset);
      Serial.print("\t");
      Serial.print(zGyroOffset);
      Serial.print("\t");
      Serial.print(xAccelOffset);
      Serial.print("\t");
      Serial.print(yAccelOffset);
      Serial.print("\t");
      Serial.println(zAccelOffset);
#endif
      EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6,
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
        calibrated = true;
        cx = cx / sampleCount;
        cy = cy / sampleCount;
        cz = cz / sampleCount;
        recalibrateSamples = 200;// reduce calibrate next time around
        xBorder = yBorder = zBorder = 0;
      }
      return;
    }

    // apply calibration offsets
    newX = newX - cx;
    newY = newY - cy;
    newZ = newZ - cz;

    float xSensitivity = 4.0;
    float ySensitivity = 4.0;
    float zSensitivity = 6.0;

    float xFudge       = 0.0;//-15.0 * 256.;
    float yFudge       = 0.0;//-15.0 * 256.0; //-25.0 *256.0;
    float zFudge       = 0.0;// 50.0;

    // and scale to out target range plus a 'sensitivity' factor;
    int   iX = (int)(newX * 32767.0 * xSensitivity + xFudge );//side mount = yaw  *255
    int   iY = (int)(newY * 32767.0 * ySensitivity + yFudge ); // side mount = pich
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


    if (xBorder == 0)
    {
      if (iX > 17000 && lastX < -17000)  // we wrapped
      {
        xBorder = -1; //left
        iX = lastX;
      }
      else  if (iX < -17000 && lastX > 17000)
      {
        xBorder = 1 ;//left
        iX = lastX;
      }
    }
    else
    {
      //are we back inside ?
      if (xBorder == -1 && iX > -32767 && iX < -17000)
        xBorder = 0;
      else if (xBorder == 1 && iX > 17000 && iX < 32767)
        xBorder = 0;
      else
        iX = lastX;
    }


    if (yBorder == 0)
    {
      if (iY > 17000 && lastY < -17000)  // we wrapped
      {
        yBorder = -1; //left
        iY = lastY;
      }
      else  if (iY < -17000 && lastY > 17000)
      {
        yBorder = 1 ;//left
        iY = lastY;
      }
    }
    else
    {
      //are we back inside ?
      if (yBorder == -1 && iY > -32767 && iY < -17000)
        yBorder = 0;
      else if (yBorder == 1 && iY > 17000 && iY < 32767)
        yBorder = 0;
      else
        iY = lastY;
    }

    if (zBorder == 0)
    {
      if (iZ > 17000 && lastZ < -17000)  // we wrapped
      {
        zBorder = -1; //left
        iZ = lastZ;
      }
      else  if (iZ < -17000 && lastZ > 17000)
      {
        zBorder = 1 ;//left
        iZ = lastZ;
      }
    }
    else
    {
      //are we back inside ?
      if (zBorder == -1 && iZ > -32767 && iZ < -17000)
        zBorder = 0;
      else if (zBorder == 1 && iZ > 17000 && iZ < 32767)
        zBorder = 0;
      else
        iZ = lastZ;
    }



    // Only send data if we have moved.
    //    if (( (joySt.xAxis - iX) * (joySt.xAxis - iX) +
    //          (joySt.yAxis - iY) * (joySt.yAxis - iY) +
    //          (joySt.zAxis - iZ) * (joySt.zAxis - iZ)) > 1024)
    {
      lastX = joySt.xAxis = iX;
      lastY = joySt.yAxis = iY;
      lastZ = joySt.zAxis = iZ;

      // Do it to it.
      Tracker.setState(&joySt);

#ifdef DEBUGOUTPUT
      Serial.print("\t\t");
      Serial.print(iX / 256 );
      Serial.print("\t\t");
      Serial.print(iY / 256);
      Serial.print("\t\t");
      Serial.print(iZ / 256);
      Serial.print("\t\t");
      frame++;
      latency += (millis() - go);
      Serial.print((float)latency / (float)frame);
      Serial.println(" ");
#endif

    }
  }
}


/* Additional calibration code by luisrodenas */
///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

void full_calibrate()
{

#ifdef DEBUGOUTPUT
  Serial.println("FULL Calibrate. Intrrupts off. Set offsets to 0");
#endif
  // Reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  delay(100);

  if (state == 0) {
#ifdef DEBUGOUTPUT
    Serial.println("State 0");
#endif
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
#ifdef DEBUGOUTPUT
    Serial.println("State 1: Calibrate!");
#endif
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
#ifdef DEBUGOUTPUT
    Serial.println("State 2:Calculate means and reset offsets");
#endif
    meansensors();
    xGyroOffset = gx_offset;
    yGyroOffset = gy_offset;
    zGyroOffset = gz_offset;
    xAccelOffset = ax_offset;
    yAccelOffset = ay_offset;
    zAccelOffset = az_offset;
    saveOffsets();
  }
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

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

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();

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

    if (ready == 6) break;
  }
}




