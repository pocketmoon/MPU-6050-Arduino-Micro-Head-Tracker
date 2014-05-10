//
//  Head Tracker Sketch
//
// 22/03/2014 by Rob James  and Dan 'Brumter'
//
// Changelog:
//     2014-03-01 Initial Version
//     2014-03-21 Ensured License Info included
//     2014-03-22 HD version. Axis range now -32767 to 32767
//     2014-03-25 Clip axis values on limits
//     2014-03-26 Button press during 1st 10 seconds will initiate full calibrate
//     2014-03-27 Read/Write Calibrated values to EEPROM
//     2014-04-03 Added Dan's drift compensation. Changed clipping code
//     2014-04-21 Tidy up - moved main user enterable value to top
//     2014-04-23 Add Dan's exponential response as a #define
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
//when debug output is active, serial input is deactivated.
//#define DEBUGOUTPUT 

// uncomment this in to enable exponential scaling of head motion
// smaller movements near the centre, larger at the edges
//#define EXPONENTIAL

//Plug in your drift compensation value here
float xDriftComp =0.00; 
float yDriftComp =0.00; 
float zDriftComp =0.00; 

// Head movement scaling factors. Will depend on how large your screen is and 
// how close you sit to it

#ifdef EXPONENTIAL
float xScale = 8.0;
float yScale = 8.0;
float zScale = 8.0;
#else // standard linear response
float xScale = 2.5;
float yScale = 2.5;
float zScale = 2.5;
#endif;

// To avoid drift, please enter your own offset values gleaned from 
// running the master calibration sketch at
// http://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/
//
int xGyroOffset = 50;
int yGyroOffset = 20;
int zGyroOffset = 2;
int xAccelOffset = -1086;
int yAccelOffset = -807;
int zAccelOffset = 1263;

////////////////////////// END OF USER ADJUSTMENTS ////////////////////

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

float lastX, lastY, lastZ;
float dX, dY, dZ;
int driftSamples = 0;

// packet structure for InvenSense teapot demo
unsigned long lastMillis;
unsigned long lastUpdate;

float cx , cy, cz = 0.0;

//Running count of samples - used when recalibrating
int   sampleCount = 0; //should not have . value
boolean calibrated = false;

//Allows the MPU6050 to settle for 10 seconds.
//There should be no drift after this time
unsigned int calibrateTime     = 10000; //was unsigned long

//Number of samples to take when recalibrating
unsigned int  recalibrateSamples =  500;

// Holds the time since sketch stared
unsigned long  nowMillis;

TrackState_t joySt;

// Interrupt Routine - called when MPU interrupt pin gones high
// i.e. it has some juicy data for us
volatile bool mpuInterrupt = false;

void dmpDataReady() {
  mpuInterrupt = true;
}

// SERIAL PRINT - discussed at http://scott.dd.com.au/wiki/Arduino_Static_Strings
// replace Serial.print("string") with DebugPrint("string")
#define DebugPrint(x) SerialPrint_P(PSTR(x))
void SerialPrint_P(PGM_P str) {
  for (uint8_t c; (c = pgm_read_byte(str)); str++) Serial.write(c);
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
  DebugPrint("Save offsets");
#endif
  EEPROM.write(0, 97);
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
  if (valid == 97) // We have some stored offsets
  {
#ifdef DEBUGOUTPUT
    DebugPrint("Read offsets");
#endif
    xGyroOffset = readIntEE(1);
    yGyroOffset = readIntEE(3);
    zGyroOffset = readIntEE(5);
    xAccelOffset = readIntEE(7);
    yAccelOffset = readIntEE(9);
    zAccelOffset = readIntEE(11);
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
#endif

  // On first run will write the initial offsets to EEPROM
  // on subsequent runs will use offsets in EEPROM
  readOffsets();

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);

  lastMillis = millis();

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
#ifdef DEBUGOUTPUT
  DebugPrint("Init MPU6050");
#endif

  mpu.initialize();
  



  // verify connection
#ifdef DEBUGOUTPUT
  DebugPrint("Testing Connection");
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  DebugPrint("Initialising DMP");
#endif

  devStatus = mpu.dmpInitialize();

#ifdef DEBUGOUTPUT
  DebugPrint("DMP Initiaised");
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
    DebugPrint("Enabling DMP");
#endif
    mpu.setDMPEnabled(true);

    // enable interrupt detection
    //  attachInterrupt(4, dmpDataReady, RISING);
    EICRB |= (1 << ISC60) | (1 << ISC61); // sets the interrupt type for EICRB (INT6)
    EIMSK |= (1 << INT6); // activates the interrupt. 6 for 6, etc

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
#ifdef DEBUGOUTPUT
    DebugPrint("DMP ready");
#endif
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else {
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
#ifdef DEBUGOUTPUT
    DebugPrint("DMP Initialisation failed: code ");
    Serial.print(devStatus);
#endif
  }
}

void blink()
{
  // Blink the LED, slowly if we're calibrating
  // Fast if we're not.
  unsigned int delta = 100; //was unsigned long

  if (calibrated)
    delta = 300;

  if (nowMillis > lastMillis + (unsigned long)delta)
  {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    lastMillis = nowMillis;
  }
}

unsigned long latency;
boolean full_calib = false;

#ifndef DEBUGOUTPUT
//parse serial for changed values, values are used until device restart.
void parseCommand() {
      char cmd = Serial.read();
      
      switch (cmd) {
        case 'x':
        setXdrift();
        break;
        case 'y':
        setYdrift();
        break;
        case 'z':
        setZdrift();
        break;
        }
      } 
      
   void setXdrift() {
    xDriftComp = Serial.parseFloat();
    Serial.print("\nxDrift set to: ");
    Serial.println(xDriftComp);
    }   
    
   void setYdrift() {
    yDriftComp = Serial.parseFloat();
    DebugPrint("\nyDrift set to: ");
    Serial.println(yDriftComp);
    }       

   void setZdrift() {
    zDriftComp = Serial.parseFloat();
    DebugPrint("\nzDrift set to: ");
    Serial.println(zDriftComp);
    }     
#endif

void printStats()
{
  //Print current settings to serial
      Serial.println(F("ED Tracker version: 10MAY2014 \nOFFSETS Gyro X/Y/Z:"));
      //Serial.print(F("\n\tOFFSETS \tGyro X: "));
      Serial.println(xGyroOffset);
      Serial.println(yGyroOffset);
      Serial.println(zGyroOffset);
      Serial.println(F("ACCEL X/Y/Z:"));
      Serial.println(xAccelOffset);
      Serial.println(yAccelOffset);
      Serial.println(zAccelOffset);
      Serial.println(F("DRIFT Comp X/Y/Z:"));
      Serial.println(xDriftComp);
      Serial.println(yDriftComp);
      Serial.println(zDriftComp);
    
    Serial.println(F("DRIFT Sample X/Y/Z:")); 
    Serial.println(dX/(float)driftSamples  );
    Serial.println(dY/(float)driftSamples );;
    Serial.println(dZ/(float)driftSamples );
      
      /* 
      dtostrf(xDriftComp, 10, 3, buffer);
      Serial.println(buffer);
      dtostrf(yDriftComp, 10, 3, buffer);
      Serial.println(buffer);
      dtostrf(zDriftComp, 10, 3, buffer);
      Serial.println(buffer);
      */

#ifdef EXPONENTIAL
      DebugPrint("EXP ON\n");
#else // standard linear response
      DebugPrint("EXP OFF\n");
#endif;
      DebugPrint("SCALE X/Y/Z\n");
      Serial.println(xScale);
      Serial.println(yScale);
      Serial.println(zScale);
}



// Main Loop
void loop() {

  // flash the LED
  blink();

  // if DMP not ready do nothing
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    delay(0);
  }

  nowMillis = millis();

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
   // Serial.println(F("FIFO overflow!"));
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
    float newZ=   atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
    float newY = -asin(-2.0 * (q.x * q.z - q.w * q.y));
    float newX = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);

    // scale to range -32767 to 32767
    newX = newX   * 10430.06;
    newY = newY   * 10430.06;
    newZ = newZ   * 10430.06;
    
    // nowMillis = millis();
    // if we're still in the initial 'settling' period do nothing ...
    if (nowMillis < (unsigned long)calibrateTime) //added (unsigned long) to keep calibrateTime an int
    {
      
#ifdef DEBUGOUTPUT
      DebugPrint("Settle-");
#endif
      // unless we have been asked for a full blow auto calibrate!
      if (digitalRead(BUTTON_PIN) == LOW)
      {
#ifdef DEBUGOUTPUT
        DebugPrint("Full Calibration Requested.");
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
      full_calibrate();
#ifdef DEBUGOUTPUT
      DebugPrint("POST ");
      printStats();
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
        cx = cx / (float)sampleCount;
        cy = cy / (float)sampleCount;
        cz = cz / (float)sampleCount;
        dX = dY = dZ = 0.0;
        driftSamples=0;
        recalibrateSamples = 200;// reduce calibrate next time around
      }
      return;
    }

    // Have we been asked to recalibrate ?
    if (digitalRead(BUTTON_PIN) == LOW)
    {

      DebugPrint("\nRECALIBRATING\n");

      printStats(); //print stats when button is pressed
      sampleCount = 0;
      cx = cy = cz = 0;
      calibrated = false;
      return;
    }

     // apply calibration offsets
    newX = newX - cx;
    newY = newY - cy;
    newZ = newZ - cz;
    
    //clamp at 90 degrees left and right
    if (newX < -16383.0)      newX = -16383.0;
    if (newX >  16383.0)      newX =  16383.0;
    if (newY < -16383.0)      newY = -16383.0;
    if (newY >  16383.0)      newY =  16383.0;
    if (newZ < -16383.0)      newZ = -16383.0;
    if (newZ >  16383.0)      newZ =  16383.0;


#ifdef EXPONENTIAL
   long  iX = (0.0000304704*newX*newX*xScale)*(newX/abs(newX));//side mount = yaw  
   long  iY = (0.0000304704*newY*newY*yScale)*(newY/abs(newY));//side mount = pitch
   long  iZ = (0.0000304704*newZ*newZ*zScale)*(newZ/abs(newZ));//side mount = roll
#else
    // and scale to out target range plus a 'sensitivity' factor;
    long  iX = (newX * xScale );//side mount = yaw  *255
    long  iY = (newY * yScale );// side mount = pich
    long  iZ = (newZ * zScale );//side mount = roll
#endif    
    
   
    // clamp after scaling to keep values within 16 bit range
    if (iX < -32767)        iX = -32767;
    if (iX >  32767)        iX =  32767;
    if (iY < -32767)        iY = -32767;
    if (iY >  32767)        iY =  32767;
    if (iZ < -32767)        iZ = -32767;
    if (iZ >  32767)        iZ =  32767;

    // Apply X axis drift compensation every 1 second
    if (nowMillis > lastUpdate)
    {
      //depending on your mounting
      cx = cx + xDriftComp;
      cy = cy + yDriftComp;
      cz = cz + zDriftComp;
      
      lastUpdate = nowMillis + 1000;
      
      driftSamples++;
       dX += (newX - lastX);
       dY += (newY - lastY);
       dZ += (newZ - lastZ);
      
      lastX = newX;
      lastY = newY;
      lastZ = newZ;
      
      #ifdef DEBUGOUTPUT
    DebugPrint("X/Y/Z\t");
    Serial.print(newX  );
    DebugPrint("\t\t");
    Serial.print(newY );
    DebugPrint("\t\t");
    Serial.print(newZ );
    DebugPrint("\tDrift\tx:");
    
    Serial.print(dX/(float)driftSamples  );
    DebugPrint("\ty:");
    Serial.print(dY/(float)driftSamples );
    DebugPrint("\tz:");
    Serial.println(dZ/(float)driftSamples );

#endif
    }

    // Do it to it.
    joySt.xAxis = iX ;
    joySt.yAxis = iY;
    joySt.zAxis = iZ;

    Tracker.setState(&joySt);
  }
#ifndef DEBUGOUTPUT
  //if serial buffer exists then parse it
  //type x20.2 ENTER to temporarily use 20.2 as the xdrift compensation
  if (Serial.available()) parseCommand(); 
#endif
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
  DebugPrint("FULL Calibrate");
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
    DebugPrint("State 0");
#endif
    meansensors();
    state++;
    delay(1000);
  }

  if (state == 1) {
#ifdef DEBUGOUTPUT
    DebugPrint("State 1");
#endif
    calibration();
    state++;
    delay(1000);
  }

  if (state == 2) {
#ifdef DEBUGOUTPUT
    Serial.println(F("State 2"));
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
  long buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
  int i = 0; //was long

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
