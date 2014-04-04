
#define EMPL_TARGET_ATMEGA328

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(x)     Serial.print (x);
#define DEBUG_PRINTLN(x)  Serial.println (x);
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#endif

//comment this in if you want so see output to the serial monitor
#include <Wire.h>
#include <I2Cdev.h>

#include <helper_3dmath.h>
extern "C" {
#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
}
#include <HMC5883LCalibratable.h>

#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define BUTTON_PIN 10

bool blinkState = false;

#define SDA_PIN 2
#define SCL_PIN 3

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion
unsigned long lastReport;

// Minimal Seup.
void setup()
{
  delay(2000);
  Serial.begin(115200);

  // More robust setup and initialistion
  pinMode(SDA_PIN, INPUT);
  pinMode(SCL_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  // Clock through up to 1000 bits
  int x = 0;
  for ( int i = 0; i < 10000; i++ ) {
    digitalWrite(SCL_PIN, HIGH);
    digitalWrite(SCL_PIN, LOW);
    digitalWrite(SCL_PIN, HIGH);
    x++;
    if ( x == 8 ) {
      x = 0;
      // send a I2C stop signal
      digitalWrite(SDA_PIN, HIGH);
      digitalWrite(SDA_PIN, LOW);
    }
  }

  // send a I2C stop signal
  digitalWrite(SDA_PIN, HIGH);
  digitalWrite(SDA_PIN, LOW);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // Disable internal I2C pull-ups
  cbi(PORTC, 4);
  cbi(PORTC, 5);


  DEBUG_PRINTLN(F("Initializing MPU..."));

  boolean mpu_initialized = false;
  while ( !mpu_initialized ) {
    digitalWrite(LED_PIN, HIGH);
    if ( initialize_mpu() ) {
      mpu_initialized = true;
      DEBUG_PRINT(F("Success"));
      //boolean gyro_ok, accel_ok;
      //run_mpu_self_test(gyro_ok,accel_ok);
      enable_mpu();
    }
    else {
      digitalWrite(LED_PIN, LOW);
      DEBUG_PRINT(F("Failed"));
      mpu_force_reset();
      delay(200);
      DEBUG_PRINTLN(F("Re-initializing"));
    }
  }
  DEBUG_PRINTLN(F("Initialization Complete"));
  digitalWrite(LED_PIN, LOW);
}




/***************************************
* Invensense Hardware Abstracation Layer
***************************************/

struct hal_s {
  unsigned char sensors;
  unsigned char dmp_on;
  unsigned char wait_for_tap;
  volatile unsigned char new_gyro;
  unsigned short report;
  unsigned short dmp_features;
  unsigned char motion_int_mode;
};

static struct hal_s hal = {0};

#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ    (100)
#define MAX_NAV6_MPU_RATE (100)
#define MIN_NAV6_MPU_RATE (4)
/* Data requested by client. */
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)

/****************************************
* Gyro/Accel/DMP Configuration
****************************************/

unsigned char accel_fsr;  // accelerometer full-scale rate, in +/- Gs (possible values are 2, 4, 8 or 16).  Default:  2
unsigned short dmp_update_rate; // update rate, in hZ (possible values are between 4 and 1000).  Default:  100
unsigned short gyro_fsr;  // Gyro full-scale_rate, in +/- degrees/sec, possible values are 250, 500, 1000 or 2000.  Default:  2000

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1
                                         };



// Main Loop
void loop() {

  // wait for MPU interrupt or extra packet(s) available
  if (hal.new_gyro && hal.dmp_on)
  {
    short gyro[3], accel[3], sensors;
    unsigned char more = 0;
    long quat[4];
    unsigned long sensor_timestamp;
    float ypr[3] = { 0, 0, 0 };

    int success = dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);

    if (!more)
      hal.new_gyro = 0;

    if ( success == 0  )
    {
      Quaternion q( (float)(quat[0] >> 16) / 16384.0f,
                    (float)(quat[1] >> 16) / 16384.0f,
                    (float)(quat[2] >> 16) / 16384.0f,
                    (float)(quat[3] >> 16) / 16384.0f);

      ypr[2] =  atan2(2.0 * (q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z);
      ypr[1] = -asin(-2.0 * (q.x * q.z - q.w * q.y));
      ypr[0] = -atan2(2.0 * (q.x * q.y + q.w * q.z), q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z);


     // if (millis() > lastReport)
      {
       // lastReport = millis() + 100;
        DEBUG_PRINT("\t\t");
        DEBUG_PRINT((int)(ypr[0] * 180.0) );
        DEBUG_PRINT("\t\t");
        DEBUG_PRINT((int)(ypr[1] * 180.0) );
        DEBUG_PRINT("\t\t");
        DEBUG_PRINT((int)(ypr[2] * 180.0));
        DEBUG_PRINT("\t\t");
        DEBUG_PRINTLN(" ");
      }
    }
  }
}


void gyro_data_ready_cb(void) {
  hal.new_gyro = 1;
}

boolean initialize_mpu() {

  int result;
  struct int_param_s int_param;

  int_param.cb = gyro_data_ready_cb;  // interrupt stuff...
  int_param.pin = 4;                  // err....
  result = mpu_init(&int_param);

  if ( result != 0 ) {
    DEBUG_PRINT("mpu_init failed!");
    return false;
  }

  /* Get/set hardware configuration. Start gyro. */
  /* Wake up all sensors. */
  mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  /* Push both gyro and accel data into the FIFO. */
  mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
  mpu_set_sample_rate(DEFAULT_MPU_HZ);
  /* Read back configuration in case it was set improperly. */
  mpu_get_sample_rate(&dmp_update_rate);
  mpu_get_gyro_fsr(&gyro_fsr);
  mpu_get_accel_fsr(&accel_fsr);

  /* Initialize HAL state variables. */
  memset(&hal, 0, sizeof(hal));
  hal.sensors = ACCEL_ON | GYRO_ON;
  hal.report = PRINT_QUAT;

  result = dmp_load_motion_driver_firmware();
  if ( result != 0 ) {
    DEBUG_PRINT("Firmware Load ERROR ");
    DEBUG_PRINTLN(result);
    return false;
  }
  dmp_set_orientation(
    inv_orientation_matrix_to_scalar(gyro_orientation));

  unsigned short dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL |
                                DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL;
  dmp_enable_feature(dmp_features);
  dmp_set_fifo_rate(DEFAULT_MPU_HZ);
  return true;
}

void disable_mpu() {
  mpu_set_dmp_state(0);
  hal.dmp_on = 0;
}

void enable_mpu() {

  mpu_set_dmp_state(1);  // This enables the DMP; at this point, interrupts should commence
  hal.dmp_on = 1;
}



/* These next two functions converts the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from Invensense's MPL.
 */
unsigned short inv_row_2_scale(const signed char * row) {

  unsigned short b;

  if (row[0] > 0)
    b = 0;
  else if (row[0] < 0)
    b = 4;
  else if (row[1] > 0)
    b = 1;
  else if (row[1] < 0)
    b = 5;
  else if (row[2] > 0)
    b = 2;
  else if (row[2] < 0)
    b = 6;
  else
    b = 7;      // error
  return b;
}


unsigned short inv_orientation_matrix_to_scalar(
  const signed char * mtx) {

  unsigned short scalar;

  /*
     XYZ  010_001_000 Identity Matrix
     XZY  001_010_000
     YXZ  010_000_001
     YZX  000_010_001
     ZXY  001_000_010
     ZYX  000_001_010
   */

  scalar = inv_row_2_scale(mtx);
  scalar |= inv_row_2_scale(mtx + 3) << 3;
  scalar |= inv_row_2_scale(mtx + 6) << 6;


  return scalar;
}


