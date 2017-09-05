#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>



// ADNS3080 headers
#include "SPI.h"
#include "ADNS3080.h"
// VL6180X headers
#include <Wire.h>
#include <SparkFun_VL6180X.h>
// MPU9250 headers
#include "quaternionFilters.h"
#include "MPU9250.h"

// ADNS3080 pin setups - refers to physical hardware
/// MISO
#define AP_SPI_DATAIN          12
/// MOSI
#define AP_SPI_DATAOUT         11
/// SCK
#define AP_SPI_CLOCK           13
/// SS
#define ADNS3080_CHIP_SELECT_2   10
/// SS - default active
#define ADNS3080_CHIP_SELECT_1  9
/// RESET
#define ADNS3080_RESET         5
/// First LED PWM pin - default active
#define ADNS3080_LED_1         6
/// Second LED PWM pin
#define ADNS3080_LED_2         20

/// VL6180X current Address
#define VL6180X_ADDRESS 0x29

/// ADNS3080 backup SPI settings
byte orig_spi_settings_spcr;
/// ADNS3080 backup SPI settings
byte orig_spi_settings_spsr;

// MPU9250 settings
/// Set to false for basic data read
#define AHRS true
/// Set to true to get Serial output for debugging
#define SerialDebug true
/// These can be changed, 2 and 3 are the Arduinos ext int pins
int intPin = 12;
/// Set up pin 13 led for toggling
int myLed  = 13;
float time1;
float time2;
float accel_yaw = 0;
float gyro_yaw = 0;
/// using 1d Kalman filter
float predicted_yaw = 0;
/// Complimentary filter
float comp_yaw;
/// Prediction Estimate Initial Guess
float Q = 0.1;
/// Prediction Estimate Initial Guess
float R = 5;
/// Prediction Estimate Initial Guess
float P00 = 0.1;
/// Prediction Estimate Initial Guess
float P11 = 0.1;
/// Prediction Estimate Initial Guess
float P01 = 0.1;
float Kk0, Kk1;
float gyro_scale = 17.5;
float dt;

// Various variables for ADNS3080 computations and switching and reading buffers
/// Currently selected camera
int _cs_pin = ADNS3080_CHIP_SELECT_1;
/// Non-selected camera
int _n_cs = ADNS3080_CHIP_SELECT_2;
/// set to 1 if you have reset connected
int _reset_pin = 1;
/// Currently selected LED
int _cs_led = ADNS3080_LED_1;
/// Other selected LED (currently selected regardless of camera switching)
int _n_cs_led = ADNS3080_LED_2;
///Boolean to determine if IMU is still preforming checks
boolean first_imu_reading = true;
/// max PWM duty cycle (0 - 255) note: begins to overheat over 100
int max_led_PWR = 100;
//unsigned int last_update;
/// Boolean to check for overflow of current camera
boolean _overflow = false;
/// Boolean to check for movement change of current camera
boolean _motion = false;
// computation distance values
/// Calculated int to measure total change in x direction
signed int raw_dx;
/// Calculated int to measure total change in y direction
signed int raw_dy;
// buffer distance values
/// Buffer int to measure change in x direction of the first cam
signed int raw_dx_1;
/// Buffer int to measure change in y direction of the first cam
signed int raw_dy_1;
/// Buffer int to measure change in x direction of the second cam
signed int raw_dx_2;
/// Buffer int to measure change in y direction of the second cam
signed int raw_dy_2;
// angle values
/// Last angle measurement for calculating change in angle
float last_imu_angle;
/// Current angle reading
float nonzeroed_imu_angle;
/// Calculated current angle of the IMU
float imu_angle;
/// Calculated current angle derived from the ADNS3080 cameras
float vo_angle;
/// Combined current angle of the sensor
float current_angle;
// SQUAL ints - registers quality of surface for ADNS3080
/// surface quality buffer int
unsigned int surface_quality;
/// surface quality of the first camera
unsigned int squal_1;
/// surface quality of the second camera
unsigned int squal_2;

// Learned parameters, make sure to output these for dynamic tuning.
/// Tuneable parameter for scaling down the IMU angle to real world
float tune_imu = 6.7;
/// Tuneable parameter for scaling down the motion data from the first ADNS3080 camera
float tune_squal_1 = 1.0;
/// Tuneable parameter for scaling down the motion data from the second ADNS3080 camera
float tune_squal_2 = .99;
/// Tuneable parameter relating to distance between cameras - merge with tune_cam_2
float tune_cam_1 = 75;
/// Tuneable parameter relating to scaling down angle calculations to the real world - merge with tune_cam_1
float tune_cam_2 = 8.0;

//counter for sequenceof transforms
unsigned long int cnt = 0;

/*
  // Numeric Int Type for accessing longer registers - used with changing framerate
  union NumericIntType
  {
   int         intValue; ///
   unsigned int uintValue;
   byte        byteValue[2];
  };
*/

/// VL6180X sensor initialization
VL6180x sensor(VL6180X_ADDRESS);

/// MPU9250 IMU initialization
MPU9250 myIMU;

/**
  Basic sensor initializations.
  Checks that all sensors have been initalized correctly,
  sets up high frequency PWM for the LEDs, and calibrates the IMU.
  @author Eliana Cohen (ebcohen@andrew.cmu.edu)
  @date 8/2/17
*/
float op_angle=0;

//node handle for rosserail
ros::NodeHandle  nh;

//transform broadcaster
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

//frames for tf
char base_link[] = "/base_link";
char odom[] = "/odom";


void setup()
{
  //initializing ros node
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  broadcaster.init(nh);

  // change settings to 200k for PWM signal through LED pins
  analogWriteFrequency(ADNS3080_LED_1, 200000);
  analogWriteFrequency(ADNS3080_LED_2, 200000);
  // set LEDs to output
  pinMode(ADNS3080_LED_1, OUTPUT);
  pinMode(ADNS3080_LED_2, OUTPUT);

  // turn on LEDs
  analogWrite(_n_cs_led, max_led_PWR);
  analogWrite(_cs_led, max_led_PWR);

  // set cs pins for ADNS3080 to output
  pinMode(_cs_pin, OUTPUT);
  pinMode(_n_cs, OUTPUT);
  // ensure not selected chip is not selected (active low so disable through high digital write)
  digitalWrite(_n_cs, HIGH);
  // Begin ADNS3080 communication
  Serial.begin(115200);


  delay(100);

  // Attempt ADNS3080 initialization
  if ( initOF() == false )
    Serial.println("FAILED TO INITALIZE ADNS3080");

  delay(100);

  // Begin VL6180 communication
  Wire.begin(); /// Start I2C library
  delay(100);

  if (sensor.VL6180xInit() != 0)
  {
    // Initialize device and check for errors
    Serial.println("FAILED TO INITALIZE VL6");

  };

  sensor.VL6180xDefautSettings(); // Load default settings to get started.

  // Set up the MPU9250 interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH);
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x68
  {
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);


    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.magCalibration[2], 2);
    }



  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);
    while (1) ; // Loop forever if communication doesn't happen
  }


  delay(700); // delay 1s in total in setup()
}

void loop()
{

 update_Param();
 sensor_data();
 Serial.flush();
 delay(3);
 
}

void update_Param()
{
  nh.getParam("/voof_dyn/tune_imu", &tune_imu);
  nh.getParam("/voof_dyn/tune_cam_1", &tune_cam_1);
  nh.getParam("/voof_dyn/tune_cam_2", &tune_cam_2);
  nh.getParam("/voof_dyn/tune_squal_1", &tune_squal_1);
  nh.getParam("/voof_dyn/tune_squal_2", &tune_squal_2);

 // Serial.print(tune_imu);
 // Serial.print(" ");
 // Serial.print(tune_cam_1);
 // Serial.print(" ");
 // Serial.print(tune_cam_2);
 // Serial.print(" ");
 // Serial.print(tune_squal_1);
 // Serial.print(" ");
 // Serial.println(tune_squal_2);
  
  
}


boolean initOF()
{
  int retry = 0;

  pinMode(AP_SPI_DATAOUT, OUTPUT);
  pinMode(AP_SPI_DATAIN, INPUT);
  pinMode(AP_SPI_CLOCK, OUTPUT);


  if ( _reset_pin != 0)
    pinMode(ADNS3080_RESET, OUTPUT);

  // disable current camera (Chip select is active low)
  digitalWrite(_cs_pin, HIGH);

  reset();


  // start the SPI library:
  SPI.begin();

  // check the sensor is functioning
  if ( retry < 3 )
  {
    if ( read_register(ADNS3080_PRODUCT_ID) == 0x17 )
      return true;
    retry++;
  }

  return false;
}

void reset()
{
  // return immediately if the reset pin is not defined
  if ( _reset_pin == 0)
    return;

  digitalWrite(_reset_pin, HIGH);                // reset sensor
  delayMicroseconds(10);
  digitalWrite(_reset_pin, LOW);                 // return sensor to normal
}

void sensor_data()
{
  // update both cameras' dy/dx
  process_motion();

  swap_current_camera();
  
  process_motion();
 
  // calculate VO angle - using below equation
  // (y_1*tune_squal_1 - y2*tune_squal_2) / cam_distance) / angle_scaleing
  // note, combine tune_cam_1 and tune_cam_2 for learning application
  vo_angle += asin(min(max(-1, (((raw_dy_1 * tune_squal_1) - (raw_dy_2 * tune_squal_2)) / tune_cam_1)), 1) / tune_cam_2);

  // update imu angle
  last_imu_angle = nonzeroed_imu_angle;
  IMU_MPU9250();
  
  // scale it appropriately and convert to radians
  nonzeroed_imu_angle = ((nonzeroed_imu_angle * M_PI) / 180) / tune_imu;
  // calculate change and add
  imu_angle += nonzeroed_imu_angle - last_imu_angle;

  // May want to weight vo_angle and imu_angle based on sensor certainity
  current_angle = (vo_angle + imu_angle) / 2;

  // update current delta x and y based on trajectory. (currently using one sensor)
  // in future consider using both sensors to create a new dy and dx
  raw_dx = ((raw_dy_1 * sin(current_angle)) + (raw_dx_1 * cos(current_angle)));
  raw_dy = ((raw_dy_1 * cos(current_angle)) + (raw_dx_1 * sin(current_angle)));

  // output the data
 
  //Serial.print(raw_dx, DEC);
  //Serial.print(",");
 // Serial.print(raw_dy, DEC);
 // Serial.print(",");
 // Serial.print(squal_1);
 // Serial.print(",");
 // Serial.print(squal_2);
 // Serial.print(",");
 // Serial.print(current_angle, DEC);
//  Serial.println(",");
  
//publish transforms over topic /tf
  publish_tf();
}

void publish_tf()
{

  op_angle += current_angle;
  t.header.seq = cnt++;

  //frames
  t.header.frame_id = odom;
  t.child_frame_id = base_link;

  //transform
  t.transform.translation.x += raw_dx;
  t.transform.translation.y += raw_dy; 
  t.transform.rotation = tf::createQuaternionFromYaw(current_angle);
  
  //broadcaster
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  delay(5);
  
  
}

void process_motion()
{
  
  //Updates sensor to read movement values, and checks for lack of overflow
  //Make sure to send this information if uncommented.
  updateOF();
    

  //get change in x
  if (_cs_pin == ADNS3080_CHIP_SELECT_1)
  {
    //Change y to negative x
    raw_dx_1 = (-1 * raw_dy);
    //Change x to positive y
    raw_dy_1 = (raw_dx);
    squal_1 = surface_quality;
  }
  else
  {
    //Change y to positive x
    raw_dx_2 = raw_dy;
    //change x to negative y
    raw_dy_2 = (-1*raw_dx);
    squal_2 = surface_quality;
  }

}

bool updateOF()
{
  
  byte motion_reg;
  surface_quality = (unsigned int)read_register(ADNS3080_SQUAL);
  delayMicroseconds(50);  // small delay

  // check for movement, update x,y values
  motion_reg = read_register(ADNS3080_MOTION);
  // check if we've had an overflow
  _overflow = ((motion_reg & 0x10) != 0);
  if ( (motion_reg & 0x80) != 0 ) {
    raw_dx = ((signed char)read_register(ADNS3080_DELTA_X));
    delayMicroseconds(50);  // small delay
    raw_dy = ((signed char)read_register(ADNS3080_DELTA_Y));
    _motion = true;
  }
  else
  {
    raw_dx = 0;
    raw_dy = 0;
  }
  
  //last_update = millis();

  return true;
  
}

void swap_current_camera()
{
  // swap variables
  int temp_pin = _cs_pin;
  _cs_pin = _n_cs;
  _n_cs = temp_pin;
  // pull the nonselected chip up high (disabling it)
  digitalWrite(_n_cs, HIGH);
  

}


void IMU_MPU9250()
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    myIMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes + 0.1; // - accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

    accel_yaw = atan2(myIMU.ax, myIMU.ay) * 180 / PI;

    time1 = millis();
    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
    myIMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;
    time2 = millis();
    dt = (time2 - time1) / gyro_scale;
    gyro_yaw = gyro_yaw + myIMU.gz * dt;
    comp_yaw = gyro_yaw;
    predicted_yaw = predicted_yaw + myIMU.gz * dt;

    //These codes are adapted from a paper http://digitalcommons.calpoly.edu/cgi/viewcontent.cgi?article=1114&context=aerosp
    //These p and Kk are for Kalman filter which is not used because complimentary filter works better
    P00 += dt * (2 * P01 + dt * P11); // Projected error covariance terms from derivation result: Time Update step 2
    P01 += dt * P11; // Projected error covariance terms from derivation result: Time Update step 2
    P00 += dt * Q; // Projected error covariance terms from derivation result: Time Update step 2
    P11 += dt * Q; // Projected error covariance terms from derivation result: Time Update step 2
    Kk0 = P00 / (P00 + R); // Measurement Update step 1
    Kk1 = P01 / (P01 + R); // Measurement Update step 1
    //Kalman filter output
    predicted_yaw += (accel_yaw - predicted_yaw) * Kk0;

    P00 *= (1 - Kk0); // Measurement Update step 3
    P01 *= (1 - Kk1); // Measurement Update step 3
    P11 -= Kk1 * P01; // Measurement Update step 3
    float alpha = 0.98;
    //complimentary filter output
    comp_yaw = alpha * (comp_yaw + comp_yaw * dt) + (1.0 - alpha) * accel_yaw;

    if (comp_yaw > 180) {
      comp_yaw -= 360;
    }
    if (comp_yaw < -180) {
      comp_yaw += 360;
    }

    //Serial.print("predicted_yaw is: ");
    //Serial.print(predicted_yaw);
    //Serial.print("\t");
    myIMU.roll = 0; //for VOOF which only requires YAW hence ROLL set to 0
    myIMU.pitch = 0; // for VOOF which only requires YAW hence PITCH set to 0
    //dont take the first reading as it is initially wonky
    if (!first_imu_reading) {
      nonzeroed_imu_angle = comp_yaw;
    }
    else {
      first_imu_reading = false;
    }

  }
}

byte read_register(byte address)
{
  byte result = 0, junk = 0;

  backup_spi_settings();

  // take the chip select low to select the device
  digitalWrite(_cs_pin, LOW);


  // send the device the register you want to read:
  junk = SPI.transfer(address);

  // small delay
  delayMicroseconds(50);

  //  send a value of 0 to read the first byte returned:
  result = SPI.transfer(0x00);

  //delayMicroseconds(2);

  // take the chip select high to de-select:
  digitalWrite(_cs_pin, HIGH);

  restore_spi_settings();

  return result;
}

byte backup_spi_settings()
{
  // store current spi values
  orig_spi_settings_spcr = SPCR & (DORD | CPOL | CPHA);
  orig_spi_settings_spsr = SPSR & SPI2X;

  // set the values that we need
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);  // sensor running at 2Mhz.  this is it's maximum speed

  return orig_spi_settings_spcr;
}

byte restore_spi_settings()
{
  byte temp;

  // restore SPSR
  temp = SPSR;
  temp &= ~SPI2X;
  temp |= orig_spi_settings_spsr;
  SPSR = temp;

  // restore SPCR
  temp = SPCR;
  // zero out the important bits
  temp &= ~(DORD | CPOL | CPHA);
  // restore important bits
  temp |= orig_spi_settings_spcr;
  SPCR = temp;

  return temp;
}

