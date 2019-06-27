/*
 * Copyright (c) 2019 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 *
 * Example of using two LSM6DSM accel/gyro combo IMUs on the same I2C bus including configuring the LSM6DSMs
 * as masters to slave LIS2MDL magnetometers and LPS22HB barometers, calculation of absolute orientation using Madgwick's
 * open-source sensor fusion, and demonstrating ultra-low-power sleep/wake behavior.
 */
#include "LSM6DSM.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"
#include <RTC.h>
#include "I2Cdev.h"

#define I2C_BUS   Wire1             // Define the I2C bus (Wire instance) you wish to use

I2Cdev            i2c_0(&I2C_BUS);  // Instantiate the I2Cdev object and point to the desired I2C bus

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed    13
#define myGNDpin  0
#define my3V3pin  1

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS


// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float pi = 3.141592653589793238462643383279502884f;
float GyroMeasError = pi * (40.0f / 180.0f);       // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = pi * (0.0f  / 180.0f);       // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
uint32_t sumCount1 = 0, sumCount2 = 0;             // used to control display output rate
float pitch1, yaw1, roll1, pitch2, yaw2, roll2;    // absolute orientation
float a12, a22, a31, a32, a33;            // rotation matrix 1 coefficients for Euler angles and gravity components
float A12, A22, A31, A32, A33;            // rotation matrix 2 coefficients for Euler angles and gravity components
float deltat1 = 0.0f, sum1 = 0.0f;          // integration interval for both filter schemes
float deltat2 = 0.0f, sum2 = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate1 = 0, firstUpdate1 = 0; // used to calculate integration interval
uint32_t lastUpdate2 = 0, firstUpdate2 = 0; // used to calculate integration interval
uint32_t Now1 = 0, Now2 = 0;                         // used to calculate integration interval
float lin_ax1, lin_ay1, lin_az1, lin_ax2, lin_ay2, lin_az2;             // linear acceleration (acceleration with gravity component subtracted)
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float Q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
uint32_t count = 0;
volatile bool sleepFlag = true;

//LSM6DSM definitions
#define LSM6DSM1_intPin1 2  // LSM6DSM1 INT1 pin definitions 
#define LSM6DSM2_intPin1 9  // LSM6DSM2 INT1 pin definitions 

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are:
      AFS_2G, AFS_4G, AFS_8G, AFS_16G  
      GFS_245DPS, GFS_500DPS, GFS_1000DPS, GFS_2000DPS 
      AODR_12_5Hz, AODR_26Hz, AODR_52Hz, AODR_104Hz, AODR_208Hz, AODR_416Hz, AODR_833Hz, AODR_1660Hz, AODR_3330Hz, AODR_6660Hz
      GODR_12_5Hz, GODR_26Hz, GODR_52Hz, GODR_104Hz, GODR_208Hz, GODR_416Hz, GODR_833Hz, GODR_1660Hz, GODR_3330Hz, GODR_6660Hz
*/ 
uint8_t Ascale = AFS_2G, Gscale = GFS_245DPS, AODR = AODR_833Hz, GODR = GODR_833Hz;

float aRes, gRes;              // scale resolutions per LSB for the accel and gyro sensor2
float accelBias1[3] = {0.0f, 0.0f, 0.0f}, gyroBias1[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro
int16_t LSM6DSMData1[7];        // Stores the 16-bit signed sensor output
float accelBias2[3] = {0.0f, 0.0f, 0.0f}, gyroBias2[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel and gyro
int16_t LSM6DSMData2[7];        // Stores the 16-bit signed sensor output
float   Gtemperature1, Gtemperature2;          // Stores the real internal gyro temperature in degrees Celsius
float ax1, ay1, az1, gx1, gy1, gz1, ax2, ay2, az2, gx2, gy2, gz2;  // variables to hold latest accel/gyro data values 

bool newLSM6DSMData1 = false;
bool newLSM6DSMData2 = false;

LSM6DSM LSM6DSM(&i2c_0); // instantiate LSM6DSM class

//LIS2MDL definitions

/* Specify sensor parameters (sample rate is twice the bandwidth)
 * choices are: MODR_10Hz, MOIDR_20Hz, MODR_50 Hz and MODR_100Hz
*/ 
uint8_t MODR = MODR_100Hz;

float mRes = 0.0015f;            // mag sensitivity
float magBias1[3] = {0.0f, 0.0f, 0.0f}, magScale1[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
float magBias2[3] = {0.0f, 0.0f, 0.0f}, magScale2[3]  = {0.0f, 0.0f, 0.0f}; // Bias corrections for magnetometer
int16_t LIS2MDLData1[3], LIS2MDLData2[3];          // Stores the 16-bit signed sensor output
float mx1, my1, mz1, mx2, my2, mz2;                // variables to hold latest mag data values 

LIS2MDL LIS2MDL(&i2c_0); // instantiate LIS2MDL class


// LPS22H definitions

/* Specify sensor parameters (sample rate is twice the bandwidth) 
   Choices are P_1Hz, P_10Hz P_25 Hz, P_50Hz, and P_75Hz
 */
uint8_t PODR = P_25Hz;     // set pressure amd temperature output data rate
float Temperature1, Pressure1, altitude1, Temperature2, Pressure2, altitude2;

LPS22H LPS22H(&i2c_0);


// RTC time labels
uint8_t seconds, minutes, hours, day, month, year;  
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

bool alarmFlag = false; // for RTC alarm interrupt


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.blockOnOverrun(false);
  delay(4000);

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, LOW); // start with led on

  pinMode(LSM6DSM1_intPin1, INPUT); // enable LSM6DSM interrupt
  pinMode(LSM6DSM2_intPin1, INPUT); // enable LSM6DSM interrupt

  // Configure power pins for mounted board
  pinMode(myGNDpin, OUTPUT);
  digitalWrite(myGNDpin, LOW);
  pinMode(my3V3pin, OUTPUT);
  digitalWrite(my3V3pin, HIGH);
  
  I2C_BUS.begin(); // designate I2C pins for master mode 
  I2C_BUS.setClock(400000);      // I2C frequency at 400 kHz  
  delay(1000);
 
  i2c_0.I2Cscan();

  // Read the LSM6DSM Chip ID registers, this is a good test of communication
  Serial.println("LSM6DSM accel/gyro...");
  byte c = LSM6DSM.getChipID(LSM6DSM1);  // Read CHIP_ID register for LSM6DSM
  Serial.print("LSM6DSM1 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
  Serial.println(" ");
  
  /* Configure LSM6DSM1 sensors first */
  // Put LSM6DSM1 in passthrough mode
  LSM6DSM.passthruMode(LSM6DSM1); // enable passthrough mode to initialize all the sensors
  Serial.println("LSM6DSM1 in passthrough mode for sensor configuration!");
  Serial.println(" ");
  delay(1000); 

  // Read the LIS2MDL Chip ID register, this is a good test of communication
  Serial.println("LIS2MDL mag...");
  byte d = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM6DSM
  Serial.print("LIS2MDL1 "); Serial.print("I AM "); Serial.print(d, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
  Serial.println(" ");
  delay(1000); 

  Serial.println("LPS22HB barometer...");
  uint8_t e = LPS22H.getChipID();
  Serial.print("LPS22H1 "); Serial.print("I AM "); Serial.print(e, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
  Serial.println(" ");
  delay(1000); 
  

  if(c == 0x6A && d == 0x40 && e == 0xB1) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM6DSM1 and LIS2MDL1 and LPS22HB are online..."); Serial.println(" ");
   Serial.println(" ");
      
   LPS22H.reset();
   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   // get accel/gyro sensor resolutions, only need to do this once
   aRes = LSM6DSM.getAres(Ascale);
   gRes = LSM6DSM.getGres(Gscale);

   LSM6DSM.init(LSM6DSM1, Ascale, Gscale, AODR, GODR); // configure LSM6DSM  
   LSM6DSM.selfTest(LSM6DSM1);

   LIS2MDL.reset(); // software reset LIS2MDL to default registers
   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss);  
   LIS2MDL.init(MODR);
   LIS2MDL.selfTest();

   // Put LSM6DSM1 in master mode
   LSM6DSM.masterMode(LSM6DSM1); // enable master mode to initialize all the sensors
   Serial.println("LSM6DSM1 in master mode for sensor data acquisition!");
   Serial.println(" ");
   delay(100);

  }
  else 
  {
  if(c != 0x6A) Serial.println(" LSM6DSM1 not functioning!");
  if(d != 0x40) Serial.println(" LIS2MDL1 not functioning!");    
  if(e != 0xB1) Serial.println(" LPS22HB1 not functioning!");   

  while(1){};
  }
   /* end of LSM6DSM1 sensor configuration */


   /* Configure LSM6DSM2 sensors */
   Serial.println("LSM6DSM accel/gyro...");
   byte f = LSM6DSM.getChipID(LSM6DSM2);  // Read CHIP_ID register for LSM6DSM
   Serial.print("LSM6DSM2 "); Serial.print("I AM "); Serial.print(f, HEX); Serial.print(" I should be "); Serial.println(0x6A, HEX);
   Serial.println(" ");

   // Put LSM6DSM2 in passthrough mode
   LSM6DSM.passthruMode(LSM6DSM2); // enable passthrough mode to initialize all the sensors
   Serial.println("LSM6DSM2 in passthrough mode for sensor configuration!");
   Serial.println(" ");
   delay(1000); 

   // Read the LIS2MDL Chip ID register, this is a good test of communication
   Serial.println("LIS2MDL mag...");
   byte g = LIS2MDL.getChipID();  // Read CHIP_ID register for LSM6DSM
   Serial.print("LIS2MDL1 "); Serial.print("I AM "); Serial.print(g, HEX); Serial.print(" I should be "); Serial.println(0x40, HEX);
   Serial.println(" ");
   delay(1000); 

   Serial.println("LPS22HB barometer...");
   uint8_t h = LPS22H.getChipID();
   Serial.print("LPS22H1 "); Serial.print("I AM "); Serial.print(h, HEX); Serial.print(" I should be "); Serial.println(0xB1, HEX);
   Serial.println(" ");
   delay(1000); 

  if(f == 0x6A && g == 0x40 && h == 0xB1) // check if all I2C sensors have acknowledged
  {
   Serial.println("LSM6DSM2 and LIS2MDL2 and LPS22HB2 are online..."); Serial.println(" ");
   Serial.println(" ");
      
   LPS22H.reset();
   LPS22H.Init(PODR);  // Initialize LPS22H altimeter
   delay(1000);

   LSM6DSM.init(LSM6DSM2, Ascale, Gscale, AODR, GODR); // configure LSM6DSM  
   LSM6DSM.selfTest(LSM6DSM2);

   LIS2MDL.reset(); // software reset LIS2MDL to default registers
   mRes = 0.0015f;  // fixed sensitivity and full scale (+/- 49.152 Gauss); 
   LIS2MDL.init(MODR);
   LIS2MDL.selfTest();
   
   // Put LSM6DSM2 in master mode
   LSM6DSM.masterMode(LSM6DSM2); // enable master mode to initialize all the sensors
   Serial.println("LSM6DSM2 in master mode for sensor data acquisition!");
   Serial.println(" ");
   delay(100);
  }
  else 
  {
  if(f != 0x6A) Serial.println(" LSM6DSM2 not functioning!");
  if(g != 0x40) Serial.println(" LIS2MDL2 not functioning!");    
  if(h != 0xB1) Serial.println(" LPS22HB2 not functioning!");   

  while(1){};
  }
  /* end of LSM6DSM1 sensor configuration */

   LSM6DSM.MagoffsetBias(LSM6DSM1, magBias1, magScale1);
   Serial.println("mag1 biases (mG)"); Serial.println(1000.0f * magBias1[0]); Serial.println(1000.0f * magBias1[1]); Serial.println(1000.0f * magBias1[2]); 
   Serial.println("mag1 scale (mG)"); Serial.println(magScale1[0]); Serial.println(magScale1[1]); Serial.println(magScale1[2]); 
   Serial.println(" ");
   delay(1000); // add delay to see results before serial spew of data

   LSM6DSM.MagoffsetBias(LSM6DSM2, magBias2, magScale2);
   Serial.println("mag2 biases (mG)"); Serial.println(1000.0f * magBias2[0]); Serial.println(1000.0f * magBias2[1]); Serial.println(1000.0f * magBias2[2]); 
   Serial.println("mag2 scale (mG)"); Serial.println(magScale2[0]); Serial.println(magScale2[1]); Serial.println(magScale2[2]); 
   Serial.println(" ");
   delay(1000); // add delay to see results before serial spew of data

   LSM6DSM.AGoffsetBias(LSM6DSM1, gyroBias1, accelBias1);
   Serial.println("accel1 biases (mg)"); Serial.println(1000.0f * accelBias1[0]); Serial.println(1000.0f * accelBias1[1]); Serial.println(1000.0f * accelBias1[2]);
   Serial.println(" ");
   Serial.println("gyro1 biases (dps)"); Serial.println(gyroBias1[0]); Serial.println(gyroBias1[1]); Serial.println(gyroBias1[2]);
   Serial.println(" ");
   delay(1000); 

   LSM6DSM.AGoffsetBias(LSM6DSM2, gyroBias2, accelBias2);
   Serial.println("accel2 biases (mg)"); Serial.println(1000.0f * accelBias2[0]); Serial.println(1000.0f * accelBias2[1]); Serial.println(1000.0f * accelBias2[2]);
   Serial.println(" ");
   Serial.println("gyro2 biases (dps)"); Serial.println(gyroBias2[0]); Serial.println(gyroBias2[1]); Serial.println(gyroBias2[2]);
   Serial.println(" ");
   delay(1000); 

   digitalWrite(myLed, HIGH); // turn led off

 // Set the time
  SetDefaultRTC();
  
 
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
//  RTC.enableAlarm(RTC.MATCH_SS);  // alarm once per minute

  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(digitalPinToInterrupt(LSM6DSM1_intPin1), myinthandler1, RISING);  // define interrupt for intPin1 output of LSM6DSM1
  attachInterrupt(digitalPinToInterrupt(LSM6DSM2_intPin1), myinthandler2, RISING);  // define interrupt for intPin1 output of LSM6DSM2

}

// End of setup 


void loop() {

   // If intPin goes high, either all data registers have new data
   if(newLSM6DSMData1 == true) {   // On interrupt, read data
      newLSM6DSMData1 = false;     // reset newData flag

     LSM6DSM.readAccelGyroData(LSM6DSM1, LSM6DSMData1); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax1 = (float)LSM6DSMData1[4]*aRes - accelBias1[0];  // get actual g value, this depends on scale being set
     ay1 = (float)LSM6DSMData1[5]*aRes - accelBias1[1];   
     az1 = (float)LSM6DSMData1[6]*aRes - accelBias1[2];  

   // Calculate the gyro value into actual degrees per second
     gx1 = (float)LSM6DSMData1[1]*gRes - gyroBias1[0];  // get actual gyro value, this depends on scale being set
     gy1 = (float)LSM6DSMData1[2]*gRes - gyroBias1[1];  
     gz1 = (float)LSM6DSMData1[3]*gRes - gyroBias1[2]; 
    
     LSM6DSM.readMagData(LSM6DSM1, LIS2MDLData1);
   
   // Now we'll calculate the accleration value into actual G's
     mx1 = (float)LIS2MDLData1[0]*mRes - magBias1[0];  // get actual G value 
     my1 = (float)LIS2MDLData1[1]*mRes - magBias1[1];   
     mz1 = (float)LIS2MDLData1[2]*mRes - magBias1[2]; 
     mx1 *= magScale1[0];
     my1 *= magScale1[1];
     mz1 *= magScale1[2];  

    for(uint8_t i = 0; i < 20; i++) { // iterate a fixed number of times per data read cycle
    Now1 = micros();
    deltat1 = ((Now1 - lastUpdate1)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate1 = Now1;

    sum1 += deltat1; // sum for averaging filter update rate
    sumCount1++;

    MadgwickQuaternionUpdate1(-ax1, ay1, az1, gx1*pi/180.0f, -gy1*pi/180.0f, -gz1*pi/180.0f,  mx1,  my1, -mz1);
    }
     
}

   // If intPin goes high, either all data registers have new data
   if(newLSM6DSMData2 == true) {   // On interrupt, read data
      newLSM6DSMData2 = false;     // reset newData flag

     LSM6DSM.readAccelGyroData(LSM6DSM2, LSM6DSMData2); // INT1 cleared on any read
   
   // Now we'll calculate the accleration value into actual g's
     ax2 = (float)LSM6DSMData2[4]*aRes - accelBias2[0];  // get actual g value, this depends on scale being set
     ay2 = (float)LSM6DSMData2[5]*aRes - accelBias2[1];   
     az2 = (float)LSM6DSMData2[6]*aRes - accelBias2[2];  

   // Calculate the gyro value into actual degrees per second
     gx2 = (float)LSM6DSMData2[1]*gRes - gyroBias2[0];  // get actual gyro value, this depends on scale being set
     gy2 = (float)LSM6DSMData2[2]*gRes - gyroBias2[1];  
     gz2 = (float)LSM6DSMData2[3]*gRes - gyroBias2[2]; 
    
     LSM6DSM.readMagData(LSM6DSM2, LIS2MDLData2);
   
   // Now we'll calculate the accleration value into actual G's
     mx2 = (float)LIS2MDLData2[0]*mRes - magBias2[0];  // get actual G value 
     my2 = (float)LIS2MDLData2[1]*mRes - magBias2[1];   
     mz2 = (float)LIS2MDLData2[2]*mRes - magBias2[2]; 
     mx2 *= magScale2[0];
     my2 *= magScale2[1];
     mz2 *= magScale2[2];  

    for(uint8_t i = 0; i < 20; i++) { // iterate a fixed number of times per data read cycle
    Now2 = micros();
    deltat2 = ((Now2 - lastUpdate2)/1000000.0f); // set integration time by time elapsed since last filter update
    lastUpdate2 = Now2;

    sum2 += deltat2; // sum for averaging filter update rate
    sumCount2++;

    MadgwickQuaternionUpdate2(-ax2, ay2, az2, gx2*pi/180.0f, -gy2*pi/180.0f, -gz2*pi/180.0f,  mx2,  my2, -mz2);
    }
     
}
   // end sensor interrupt handling


    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved and the MPU9250 is awake
       alarmFlag = false;

       count++;
       if(count > 10 && !sleepFlag)
       {
          LSM6DSM.passthruMode(LSM6DSM1); // enable passthrough mode to initialize all the sensors
          LIS2MDL.sleep(MODR); // set LIS2MDL to idle mode
          LPS22H.sleep();
          LSM6DSM.masterMode(LSM6DSM1);  // enable master mode to initialize all the sensors

          LSM6DSM.passthruMode(LSM6DSM2); // enable passthrough mode to initialize all the sensors
          LIS2MDL.sleep(MODR); // set LIS2MDL to idle mode
          LPS22H.sleep();
          LSM6DSM.masterMode(LSM6DSM2);  // enable master mode to initialize all the sensors

          LSM6DSM.sleepMode(LSM6DSM1);
          LSM6DSM.sleepMode(LSM6DSM2);
//          detachInterrupt(digitalPinToInterrupt(LSM6DSM1_intPin1));
//          detachInterrupt(digitalPinToInterrupt(LSM6DSM2_intPin1));
          count = 0;
          sleepFlag = true;
       }
       else if (count > 10 && sleepFlag)
       {
          LSM6DSM.passthruMode(LSM6DSM1); // enable passthrough mode to initialize all the sensors
          LIS2MDL.wake(MODR); // set LIS2MDL to idle mode
          LPS22H.wake(PODR);
          LSM6DSM.masterMode(LSM6DSM1);  // enable master mode to initialize all the sensors

          LSM6DSM.passthruMode(LSM6DSM2); // enable passthrough mode to initialize all the sensors
          LIS2MDL.wake(MODR); // set LIS2MDL to idle mode
          LPS22H.wake(PODR);
          LSM6DSM.masterMode(LSM6DSM2);  // enable master mode to initialize all the sensors

          LSM6DSM.wakeMode(LSM6DSM1, AODR, GODR);
          LSM6DSM.wakeMode(LSM6DSM2, AODR, GODR);
//          attachInterrupt(digitalPinToInterrupt(LSM6DSM1_intPin1), myinthandler1, RISING);  // define interrupt for intPin1 output of LSM6DSM1
//          attachInterrupt(digitalPinToInterrupt(LSM6DSM2_intPin1), myinthandler2, RISING);  // define interrupt for intPin1 output of LSM6DSM2
          count = 0;
          sleepFlag = false;
       }

    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) {
    Serial.print("ax1 = "); Serial.print((int)1000*ax1);  
    Serial.print(" ay1 = "); Serial.print((int)1000*ay1); 
    Serial.print(" az1 = "); Serial.print((int)1000*az1); Serial.println(" mg");
    Serial.print("gx1 = "); Serial.print( gx1, 2); 
    Serial.print(" gy1 = "); Serial.print( gy1, 2); 
    Serial.print(" gz1 = "); Serial.print( gz1, 2); Serial.println(" deg/s");
    Serial.print("mx1 = "); Serial.print((int)1000*mx1);  
    Serial.print(" my1 = "); Serial.print((int)1000*my1); 
    Serial.print(" mz1 = "); Serial.print((int)1000*mz1); Serial.println(" mG");
    
    Serial.print("q0 = "); Serial.print(q[0]);
    Serial.print(" qx = "); Serial.print(q[1]); 
    Serial.print(" qy = "); Serial.print(q[2]); 
    Serial.print(" qz = "); Serial.println(q[3]); 
    Serial.println(" ");

    Serial.print("ax2 = "); Serial.print((int)1000*ax2);  
    Serial.print(" ay2 = "); Serial.print((int)1000*ay2); 
    Serial.print(" az2 = "); Serial.print((int)1000*az2); Serial.println(" mg");
    Serial.print("gx2 = "); Serial.print( gx2, 2); 
    Serial.print(" gy2 = "); Serial.print( gy2, 2); 
    Serial.print(" gz2 = "); Serial.print( gz2, 2); Serial.println(" deg/s");
    Serial.print("mx2 = "); Serial.print((int)1000*mx2);  
    Serial.print(" my2 = "); Serial.print((int)1000*my2); 
    Serial.print(" mz2 = "); Serial.print((int)1000*mz2); Serial.println(" mG");

    Serial.print("Q0 = "); Serial.print(Q[0]);
    Serial.print(" Qx = "); Serial.print(Q[1]); 
    Serial.print(" Qy = "); Serial.print(Q[2]); 
    Serial.print(" Qz = "); Serial.println(Q[3]); 
    Serial.println(" ");
    }

    Pressure1 = (float)LSM6DSM.readBaroData(LSM6DSM1)/4096.0f;
    Temperature1 = (float)LSM6DSM.readBaroTemp(LSM6DSM1)/100.0f; 
    altitude1 = 145366.45f*(1.0f - powf((Pressure1/1013.25f), 0.190284f)); 

    if(SerialDebug) {
    Serial.print("Altimeter1 temperature = "); Serial.print( Temperature1, 2); Serial.println(" C"); // temperature in degrees Celsius  
    Serial.print("Altimeter1 temperature = "); Serial.print(9.0f*Temperature1/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter1 pressure = "); Serial.print(Pressure1, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude1 = "); Serial.print(altitude1, 2); Serial.println(" feet");
    Serial.println(" ");
    }
  
    Pressure2 = (float)LSM6DSM.readBaroData(LSM6DSM2)/4096.0f;
    Temperature2 = (float)LSM6DSM.readBaroTemp(LSM6DSM2)/100.0f; 
    altitude2 = 145366.45f*(1.0f - powf((Pressure2/1013.25f), 0.190284f)); 

    if(SerialDebug) {
    Serial.print("Altimeter2 temperature = "); Serial.print( Temperature2, 2); Serial.println(" C"); // temperature in degrees Celsius  
    Serial.print("Altimeter2 temperature = "); Serial.print(9.0f*Temperature2/5.0f + 32.0f, 2); Serial.println(" F"); // temperature in degrees Fahrenheit
    Serial.print("Altimeter2 pressure = "); Serial.print(Pressure2, 2);  Serial.println(" mbar");// pressure in millibar
    Serial.print("Altitude2 = "); Serial.print(altitude2, 2); Serial.println(" feet");
    Serial.println(" ");
    }

 
    Gtemperature1 = ((float) LSM6DSMData1[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro1 temperature is ");  Serial.print(Gtemperature1, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
    }

    Gtemperature2 = ((float) LSM6DSMData2[0]) / 256.0f + 25.0f; // Gyro chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("Gyro2 temperature is ");  Serial.print(Gtemperature2, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C
      Serial.println(" ");
    }
    
    
    a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
    a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
    a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
    a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    pitch1 = -asinf(a32);
    roll1  = atan2f(a31, a33);
    yaw1   = atan2f(a12, a22);
    pitch1 *= 180.0f / pi;
    yaw1   *= 180.0f / pi; 
    yaw1   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw1 < 0) yaw1   += 360.0f; // Ensure yaw stays between 0 and 360
    roll1  *= 180.0f / pi;
    lin_ax1 = ax1 + a31;
    lin_ay1 = ay1 + a32;
    lin_az1 = az1 - a33;

    if(SerialDebug) {
    Serial.print("Yaw1, Pitch1, Roll1: ");
    Serial.print(yaw1, 2);
    Serial.print(", ");
    Serial.print(pitch1, 2);
    Serial.print(", ");
    Serial.println(roll1, 2);

    Serial.print("Grav_x1, Grav_y1, Grav_z1: ");
    Serial.print(-a31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-a32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(a33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax1, Lin_ay1, Lin_az1: ");
    Serial.print(lin_ax1*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay1*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az1*1000.0f, 2);  Serial.println(" mg");

    Serial.print("rate2 = "); Serial.print((float)sumCount2/sum2, 2); Serial.println(" Hz");
    }
 
    A12 =   2.0f * (Q[1] * Q[2] + Q[0] * Q[3]);
    A22 =   Q[0] * Q[0] + Q[1] * Q[1] - Q[2] * Q[2] - Q[3] * Q[3];
    A31 =   2.0f * (Q[0] * Q[1] + Q[2] * Q[3]);
    A32 =   2.0f * (Q[1] * Q[3] - Q[0] * Q[2]);
    A33 =   Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
    pitch2 = -asinf(A32);
    roll2  = atan2f(A31, A33);
    yaw2   = atan2f(A12, A22);
    pitch2 *= 180.0f / pi;
    yaw2   *= 180.0f / pi; 
    yaw2   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
    if(yaw2 < 0) yaw2   += 360.0f; // Ensure yaw stays between 0 and 360
    roll2 *= 180.0f / pi;
    lin_ax2 = ax2 + A31;
    lin_ay2 = ay2 + A32;
    lin_az2 = az2 - A33;
    
    if(SerialDebug) {
    Serial.print("Yaw2, Pitch2, Roll2: ");
    Serial.print(yaw2, 2);
    Serial.print(", ");
    Serial.print(pitch2, 2);
    Serial.print(", ");
    Serial.println(roll2, 2);

    Serial.print("Grav_x2, Grav_y2, Grav_z2: ");
    Serial.print(-A31*1000.0f, 2);
    Serial.print(", ");
    Serial.print(-A32*1000.0f, 2);
    Serial.print(", ");
    Serial.print(A33*1000.0f, 2);  Serial.println(" mg");
    Serial.print("Lin_ax2, Lin_ay2, Lin_az2: ");
    Serial.print(lin_ax2*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_ay2*1000.0f, 2);
    Serial.print(", ");
    Serial.print(lin_az2*1000.0f, 2);  Serial.println(" mg");
    
    Serial.print("rate2 = "); Serial.print((float)sumCount2/sum2, 2); Serial.println(" Hz");
    }

    sumCount1 = 0;
    sum1 = 0;      
    sumCount2 = 0;
    sum2 = 0;      

    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);  
    }
     
    STM32.stop(); 
}

//  End of main loop 


void myinthandler1()
{
  newLSM6DSMData1 = true;
}


void myinthandler2()
{
  newLSM6DSMData2 = true;
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  

