/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The LSM6DSM is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include "LSM6DSM.h"
#include "I2Cdev.h"
#include "LIS2MDL.h"
#include "LPS22HB.h"

LSM6DSM::LSM6DSM(uint8_t intPin1, I2Cdev* i2c_bus)
{
  _intPin1 = intPin1;
  _i2c_bus = i2c_bus;
}


uint8_t LSM6DSM::getChipID()
{
  uint8_t c = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_WHO_AM_I);
  return c;
}

void LSM6DSM::passthruMode()
{
  //DRDY_ON_INT1 = bit 7, START_CONFIG = bit 4, PULLUP_EN = bit 3, PASSTHRU_MODE = bit 2, MASTER_ON = bit 0
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL10_C, 0x04); // enable embedded functions
  delay(100);
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_MASTER_CONFIG, 0x10); // enable start configuration to disable the sensor hub trigger
  delay(100);
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_MASTER_CONFIG, 0x04); // enable pass through mode
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL10_C, 0x00); // disable embedded functions
}

void LSM6DSM::masterMode()
{
  // MASTER_CONFIG
  //DRDY_ON_INT1 = bit 7, START_CONFIG = bit 4, PULLUP_EN = bit 3, PASSTHRU_MODE = bit 2, MASTER_ON = bit 0
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL10_C, 0x04);                          // enable embedded functions
  delay(100); // wait for all host I2C transactions to complete, 100 ms should be long enough
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_MASTER_CONFIG, 0x00);                     // disable passthrough mode
  uint8_t c = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL);                     // preserve accel configuration
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, 0x00);                          // disable accel 
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL10_C, 0x00);                          // disable embedded functions

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_FUNC_CFG_ACCESS, 0x80);                   // enable access to embedded function registers (Bank A)

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL6_C, 0x80);                           // configure INT2 as input

  // Configure LIS2MDL slave
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLV0_ADD, (LIS2MDL_ADDRESS << 1) | 0x01); // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLV0_SUBADD, LIS2MDL_OUTX_L_REG);         // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLAVE0_CONFIG, 0x16);                     // select 8x decimation, two slave sensors, six bytes to read

  // Configure LPS22HB slave
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLV1_ADD, (LPS22H_ADDRESS << 1) | 0x01); // enable slave sensor read operation
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLV1_SUBADD, LPS22H_PRESS_OUT_XL);        // select slave sensor starting register for read operation
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_SLAVE1_CONFIG, 0x85);                     // select 4x decimation, five bytes to read

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_FUNC_CFG_ACCESS, 0x00);                   // disable access to embedded function registers (Bank A)

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL10_C, 0x04);                          // enable embedded functions
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_MASTER_CONFIG, 0x19);                     // enable master mode, internal pullups, accel trigger drdy
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL6_C, 0x00);                           // un-configure INT2 as input
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, c);                             // reset accel with previous configuration
}


float LSM6DSM::getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      return _aRes;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      return _aRes;
      break;
  }
}

float LSM6DSM::getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    case GFS_245DPS:
      _gRes = 245.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      return _gRes;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      return _gRes;
      break;
  }
}


void LSM6DSM::reset()
{
  // reset device
  uint8_t temp = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x01); // Set bit 0 to 1 to reset LSM6DSM
  delay(100); // Wait for all registers to reset
}


void LSM6DSM::init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL1_XL, AODR << 4 | Ascale << 2);

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL2_G, GODR << 4 | Gscale << 2);

  uint8_t temp = _i2c_bus->readByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C);
  // enable block update (bit 6 = 1), auto-increment registers (bit 2 = 1)
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL3_C, temp | 0x40 | 0x04);
  // by default, interrupts active HIGH, push pull, little endian data
  // (can be changed by writing to bits 5, 4, and 1, resp to above register)

  // enable accel LP2 (bit 7 = 1), set LP2 tp ODR/9 (bit 6 = 1), enable input_composite (bit 3) for low noise
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL8_XL, 0x80 | 0x40 | 0x08 );

  // interrupt handling
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_DRDY_PULSE_CFG, 0x80); // latch interrupt until data read
  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_INT1_CTRL, 0x03);      // enable data ready interrupts on INT1
}


void LSM6DSM::readMagData(int16_t * destination)
{
  uint8_t rawData[8];  // x/y/z mag register data stored here
  _i2c_bus->readBytes(LSM6DSM_ADDRESS, LSM6DSM_SENSORHUB1_REG, 6, &rawData[0]);  // Read the 6 raw data registers into data array

  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;       // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}


int32_t LSM6DSM::readBaroData()
{
    uint8_t rawData[3];  // 24-bit pressure register data stored here
    _i2c_bus->readBytes(LSM6DSM_ADDRESS, LSM6DSM_SENSORHUB7_REG, 3, &rawData[0]);  
    return (int32_t) ((int32_t) rawData[2] << 16 | (int32_t) rawData[1] << 8 | rawData[0]);
}


int16_t LSM6DSM::readBaroTemp()
{
    uint8_t rawData[2];  // 16-bit pressure register data stored here
   _i2c_bus->readBytes(LSM6DSM_ADDRESS, LSM6DSM_SENSORHUB10_REG, 2, &rawData[0]);  
    return (int16_t)((int16_t) rawData[1] << 8 | rawData[0]);
}


void LSM6DSM::selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readAccelGyroData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x01); // positive accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x03); // negative accel self test
  delay(100); // let accel respond
  readAccelGyroData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x04); // positive gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x0C); // negative gyro self test
  delay(100); // let gyro respond
  readAccelGyroData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  _i2c_bus->writeByte(LSM6DSM_ADDRESS, LSM6DSM_CTRL5_C, 0x00); // normal mode
  delay(100); // let accel and gyro respond

  Serial.println("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");
  Serial.println(" ");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  Serial.println(" ");
  delay(2000);
}


void LSM6DSM::AGoffsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  Serial.println("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  delay(10000);

  for (uint8_t ii = 0; ii < 128; ii++)
  {
    readAccelGyroData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    delay(50);
  }

  dest1[0] = sum[1] * _gRes / 128.0f;
  dest1[1] = sum[2] * _gRes / 128.0f;
  dest1[2] = sum[3] * _gRes / 128.0f;
  dest2[0] = sum[4] * _aRes / 128.0f;
  dest2[1] = sum[5] * _aRes / 128.0f;
  dest2[2] = sum[6] * _aRes / 128.0f;

  if (dest2[0] > 0.75f)  {
    dest2[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[0] < -0.75f) {
    dest2[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest2[1] > 0.75f)  {
    dest2[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[1] < -0.75f) {
    dest2[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest2[2] > 0.75f)  {
    dest2[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest2[2] < -0.75f) {
    dest2[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}


void LSM6DSM::MagoffsetBias(float * dest1, float * dest2)
{
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {-32767, -32767, -32767}, mag_min[3] = {32767, 32767, 32767}, mag_temp[3] = {0, 0, 0};
  float _mRes = 0.0015f;
  
  Serial.println("Calculate mag offset bias: move all around to sample the complete response surface!");
  delay(4000);

  for (int ii = 0; ii < 4000; ii++)
  {
    readMagData(mag_temp);
       for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(12);
  }

  _mRes = 0.0015f; // fixed sensitivity
    // Get hard iron correction
    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts
    
    dest1[0] = (float) mag_bias[0] * _mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1] * _mRes;   
    dest1[2] = (float) mag_bias[2] * _mRes;  
       
    // Get soft iron correction estimate
    mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
    mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
    mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

    float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0f;

    dest2[0] = avg_rad/((float)mag_scale[0]);
    dest2[1] = avg_rad/((float)mag_scale[1]);
    dest2[2] = avg_rad/((float)mag_scale[2]);
  
   Serial.println("Mag Calibration done!");
}


void LSM6DSM::readAccelGyroData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  _i2c_bus->readBytes(LSM6DSM_ADDRESS, LSM6DSM_OUT_TEMP_L, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = (int16_t)((int16_t)rawData[1] << 8)  | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t)((int16_t)rawData[3] << 8)  | rawData[2] ;
  destination[2] = (int16_t)((int16_t)rawData[5] << 8)  | rawData[4] ;
  destination[3] = (int16_t)((int16_t)rawData[7] << 8)  | rawData[6] ;
  destination[4] = (int16_t)((int16_t)rawData[9] << 8)  | rawData[8] ;
  destination[5] = (int16_t)((int16_t)rawData[11] << 8) | rawData[10] ;
  destination[6] = (int16_t)((int16_t)rawData[13] << 8) | rawData[12] ;
}

