# LSM6DSM_master_LIS2MDL_slave

Sketches to read 10 DoF sensor data from an LSM6DSM master and LIS2MDL + LPS22HB slaves. LSM6DSM is master to the other two sensors, manages the data reads and stores the slave sensor data in LSM6DSM registers. The MCU host can burst read all of the sensor data from the LSM6DSM registers. This allows two or more LSM6DSM masters on a single I2C bus.

![breadboard](https://user-images.githubusercontent.com/6698410/56867677-e045db80-699c-11e9-9c6d-eaea89b286a6.jpg)

The idea here is that the LIS2MDL, which always has I2C address 0x1E, can be managed by the LSM6DSM so that multiple LSM6DSM's can be on the same host I2C bus and the magnetometers can be kept straight. The host only reads and writes to LSM6DSM registers, never talks with the LIS2MDL directly (except to configure in Setup) and, in effect, the LIS2MDL becomes part of an integrated 9 DoF LSM6DSM and all transactions can be performed by reference to the 0x6A or 0x6B I2C address of the "host" LSM6DSM. This avoids the conflict that usually occurs when two LIS2MDL are exposed directly to the MCU host on the master I2C bus since both LIS2MDL will always have the same I2C address.

The scheme can be extended to 10 Dof with the addition of a LPS22HB slave.

This method allows two or more 9- or 10-DoF sensors to be on the same I2C bus for use, for example, in wearable applications where joint motion is to be measured, etc.

These sketches are intended to run on the STM32L4 Dragonfly development board but can be easily adapted to work with the Teensy or ESP32.
 
