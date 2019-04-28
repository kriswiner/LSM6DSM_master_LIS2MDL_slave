# LSM6DSM_master_LIS2MDL_slave

Sketches to read 10 DoF sensor data from an LSM6DSM master and LIS2MDL + LPS22HB slaves. LSM6DSM is master to the other two sensors, manages the data reads and stores the slave sensor data in LSM6DSM registers. The MCU host can burst read all of the sensor data from the LSM6DSM registers. This allows two or more LSM6DSM masters on a single I2C bus.
