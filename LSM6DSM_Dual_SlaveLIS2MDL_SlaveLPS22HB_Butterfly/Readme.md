Dual LSM6DSM accel/gyro on the same I2C bus each with an LIS2MDL magnetometer and LPS22HB barometer using the STM32L433 Butterfly development board as host MCU. 
In this case, we are taking advantage of the STM32L4's stop mode and sensor-specific sleep/wake functions to put the MCU and all sensors into a low-power sleep mode and then wake everything up to calculate quaternions on a ten second on/off duty cycle. 
The "timer" is a simple counter run off of the RTC set for 1 second alarms (interrupts). 
When the counter reached 10 alarms, the device is put into sleep mode, and when reaching 10 again is woken up.
The sleep current is ~18 uA.
The program can be generalized for wake-on-motion/sleep-on-no-motion, timed sleep/wake, wake on significant motion, etc.
