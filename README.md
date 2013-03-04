IMU
===
The program exists in the following files:
main.c - System Initialization and main control loop
motors.c - methods for controlling Brushless Electronic Speed Controllers
offline_kalman.c - methods for precomputing Kalman Gain
MPU6050.c - methods for controlling the MPU-6050 from Invensense
I2Clib.c - methods for hardware I2C communications

The code attempts to be as easy to port as possible.  Platform specific code exists only in:
motors.c - timer routines
I2Clib.c - Hardware I2C routines
main.c - Interrupt Handlers and Configuration, Hardware UART routines
