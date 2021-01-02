# MPU9250 Calibration using Python and a Raspberry Pi Computer
Calibration procedure for the MPU9250's accelerometer, gyroscope, and magnetometer using Python and a Raspberry Pi Computer

### - Wiring Diagram - 

Power is supplied to the MPU9250 via the 3.3V/GND pins on the Raspberry Pi computer. The MPU9250 board communicates with the RPi over its Inter-Integrated Circuit (I<sup>2</sup>C) pins, labeled SDA/SCL on both the RPi and MPU9265 boards. On the RPi, SDA is located on hardware pin 3, and SCL is located on hardware pin 5. 
![Wiring diagram of MPU9250 to RPI4](./images/mpu9250_raspberry_pi_4_wiring_diagram.png)
___
### - Python Library Installs and IMU Tests - 
Install the following Python packages onto the Raspberry Pi:
```bash
pi@raspberrypi~ $ sudo pip3 install scipy matplotlib numpy
pi@raspberrypi~ $ sudo apt-get install libatlas-base-dev
```
Upon successful of the library installs above, download the mpu9250_i2c.py (the main backend for reading the MPU9250) from the repository along with the imu_test.py code. 

- Run the imu_test.py 

Verify that the 9 variables are being outputted similar to the screenshot below:

![MPU9250 Test Printout](./images/imu_test_printout.png)
___
### - Calibration Outline - 

The calibration procedure uses a uniform cube to calibrate each sensor on the IMU (accel, gyro, mag). There are different procedures involved in calibrating each sensor, which can be broken down as follows:

- Gyroscope: calibrate under steady conditions
- Accelerometer: calibrate under graviation for each axis
- Magnetometer: calibrate under 360 degree rotation for each axis

Keeping in mind the coordinate reference for the MPU9250:

![MPU9250 Coordinate References](./images/mpu6050_ak8963_drawing_coordinates.png)
___
### - Gyroscope Calibration Output - 

This section pertains to the script:
- gyroscope_calibration.py

![MPU9250 Gyro Output](./images/gyro_calibration_output_white.jpg)
