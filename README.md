# MPU9250 Calibration using Python and a Raspberry Pi Computer
Calibration procedure for the MPU9250's accelerometer, gyroscope, and magnetometer using Python and a Raspberry Pi Computer

### - Basic Wiring Diagram - 

Power is supplied to the MPU9250 via the 3.3V/GND pins on the Raspberry Pi computer. The MPU9250 board communicates with the RPi over its Inter-Integrated Circuit (I<sup>2</sup>C) pins, labeled SDA/SCL on both the RPi and MPU9265 boards. On the RPi, SDA is located on hardware pin 3, and SCL is located on hardware pin 5. 
![Wiring diagram of MPU9265 to RPI4](./images/mpu9250_raspberry_pi_4_wiring_diagram.png)

### - Python Calibration Procedure - 
```bash
pi@raspberrypi~ $</font> sudo pip3 install scipy
```
