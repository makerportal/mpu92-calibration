# MPU9250 Calibration using Python and a Raspberry Pi Computer
Calibration procedure for the MPU9250's accelerometer, gyroscope, and magnetometer using Python and a Raspberry Pi Computer

### JUMP TO:
<a href="#wiring">Basic Wiring Diagram</a><br>
<a href="#setup">Setting Up RPi4 for I<sup>2</sup>C</a><br>
<a href="#process">Process to follow when running Python codes</a><br>
<a href="#algorithms">Algorithms</a><br>
<a href="#updates">Python Routine Updates</a><br>

<a id="wiring"></a>
### - Basic Wiring Diagram - 

Power is supplied to the MPU9250 via the 3.3V/GND pins on the Raspberry Pi computer. The MPU9250 board communicates with the RPi over its Inter-Integrated Circuit (I<sup>2</sup>C) pins, labeled SDA/SCL on both the RPi and MPU9265 boards. On the RPi, SDA is located on hardware pin 3, and SCL is located on hardware pin 5. 
![Wiring diagram of MPU9265 to RPI4](./images/mpu9250_raspberry_pi_4_wiring_diagram.png)
