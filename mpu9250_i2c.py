#########################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
#########################################
#
# This code handles the smbus 
# communications between the RPi and the
# GY-91 (MPU9250+BMP280). For implementations
# see: gy91_plotter_basic.py
#
#########################################
#
import smbus,time

def MPU6050_start():
    # reset all sensors
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x80)
    time.sleep(0.1)
    bus.write_byte_data(MPU6050_ADDR,PWR_MGMT_1,0x00)
    time.sleep(0.1)
    # power management and crystal settings
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    # alter sample rate (stability)
    samp_rate_div = 0 # sample rate = 8 kHz/(1+samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    #Write to Configuration register
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    #Write to Gyro configuration register
    gyro_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    gyro_config_vals = [250.0,500.0,1000.0,2000.0] # degrees/sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    #Write to Accel configuration register
    accel_config_sel = [0b00000,0b01000,0b10000,0b11000] # byte registers
    accel_config_vals = [2.0,4.0,8.0,16.0] # g (g = 9.81 m/s^2)
    accel_indx = 0
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # interrupt register (related to overflow of data [FIFO])
    bus.write_byte_data(MPU6050_ADDR,INT_PIN_CFG,0x22)
    time.sleep(0.1)
    # enable the AK8963 magnetometer in pass-through mode
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx],accel_config_vals[accel_indx]
    
def read_raw_bits(register):
    # read accel and gyro values
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register+1)

    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    
    # convert to +- value
    if(value > 32768):
        value -= 65536
    return value

def mpu6050_conv():
    # raw acceleration bits
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)

    # raw temp bits
##    t_val = read_raw_bits(TEMP_OUT_H) # uncomment to read temp
    
    # raw gyroscope bits
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)

    #convert to acceleration in g and gyro dps
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens

    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens

##    temp = ((t_val)/333.87)+21.0 # uncomment and add below in return
    return a_x,a_y,a_z,w_x,w_y,w_z

def AK8963_start():
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x0F)
    time.sleep(0.1)
    coeff_data = bus.read_i2c_block_data(AK8963_ADDR,AK8963_ASAX,3)
    AK8963_coeffx = (0.5*(coeff_data[0]-128)) / 256.0 + 1.0
    AK8963_coeffy = (0.5*(coeff_data[1]-128)) / 256.0 + 1.0
    AK8963_coeffz = (0.5*(coeff_data[2]-128)) / 256.0 + 1.0
    time.sleep(0.1)
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,0x00)
    time.sleep(0.1)
    AK8963_bit_res = 0b0001 # 0b0001 = 16-bit
    AK8963_samp_rate = 0b0110 # 0b0010 = 8 Hz, 0b0110 = 100 Hz
    AK8963_mode = (AK8963_bit_res <<4)+AK8963_samp_rate # bit conversion
    bus.write_byte_data(AK8963_ADDR,AK8963_CNTL,AK8963_mode)
    time.sleep(0.1)
    return [AK8963_coeffx,AK8963_coeffy,AK8963_coeffz] 
    
def AK8963_reader(register):
    # read magnetometer values
    low = bus.read_byte_data(AK8963_ADDR, register-1)
    high = bus.read_byte_data(AK8963_ADDR, register)
    # combine higha and low for unsigned bit value
    value = ((high << 8) | low)
    # convert to +- value
    if(value > 32768):
        value -= 65536
    
    return value

def AK8963_conv():
    # raw magnetometer bits
    while 1:
##        if ((bus.read_byte_data(AK8963_ADDR,AK8963_ST1) & 0x01))!=1:
##            return 0,0,0
        mag_x = AK8963_reader(HXH)
        mag_y = AK8963_reader(HYH)
        mag_z = AK8963_reader(HZH)

        # the next line is needed for AK8963
        if (bus.read_byte_data(AK8963_ADDR,AK8963_ST2)) & 0x08!=0x08:
            break
        
    #convert to acceleration in g and gyro dps
##    m_x = AK8963_coeffs[0]*(mag_x/(2.0**15.0))*mag_sens
##    m_y = AK8963_coeffs[1]*(mag_y/(2.0**15.0))*mag_sens
##    m_z = AK8963_coeffs[2]*(mag_z/(2.0**15.0))*mag_sens
    m_x = (mag_x/(2.0**15.0))*mag_sens
    m_y = (mag_y/(2.0**15.0))*mag_sens
    m_z = (mag_z/(2.0**15.0))*mag_sens
    return m_x,m_y,m_z

def BMP280_start():
    # collect the temp/pressure compensation parameters
    bus.write_byte_data(BMP280_ADDR, 0xE0,0xB6) # soft reset the module first
    time.sleep(0.1)
    b1 = bus.read_i2c_block_data(BMP280_ADDR, 0x88, 24) # read 24-bytes of calib offsets
    time.sleep(0.1)
    # temp compensation coeffs
    dig_T1 = b1[1] * 256 + b1[0]
    dig_T2 = b1[3] * 256 + b1[2]
    if dig_T2 > 32767 :
        dig_T2 -= 65536
    dig_T3 = b1[5] * 256 + b1[4]
    if dig_T3 > 32767 :
        dig_T3 -= 65536
        
    # pressure compensation coeffs
    dig_P1 = b1[7] * 256 + b1[6]
    dig_P2 = b1[9] * 256 + b1[8]
    if dig_P2 > 32767 :
        dig_P2 -= 65536
    dig_P3 = b1[11] * 256 + b1[10]
    if dig_P3 > 32767 :
        dig_P3 -= 65536
    dig_P4 = b1[13] * 256 + b1[12]
    if dig_P4 > 32767 :
        dig_P4 -= 65536
    dig_P5 = b1[15] * 256 + b1[14]
    if dig_P5 > 32767 :
        dig_P5 -= 65536
    dig_P6 = b1[17] * 256 + b1[16]
    if dig_P6 > 32767 :
        dig_P6 -= 65536
    dig_P7 = b1[19] * 256 + b1[18]
    if dig_P7 > 32767 :
        dig_P7 -= 65536
    dig_P8 = b1[21] * 256 + b1[20]
    if dig_P8 > 32767 :
        dig_P8 -= 65536
    dig_P9 = b1[23] * 256 + b1[22]
    if dig_P9 > 32767 :
        dig_P9 -= 65536

    bus.write_byte_data(BMP280_ADDR, 0xF4, int('00100111',2)) # set the highest sample rate
    bus.write_byte_data(BMP280_ADDR, 0xF5, int('01000000',2)) # IIR filter and standby
    time.sleep(0.5)
    
    return [dig_T1,dig_T2,dig_T3],[dig_P1,dig_P2,dig_P3,dig_P4,dig_P5,
                                   dig_P6,dig_P7,dig_P8,dig_P9]

def BMP280_conv():
    # reading 8 bytes from the BMP280 data register (0xF7)
    data = bus.read_i2c_block_data(BMP280_ADDR, BMP280_PRESS, 8)

    # Convert pressure and temperature data to 19-bits
    adc_p = ((data[0] * 65536) + (data[1] * 256) + (data[2] & 0xF0)) / 16
    adc_t = ((data[3] * 65536) + (data[4] * 256) + (data[5] & 0xF0)) / 16

    # Temperature offset calculations
    var1 = ((adc_t) / 16384.0 - (temp_comp[0]) / 1024.0) * (temp_comp[1])
    var2 = (((adc_t) / 131072.0 - (temp_comp[0]) / 8192.0) * ((adc_t)/131072.0 -\
                                                              (temp_comp[0])/8192.0)) * (temp_comp[2])
    t_fine = (var1 + var2)
    cTemp = (var1 + var2) / 5120.0

    # Pressure offset calculations
    var1 = (t_fine / 2.0) - 64000.0
    var2 = var1 * var1 * (pres_comp[5]) / 32768.0
    var2 = var2 + var1 * (pres_comp[4]) * 2.0
    var2 = (var2 / 4.0) + ((pres_comp[3]) * 65536.0)
    var1 = ((pres_comp[2]) * var1 * var1 / 524288.0 + ( pres_comp[1]) * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * (pres_comp[0])
    p = 1048576.0 - adc_p
    p = (p - (var2 / 4096.0)) * 6250.0 / var1
    var1 = (pres_comp[8]) * p * p / 2147483648.0
    var2 = p * (pres_comp[7]) / 32768.0
    pressure = (p + (var1 + var2 + (pres_comp[6])) / 16.0) / 100
    return cTemp,pressure
    
# MPU6050 Registers
MPU6050_ADDR = 0x68
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C
INT_PIN_CFG  = 0x37
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
TEMP_OUT_H   = 0x41
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
#AK8963 registers
AK8963_ADDR   = 0x0C
AK8963_ST1    = 0x02
HXH          = 0x04
HYH          = 0x06
HZH          = 0x08
AK8963_ST1   = 0x02
AK8963_ST2   = 0x09
AK8963_CNTL  = 0x0A
AK8963_ASAX = 0x10

mag_sens = 4800.0 # magnetometer sensitivity: 4800 uT
#BMP280 registers
BMP280_ADDR =  0x76
BMP280_PRESS = 0xF7

# start I2C driver
bus = smbus.SMBus(1) # start comm with i2c bus
time.sleep(0.1)
gyro_sens,accel_sens = MPU6050_start() # instantiate gyro/accel
##gyro_sens,accel_sens = initMPU9250() # instantiate gyro/accel
time.sleep(0.1)
AK8963_coeffs = AK8963_start() # instantiate magnetometer
time.sleep(0.1)
##temp_comp,pres_comp = BMP280_start() # start the bmp280 pressure/temp sensor
##time.sleep(0.5)
