######################################################
# Copyright (c) 2020 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag) to measure the
# approximate sample rate of the IMU via I2C on the RPi
#
#
######################################################
#
import time
t0 = time.time()
start_bool = False # boolean for connection
while (time.time()-t0)<5: # wait for 5-sec to connect to IMU
    try:
        from mpu9250_i2c import *
        start_bool = True # True for forthcoming loop
        break 
    except:
        continue
#
t0 = time.time()
iter_ii = 0
data_array = []
sample_count = 100
while iter_ii<sample_count:
    if start_bool==False: # make sure the IMU was started
        print("IMU not Started, Check Wiring") # check wiring if error
        break
    ##################################
    # Reading and Printing IMU values
    ##################################
    #
    try:
        ax,ay,az,wx,wy,wz = mpu6050_conv() # read and convert mpu6050 data
        mx,my,mz = AK8963_conv() # read and convert AK8963 magnetometer data
        data_array.append([ax,ay,az,wx,wy,wz,mx,my,mz])
        iter_ii+=1
    except:
        continue 

t1 = time.time()
print("Sample Rate: {0:2.0f}Hz".format(len(data_array)/(t1-t0)))
