######################################################
# Copyright (c) 2021 Maker Portal LLC
# Author: Joshua Hrisko
######################################################
#
# This code reads data from the MPU9250/MPU9265 board
# (MPU6050 - accel/gyro, AK8963 - mag)
# and solves for calibration coefficients for the
# gyroscope and uses them to integrate over a test
# rotation, which approximates angular displacement
#
#
######################################################
#
# wait 5-sec for IMU to connect
import time,sys
sys.path.append("../")
t0 = time.time()
start_bool = False # if IMU start fails - stop calibration
while time.time()-t0<5:
    try: 
        from mpu9250_i2c import *
        start_bool = True
        break
    except:
        continue
import numpy as np
import csv,datetime
import matplotlib.pyplot as plt
from scipy.integrate import cumtrapz

time.sleep(2) # wait for MPU to load and settle
# 
#####################################
# Gyro calibration (Steady)
#####################################
#
def get_gyro():
    _,_,_,wx,wy,wz = mpu6050_conv() # read and convert gyro data
    return wx,wy,wz

def gyro_cal():
    print("-"*50)
    print('Gyro Calibrating - Keep the IMU Steady')
    [get_gyro() for ii in range(0,cal_size)] # clear buffer before calibration
    mpu_array = []
    gyro_offsets = [0.0,0.0,0.0]
    while True:
        try:
            wx,wy,wz = get_gyro() # get gyro vals
        except:
            continue

        mpu_array.append([wx,wy,wz])

        if np.shape(mpu_array)[0]==cal_size:
            for qq in range(0,3):
                gyro_offsets[qq] = np.mean(np.array(mpu_array)[:,qq]) # average
            break
    print('Gyro Calibration Complete')
    return gyro_offsets

if __name__ == '__main__':
    if not start_bool:
        print("IMU not Started - Check Wiring and I2C")
    else:
        #
        ###################################
        # Gyroscope Offset Calculation
        ###################################
        #
        gyro_labels = ['\omega_x','\omega_y','\omega_z'] # gyro labels for plots
        cal_size = 500 # points to use for calibration
        gyro_offsets = gyro_cal() # calculate gyro offsets
        #
        ###################################
        # Record new data 
        ###################################
        #
        input("Press Enter and Rotate Gyro 360 degrees")
        print("Recording Data...")
        record_time = 5 # how long to record
        data,t_vec = [],[]
        t0 = time.time()
        while time.time()-t0<record_time:
            data.append(get_gyro())
            t_vec.append(time.time()-t0)
        samp_rate = np.shape(data)[0]/(t_vec[-1]-t_vec[0]) # sample rate
        print("Stopped Recording\nSample Rate: {0:2.0f} Hz".format(samp_rate))
        #
        ##################################
        # Offset and Integration of gyro
        # and plotting results
        ##################################
        #
        rot_axis = 2 # axis being rotated (2 = z-axis)
        data_offseted = np.array(data)[:,rot_axis]-gyro_offsets[rot_axis]
        integ1_array = cumtrapz(data_offseted,x=t_vec) # integrate once in time
        #
        # print out reuslts
        print("Integration of {} in {}".format(gyro_labels[rot_axis],
                       gyro_labels[rot_axis].split("_")[1])+\
              "-dir: {0:2.2f}m".format(integ1_array[-1]))
        #
        # plotting routine
        plt.style.use('ggplot')
        fig,axs = plt.subplots(2,1,figsize=(12,9))
        axs[0].plot(t_vec,data_offseted,label="$"+gyro_labels[rot_axis]+"$")
        axs[1].plot(t_vec[1:],integ1_array,
                    label=r"$\theta_"+gyro_labels[rot_axis].split("_")[1]+"$")
        [axs[ii].legend(fontsize=16) for ii in range(0,len(axs))]
        axs[0].set_ylabel('Angular Velocity, $\omega_{}$ [$^\circ/s$]'.format(gyro_labels[rot_axis].\
                                           split("_")[1]),fontsize=16)
        axs[1].set_ylabel(r'Rotation, $\theta_{}$ [$^\circ$]'.format(gyro_labels[rot_axis].\
                                               split("_")[1]),fontsize=16)
        axs[1].set_xlabel('Time [s]',fontsize=16)
        axs[0].set_title('Gyroscope Integration over 180$^\circ$ Rotation',
                         fontsize=18)
        fig.savefig('gyroscope_integration_180deg_rot.png',dpi=300,
                    bbox_inches='tight',facecolor='#FFFFFF')
        plt.show()
