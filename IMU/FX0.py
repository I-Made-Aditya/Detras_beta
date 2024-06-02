import time
import board
import math
import serial
import busio
import threading

import numpy as np
import adafruit_fxos8700
import adafruit_fxas21002c

# Inisialisasi port untuk pengirim USART
ser = serial.Serial("/dev/ttyS0", 
                    baudrate=115200,
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE, 
                    bytesize=serial.EIGHTBITS, 
                    timeout=1
                )     

i2c = board.I2C()  # Menggunakan board.SCL dan board.SDA

AM_sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)
G_sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)

while True:
    # Read acceleration, magnetometer, gyroscope.
    accel_x, accel_y, accel_z = AM_sensor.accelerometer
    gyro_x, gyro_y, gyro_z = G_sensor.gyroscope
    mag_x, mag_y, mag_z = AM_sensor.magnetometer

    # Convert acceleration values to g (gravity)
    accel_x_g = accel_x / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
    accel_y_g = accel_y / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
    accel_z_g = accel_z / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
                
    # # Convert radians/s to degrees/s.
    # gyro_x_deg = math.degrees(gyro_x)
    # gyro_y_deg = math.degrees(gyro_y)
    # gyro_z_deg = math.degrees(gyro_z)

    # # Convert magnetometer values to Gauss
    # mag_x_G = mag_x / 100.0
    # mag_y_G = mag_y / 100.0
    # mag_z_G = mag_z / 100.0

    print("Acc_X:{0:0.3f}, Acc_Y:{1:0.3f}, Acc_Z:{2:0.3f}\t\t"  #get Accel data
            "Gyro_X:{3:0.3f}, Gyro_Y:{4:0.3f}, Gyro_Z:{5:0.3f}\t" #get Gyro data
            "Mag_X:{6:0.3f}, Mag_Y:{7:0.3f}, Mag_Z:{8:0.3f}" #get Mag data
            .format(accel_x_g, accel_y_g, accel_z_g, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z))
    
    ser.write(("{0:0.3f},{1:0.3f},{2:0.3f},{3:0.3f},{4:0.3f},{5:0.3f},{6:0.3f},{7:0.3f},{8:0.3f}\n"
                .format(accel_x_g, accel_y_g, accel_z_g, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z))
                .encode())
    time.sleep(0.01)