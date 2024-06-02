import time
import board
import math
import serial
import busio

import numpy as np
import adafruit_fxos8700
import adafruit_fxas21002c

#inisialisasi port untuk pengirim USART
ser = serial.Serial("/dev/ttyS0", 
                    baudrate = 115200,
                    parity   = serial.PARITY_NONE, 
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS, 
                    timeout  = 1
                )     

# Create sensor object, communicating over the board's default I2C bus

i2c = board.I2C()  # uses board.SCL and board.SDA

AM_sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)
G_sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_1000DPS)

class MadgwickFilter:
    def __init__(self, beta=0.041):
        self.beta = beta
        self.q0 = 1.0
        self.q1 = 0.0
        self.q2 = 0.0
        self.q3 = 0.0

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        beta = self.beta

        recipNorm = 1.0 / math.sqrt(ax * ax + ay * ay + az * az)
        ax *= recipNorm
        ay *= recipNorm
        az *= recipNorm

        s0 = 2.0 * (q1 * q3 - q0 * q2)
        s1 = 2.0 * (q0 * q1 + q2 * q3)
        s2 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
        s3 = -q1 * q3 + q0 * q2

        qDot1 = 0.5 * (-s1 * gx - s2 * gy - s3 * gz)
        qDot2 = 0.5 * (s0 * gx + s2 * gz - s3 * gy)
        qDot3 = 0.5 * (s0 * gy - s1 * gz + s2 * gx)
        qDot4 = 0.5 * (s0 * gz + s1 * gy - s2 * gx)

        q0 += qDot1 * dt
        q1 += qDot2 * dt
        q2 += qDot3 * dt
        q3 += qDot4 * dt

        recipNorm = 1.0 / math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q0 = q0 * recipNorm
        self.q1 = q1 * recipNorm
        self.q2 = q2 * recipNorm
        self.q3 = q3 * recipNorm

    def to_euler(self):
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        roll = math.atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1**2 + q2**2))
        pitch = math.asin(2.0 * (q0 * q2 - q3 * q1))
        yaw = math.atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2**2 + q3**2))
        return yaw, pitch, roll
    
while True:
    # Read acceleration, magnetometer, gyroscope.
    accel_x, accel_y, accel_z = AM_sensor.accelerometer
    # mag_x, mag_y, mag_z = sensor.magnetometer
    gyro_x, gyro_y, gyro_z = G_sensor.gyroscope

    # Initialize Madgwick filter
    filter = MadgwickFilter()

    # Update filter with sample data
    filter.update(gyro_x, gyro_y, gyro_z,
            accel_x, accel_y, accel_z,
            dt=0.01)  # Assuming a timestep of 0.01 seconds

    # Convert quaternion to Euler angles
    yaw, pitch, roll = filter.to_euler()

    #SEND DATA TO SERIAL with comma separator!!
    ser.write(("{0:0.3f},{1:0.3f},{2:0.3f},{3:0.3f},{4:0.3f},{5:0.3f},{6:0.3f},{7:0.3f},{8:0.3f}\n"
               .format(accel_x, accel_y, accel_z,
                       gyro_x, gyro_y, gyro_z,
                       math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
            .encode()
        )
    
    
    # Print values.
    print("Acc_X:{0:0.3f}, Acc_Y:{1:0.3f}, Acc_Z:{2:0.3f}\t\t"  #get Accel data
          "Gyro_X:{3:0.3f}, Gyro_Y:{4:0.3f}, Gyro_Z:{5:0.3f}\t\t" #get Gyro data
          "Roll:{6:0.3f}, Pitch:{7:0.3f}, Yaw:{8:0.3f}" #get orientation data
          .format(accel_x, accel_y, accel_z,
                  gyro_x, gyro_y, gyro_z,
                  math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        )
    
    # Delay for a second.
    time.sleep(0.01)

