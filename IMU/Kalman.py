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

def convert_acc(acc):
    """Convert raw accelerometer measurements in roll pitch yaw
    https://stackoverflow.com/questions/3755059/3d-accelerometer-calculate-the-orientation
    The yaw equation comes from here https://robotics.stackexchange.com/questions/14305/yaw-from-accelerometer-no-so-what-do-these-equations-actually-mean
    If you have a magnetometer check out https://habr.com/en/post/499190/
    Args:
        acc (np.array): Array containing the three raw measurements on the accelerometer along
        x, y, z
    Returns:
        np.array: numpy array with [roll, pitch, yaw]
    """
    roll = math.atan2(acc[1], acc[2]) * (180 / math.pi)
    pitch = math.atan2(-acc[0], (math.sqrt((acc[1]*acc[1]) + (acc[2]*acc[2])))) * (180 / math.pi)
    yaw = math.atan(math.sqrt((acc[1]*acc[1]) +(acc[0]*acc[0])) / acc[2]) * (180 / math.pi)
    
    return np.array([roll, pitch, yaw])

def convert_gyro(orientation, gyro):
    """Convert raw gyroscope measurements in roll pitch yaw rate of change
    Args:
        orientation (np.array): Actual position of IMU as a numpy array containing [roll, pitch, yaw]
        gyro (np.array): Raw measurements of the gyroscope as a numpy array [Gx, Gy, Gz]
    Returns:
        np.array: numpy array with [roll, pitch, yaw] rates of change
    """
    # Transform gyro measurement into roll pitch yaw
    roll = orientation[0][0]  #phi
    pitch = orientation[1][0]  #theta
    
    # from http://www.chrobotics.com/library/understanding-euler-angles
    mat = np.array([[1, math.sin(roll) * math.tan(pitch), math.cos(roll) * math.tan(pitch)],
                    [0, math.cos(roll), -math.sin(roll)],
                    [0, math.sin(roll) / math.cos(pitch), math.cos(roll) / math.cos(pitch)]])
    
    rate_change_gyro = np.matmul(mat, gyro)
    return rate_change_gyro

class Kalman_IMU(BK.BaseKF):
    """Kalman filter tracking the orientation of an IMU
    """
    def __init__(self, z0: np.array, r: np.array, q: np.array, pval) -> None:
        super().__init__(z0, r, q, pval=pval)
        
        self.gyroconv = list()
    
    def f(self, x, u):
        """Get as input the gyroscope data and integrate it to give the new angles of the IMU
        Args:
            x (np.array): IMU roll pitch yaw
            u (np.array): Gyrosocope rate of changes for roll pitch and yaw and time difference between
            this measurement and the last one [roll, pitch, yaw, time_diff]
        Returns:
            (np.array, np.array): tuple containing the new x and the stae model matrix A 
        """
        
        rate_change_gyro = convert_gyro(x, u[0:-1])
        rate_change_gyro = np.reshape(rate_change_gyro, (-1, 1))
        self.gyroconv.append(rate_change_gyro)
        
        B = np.diag(np.full(u.shape[0] - 1, u[-1]))
        A = np.eye(self.n)

        x_n =  np.matmul(A, x) + np.matmul(B, rate_change_gyro)
        
        # Here returning A---i.e. identity matrix---instead of the jacobian leads to the normal kalman filter
        # instead of the EKF
        return x_n, A
    
    def h(self, x):
        # Observation function is identity
        return x, np.eye(self.n)
    
    def update(self, z):
        acc = convert_acc(z)
        acc = np.reshape(acc, (-1, 1))
        #Call update with accelerometer data
        super().update(acc)
        
while True:
    # Read acceleration, magnetometer, gyroscope.
    accel_x, accel_y, accel_z = AM_sensor.accelerometer
    # mag_x, mag_y, mag_z = sensor.magnetometer
    gyro_x, gyro_y, gyro_z = G_sensor.gyroscope

    Apitch = math.atan2(accel_y, accel_z) * 180.0 / math.pi
    Aroll = math.atan2(-accel_x, math.sqrt(math.pow(accel_y, 2) + math.pow(accel_z, 2))) * 180.0 / math.pi 

    #SEND DATA TO SERIAL with comma separator!!
    ser.write(("{0:0.3f},{1:0.3f},{2:0.3f},{3:0.3f},{4:0.3f},{5:0.3f},{6:0.3f},{7:0.3f},{8:0.3f}\n"
               .format(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw))
            .encode()
      )
    
    
    # Print values.
    print("Acc_X:{0:0.3f}, Acc_Y:{1:0.3f}, Acc_Z:{2:0.3f}\t\t"  #get Accel data
          "Gyro_X:{3:0.3f}, Gyro_Y:{4:0.3f}, Gyro_Z:{5:0.3f}\t\t" #get Gyro data
          "Roll:{6:0.3f}, Pitch:{7:0.3f}, Yaw:{8:0.3f}" #get orientation data
          .format(accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, roll, pitch, yaw))
    
    # Delay for a second.
    time.sleep(0.01)

