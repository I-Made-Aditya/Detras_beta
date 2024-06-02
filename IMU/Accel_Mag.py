import time
import board
import adafruit_fxos8700

i2c = board.I2C()
sensor = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)

while True:
    accel_x, accel_y, accel_z = sensor.accelerometer
    mag_x, mag_y, mag_z = sensor.magnetometer
    
    # Convert acceleration values to g (gravity)
    accel_x_g = accel_x / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
    accel_y_g = accel_y / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
    accel_z_g = accel_z / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
    
    # Convert magnetometer values to Gauss
    mag_x_G = mag_x / 100.0
    mag_y_G = mag_y / 100.0
    mag_z_G = mag_z / 100.0
    
    # Print values in the desired format
    # print("Accelerometer X (g), Accelerometer Y (g), Accelerometer Z (g), Magnetometer X (G), Magnetometer Y (G), Magnetometer Z (G)")
    print("{0:.3f}, {1:.3f}, {2:.3f}, {3:.3f}, {4:.3f}, {5:.3f}".format(
        accel_x_g, accel_y_g, accel_z_g, mag_x_G, mag_y_G, mag_z_G
    ))
    
    time.sleep(0.01)
