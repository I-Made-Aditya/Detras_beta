import time
import board
import adafruit_fxas21002c
import math

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()
sensor = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)

# Main loop will read the gyroscope values every second and print them out.
while True:
    # Read gyroscope.
    gyro_x, gyro_y, gyro_z = sensor.gyroscope
    # Convert radians/s to degrees/s.
    gyro_x_deg = math.degrees(gyro_x)
    gyro_y_deg = math.degrees(gyro_y)
    gyro_z_deg = math.degrees(gyro_z)
    # Print values.
    print(
        "Gyroscope X (deg/s): {0:0.3f}, Gyroscope Y (deg/s): {1:0.3f}, Gyroscope Z (deg/s): {2:0.3f}".format(
            gyro_x_deg, gyro_y_deg, gyro_z_deg
        )
    )
    # Delay for a second.
    time.sleep(0.01)
