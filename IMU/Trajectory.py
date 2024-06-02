import time
import board
import adafruit_fxas21002c
import adafruit_fxos8700
import math
import csv
import threading

# Create sensor object, communicating over the board's default I2C bus
i2c = board.I2C()
Gyro = adafruit_fxas21002c.FXAS21002C(i2c, gyro_range=adafruit_fxas21002c.GYRO_RANGE_2000DPS)
Acc_Mag = adafruit_fxos8700.FXOS8700(i2c, accel_range=adafruit_fxos8700.ACCEL_RANGE_8G)

def record_data_to_csv(file_path):
    with open(file_path, 'w', newline='') as csvfile:
        # ['accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 'gyro_x', 'gyro_y', 'gyro_z'] , 'mag_x', 'mag_y', 'mag_z'
        fieldnames = ['gyro_x', 'gyro_y', 'gyro_z', 'accel_x', 'accel_y', 'accel_z', 'mag_x', 'mag_y', 'mag_z']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        
        data_count = 0  # Untuk menghitung jumlah data yang sudah direkam
        
        try:
            # Main loop will read the gyroscope values every second and print them out.
            while True:
                # Read gyroscope.
                accel_x, accel_y, accel_z = Acc_Mag.accelerometer
                gyro_x, gyro_y, gyro_z = Gyro.gyroscope
                mag_x, mag_y, mag_z = Acc_Mag.magnetometer

                # Convert acceleration values to g (gravity)
                accel_x_g = accel_x / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
                accel_y_g = accel_y / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
                accel_z_g = accel_z / adafruit_fxos8700._SENSORS_GRAVITY_STANDARD
                
                # Convert radians/s to degrees/s.
                gyro_x_deg = math.degrees(gyro_x)
                gyro_y_deg = math.degrees(gyro_y)
                gyro_z_deg = math.degrees(gyro_z)

                # Convert magnetometer values to Gauss
                mag_x_G = mag_x / 100.0
                mag_y_G = mag_y / 100.0
                mag_z_G = mag_z / 100.0

                # Print values.
                print("{0:.3f}, {1:.3f}, {2:.3f},{3:.3f}, {4:.3f}, {5:.3f}, {6:.3f}, {7:.3f}, {8:.3f},".format(
                    gyro_x_deg, gyro_y_deg, gyro_z_deg,accel_x_g, accel_y_g, accel_z_g, mag_x_G, mag_y_G, mag_z_G
                ))

                # Tulis data ke file CSV setiap 10 detik
                if data_count % 2 == 0:
                    writer.writerow({'gyro_x': gyro_x_deg,
                                    'gyro_y': gyro_y_deg,
                                    'gyro_z': gyro_z_deg,
                                    'accel_x': accel_x_g,
                                    'accel_y': accel_y_g,
                                    'accel_z': accel_z_g,
                                    'mag_x': mag_x_G,
                                    'mag_y': mag_y_G,
                                    'mag_z': mag_z_G
                                    })
                    
                data_count += 1

                # Delay for a second.
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Recording stopped.")

if __name__ == "__main__":
    # Start recording data
    record_thread = threading.Thread(target=record_data_to_csv, args=('Trajectory_Data.csv',))
    record_thread.daemon = True
    record_thread.start()
    
    while True:
        time.sleep(0.01)

    # try:
    #     while True:
    #         time.sleep(0.01)
    # except KeyboardInterrupt:
    #     print("Main program stopped.")
    #     record_thread.join()