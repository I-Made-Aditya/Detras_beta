import time
import serial

ser = serial.Serial("/dev/ttyS0", 
                    baudrate = 115200,
                    parity   = serial.PARITY_NONE, 
                    stopbits = serial.STOPBITS_ONE, 
                    bytesize = serial.EIGHTBITS, 
                    timeout  = 1
                )     

count = 0
while True:
    # ser.write("Test: {0:}".format(count))
    ser.write(("Test " + str(count) + " masuk " + str(count*3) + "\n").encode())
    # ser.write(b'Write: %d ' %(count))
    # ser.write(b'Write: %d\n' %(count*5))
    print("Printed !! ~ " + str(count))
    count += 1
    time.sleep(0.1)

