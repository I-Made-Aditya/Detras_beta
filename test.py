import time
import serial

tes = " TEST!!!\n"
data = 0

ser = serial.Serial("/dev/ttyS0", 
                    baudrate=9600,
                    parity=serial.PARITY_NONE, 
                    stopbits=serial.STOPBITS_ONE, 
                    bytesize=serial.EIGHTBITS, 
                    timeout=1
                )
while True:    
    ser.write((str(data) + tes).encode())
    print(str(data) + tes)
    data +=1
    time.sleep(1)
