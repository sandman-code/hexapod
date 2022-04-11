import serial
import time
from datetime import datetime

def command(ser,command):
    start_time = datetime.now()
    ser.flushInput()
    time.sleep(1)
    ser.write(str.encode(command))
    time.sleep(1)

    while True:
        line = ser.read(10)
        print(line)

        if line == b'ok\n':
            break

