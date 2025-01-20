#!/usr/bin/env python3

#Configure RS232 for sensor

import serial

ser = serial.Serial('COM3', baudrate=115200, timeout=1)  # Adjust port and baudrate
ser.write(b'AT+GOD\r\n')  # Command to get one data package
response = ser.read(100)  # Adjust byte size
print(response.decode())
ser.close()