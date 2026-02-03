#!/usr/bin/env python3

import serial
import time

from pynput import keyboard

def on_press(key):
    try:
        print(f'Alphanumeric key {key.char} pressed')
    except AttributeError:
        print(f'Special key {key} pressed')

# Configure the serial port
# Replace '/dev/ttyUSB0' with the actual port name of your device
ser = serial.Serial(
    port='/dev/ttyAMA0',  # or /dev/ttyACM0, etc.
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

print(f"Connected to {ser.name}")

a = 0
b = 0

try:
    while True:
        # # Send data
        # ser.write(b"Hello from Raspberry Pi!\n")
        # ser.write(str.encode(str(a)) + str.encode("\n"))
        # # data_to_send = f"{a},{b}\n"
        # # data_to_send = 'w'
        # # ser.write(data_to_send.encode('utf-8'))
        # # ser.write(str.encode({str(a),str(b)}))
        # # print("Sent: Hello from Raspberry Pi!")
        # # a = a + 1
        # # print(f"Count: {a}")

        #Read data
        if ser.in_waiting > 0:
            received_data = ser.readline().decode('utf-8').strip()
            print(f"Received: {received_data}")

        # Collect events until released
        with keyboard.Listener(
                on_press=on_press,
                ) as listener:
            listener.join() 
        
        # received_data = ser.readline().decode('utf-8').strip()
        # print(f"Received: {received_data}")

        time.sleep(2)

except KeyboardInterrupt:
    print("Exiting serial communication.")
finally:
    ser.close()
    print("Serial port closed.")