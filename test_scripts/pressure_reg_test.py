# Import Packages
import xlsxwriter
import robot_fxns as robot
import time
import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import gpiozero
# from rtplot import client
import math

# Set pin numbering style
gpio.setmode(gpio.BOARD)

# Initialize ADC
scl_pin = 3
sda_pin = 2
i2c = busio.I2C(scl_pin, sda_pin)
ads = ADS.ADS1015(i2c)
ads.gain = 2/3 # Set adc max value to 6.144 V
chan = AnalogIn(ads, ADS.P1) # Create single-ended input on channel 0

# Initialize PWM
pwm_freq = 100.0
pressure_pwm_pin = 13
pressure_pwm = gpiozero.PWMOutputDevice(pressure_pwm_pin, active_high=True, initial_value=0.0, frequency=pwm_freq)

# Define Sleep Time
sleep_time = 2

try:
    while(True):
        pressure_pwm.value = 0.01
        print(str(int(pressure_pwm.value*100)) + '% pwm - ' + '= ' + str(60*pressure_pwm.value) + ' psi\n')
        print('ADC Voltage: ' + str(chan.voltage) + ' Volts\n')
        time.sleep(sleep_time)
        pressure_pwm.value = 0.25 
        print(str(int(pressure_pwm.value*100)) + '% pwm - ' + '= ' + str(60*pressure_pwm.value) + ' psi\n')
        print('ADC Voltage: ' + str(chan.voltage) + ' Volts\n')
        time.sleep(sleep_time)
        pressure_pwm.value = 0.5 
        print(str(int(pressure_pwm.value*100)) + '% pwm - ' + '= ' + str(60*pressure_pwm.value) + ' psi\n')
        print('ADC Voltage: ' + str(chan.voltage) + ' Volts\n')
        time.sleep(sleep_time)
        pressure_pwm.value = 0.75
        print(str(int(pressure_pwm.value*100)) + '% pwm - ' + '= ' + str(60*pressure_pwm.value) + ' psi\n')
        print('ADC Voltage: ' + str(chan.voltage) + ' Volts\n')
        time.sleep(sleep_time)
        pressure_pwm.value = 0.99 
        print(str(int(pressure_pwm.value*100)) + '% pwm - ' + '= ' + str(60*pressure_pwm.value) + ' psi\n')
        print('ADC Voltage: ' + str(chan.voltage) + ' Volts\n')
        time.sleep(sleep_time)
except KeyboardInterrupt:
    pressure_pwm.value = 0.0
    pressure_pwm.close()
    print('\nStopping PWM')
