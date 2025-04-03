import time
# import board
import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

# Set numbering
gpio.setmode(gpio.BOARD)
scl_pin = 3
sda_pin = 2

# Create the I2C bus
# i2c = busio.I2C(board.SCL, board.SDA)
i2c = busio.I2C(scl_pin, sda_pin)

# Set i2c address

# Create the ADC object using the I2C bus
ads = ADS.ADS1015(i2c)

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

# Create differential input between channel 0 and 1
# chan = AnalogIn(ads, ADS.P0, ADS.P1)

print("{:>5}\t{:>5}".format('raw', 'v'))

while True:
    print("{:>5}\t{:>5.3f}".format(chan.value, chan.voltage))
    time.sleep(0.25)