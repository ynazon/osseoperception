import time
import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import xlsxwriter

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
ads.gain = 2/3

# Create single-ended input on channel 0
chan = AnalogIn(ads, ADS.P0)

# Create Data saving names
test_time = 0.5 # In seconds
name = '5kg_wshld_lc_char_t1'
date = '8_25_25'
trial_name = name + '_' + date + '.xlsx'
save_location = '/home/pi/osseoperception/test_scripts/test_data/8_25_25/'
file_save_path = save_location + trial_name

voltage_vector = []

t_init = time.time()
while(time.time()-t_init < test_time):
    voltage = chan.voltage # read adc voltage

    # Append data for saving
    voltage_vector.append(voltage)

print('Characterization Complete')
# Save data as excel sheet
book = xlsxwriter.Workbook(file_save_path)
sheet1 = book.add_worksheet('Sheet 1')
sheet1.write(0,0,'ADC Voltage')

i = 1
for index in range(len(voltage_vector)):
    sheet1.write(i,0,voltage_vector[index])
    i = i+1

book.close()