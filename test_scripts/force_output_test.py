import time
import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import xlsxwriter
import gpiozero

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

# # Initialize PWM
# pwm_freq = 5000.0
# valve_pwm_pin = 18
# valve_pwm = gpiozero.PWMOutputDevice(valve_pwm_pin, active_high=True, initial_value=0.5, frequency=pwm_freq)

# Constants
pwm_value = 0.45
sampling_freq = 250.0 # In Hz
loop_dt = 1.0/sampling_freq

# Create Data saving names
test_time = 10 # In seconds
name = 'force_test_t3'
date = '8_26_25'
trial_name = name + '_' + date + '.xlsx'
save_location = '/home/pi/osseoperception/test_scripts/test_data/8_26_25/'
file_save_path = save_location + trial_name

voltage_vector = []
current_force_vector = []
pwm_value_vector = []

try:
    question = input('Save Data? 1-Yes, 0-No: ')
    # valve_pwm.value = pwm_value
    t_init = time.time()
    # while(time.time()-t_init < test_time):
    while question == '1' or question == '0':
        t1 = time.time() # get current time
        voltage = chan.voltage # read adc voltage
        current_force = (148.26*voltage - 391.9) # convert voltage to force

        # Print force
        print('Force (N): ',current_force)

        # Append data for saving
        voltage_vector.append(voltage)
        current_force_vector.append(current_force)
        # pwm_value_vector.append(valve_pwm.value)

        while (time.time()-t1 < loop_dt):
            pass

    print('Force Testing Complete')
    if question == '1':
        # Save data as excel sheet
        book = xlsxwriter.Workbook(file_save_path)
        sheet1 = book.add_worksheet('Sheet 1')
        sheet1.write(0,0,'ADC Voltage')
        sheet1.write(0,1,'Actual Force')
        sheet1.write(0,2,'PWM Value')

        i = 1
        for index in range(len(voltage_vector)):
            sheet1.write(i,0,voltage_vector[index])
            sheet1.write(i,1,current_force_vector[index])
            sheet1.write(i,2,pwm_value_vector[index])
            i = i+1

        book.close()
except KeyboardInterrupt:
    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    if question == '1':
        # Save data as excel sheet
        book = xlsxwriter.Workbook(file_save_path)
        sheet1 = book.add_worksheet('Sheet 1')
        sheet1.write(0,0,'ADC Voltage')
        sheet1.write(0,1,'Actual Force')
        sheet1.write(0,2,'PWM Value')

        i = 1
        for index in range(len(voltage_vector)):
            sheet1.write(i,0,voltage_vector[index])
            sheet1.write(i,1,current_force_vector[index])
            sheet1.write(i,2,pwm_value_vector[index])
            i = i+1

        book.close()