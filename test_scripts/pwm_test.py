#!/usr/bin/env python3

# import pkgs
import gpiozero
import time
import math

# intialize stuff
freq = 5000.0
valve_pwm_pin = 13
valve_pwm = gpiozero.PWMOutputDevice(valve_pwm_pin, active_high=False, initial_value=0.0, frequency=freq)
sleep_time = 3
# time.sleep(1)

try:
    while(True):
        # valve_pwm.value = 0.0 #0.0 is full power retract
        # print(str(int(valve_pwm.value*100)) + '% pwm - ' + str(int(50 - valve_pwm.value*100)) + ' percent open - retract')
        # time.sleep(sleep_time)
        valve_pwm.value = 0.5
        print('50% pwm - valve closed')
        time.sleep(sleep_time)
        # valve_pwm.value = 1.0 #1.0 is full power extend
        # print(str(int(valve_pwm.value*100)) + '% pwm - ' + str(int(valve_pwm.value*100 - 50)) + ' percent open - extend')
        # time.sleep(sleep_time)
except KeyboardInterrupt:
    valve_pwm.value = 0.0
    valve_pwm.close()
    print('\nStopping PWM')

    
