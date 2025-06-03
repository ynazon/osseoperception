#!/usr/bin/env python3

#This is a test script for controlling the WR pneumatic powered perturbation robot

# Import Packages
import xlsxwriter
import robot_fxns as robot
import time
import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import gpiozero
from rtplot import client
import math
# import numpy as np

# Set pin numbering style
gpio.setmode(gpio.BOARD)

# ### Initialize Real Time Plotting
# # Step 1: Configure the IP address
# client.configure_ip('192.168.1.101')

# # Step 2: Initialize the plots
# # Initialize one plot with 1 traces
# client.initialize_plots()

# #Send 1000 datapoints
# # for i in range(1000):

# #     # Step 3: Send the data
# #     # Send the number 5 a thousand times
# #     client.send_array([5])

# Initialize ADC
scl_pin = 3
sda_pin = 2
i2c = busio.I2C(scl_pin, sda_pin)
ads = ADS.ADS1015(i2c)
ads.gain = 2/3 # Set adc max value to 6.144 V
chan = AnalogIn(ads, ADS.P0) # Create single-ended input on channel 0

# Initialize PWM
pwm_freq = 1000.0
valve_pwm_pin = 18
valve_pwm = gpiozero.PWMOutputDevice(valve_pwm_pin, active_high=True, initial_value=0.5, frequency=pwm_freq)

# Initialize Data Saving Variables
testing_flag = False
test_time = 10.0 # In seconds
name = 'test_t33'
date = '6_2_25'
trial_name = name + '_' + date + '.xlsx'
save_location = '/home/pi/osseoperception/test_scripts/test_data/6_2_25/'
file_save_path = save_location + trial_name
print('Trial name is: ',name)

# Initialize Control Variables
sampling_freq = 100.0 # In Hz
loop_dt = 1.0/sampling_freq
desired_force = 5.0 #in Newtons
iterations = 0

desired_force_vector = []#[desired_force]
current_force_vector = []#[0.0]
voltage_vector = []#[0.0]
# sum_of_all_error_vector = [0.0]
# loop_error = [0.0]
# time_step = [0.0]
control_effort_vector = []#[0.0]
time_loop_start = []#[0.0]
differentiation_points = [0.0,0.0,0.0,0.0]

filter_points = [0.0,0.0,0.0,0.0]
filtered_voltage_vector = []
filtered_force_vector = []

previous_error = 0.0
error_sum = 0.0
error_derivative = 0.0

# Create Control Trajectory
traj_start = 0.0
traj_end = 0.0 #2*math.pi
traj_type = 'traj'
traj = robot.make_trajectory(traj_start,traj_end,sampling_freq,test_time,traj_type)

# Control Robot
try:
    # Give input command to start script
    question = input('Set up for perturbation experiment. When you are ready to continue type " 1 " (the number one) without parentheses.')
    t_init = time.time()

    if question == '1':
        if testing_flag == True:
            while(time.time()-t_init < test_time):
            # for value in traj:
                t1 = time.time() # get current time
                voltage = chan.voltage # read adc voltage
                [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
                # convert adc voltage to a force
                current_force = 0.161*math.exp(1.506*voltage) - 0.161
                filtered_force = 0.161*math.exp(1.506*filtered_voltage) - 0.161

                # if voltage >= 0.0 and voltage <= 1.3: 
                #     current_force = (0.2/1.3)*voltage 
                #     filtered_force = (0.2/1.3)*filtered_voltage
                # elif voltage > 1.3 and voltage <= 3.1: 
                #     current_force = 0.4444*voltage - 0.3778
                #     filtered_force = 0.4444*filtered_voltage - 0.3778
                # elif voltage > 3.1: 
                #     current_force = 6.4286*voltage - 18.9286
                #     filtered_force = 6.4286*filtered_voltage - 18.9286

                # get a control effort based on force error    
                [control_effort, previous_error, kp_term, ki_term , kd_term,
                error_sum, error_derivative, differentiation_points] = robot.PID_control(desired_force,filtered_force,previous_error,
                                                                                        error_sum,error_derivative,differentiation_points,loop_dt)
                
                # send control effort as a pwm signal
                valve_pwm.value = control_effort + 0.5  #May have to scale/normalize btwn 0 & 1

                # increase loop counter
                iterations = iterations + 1

                # Append data for saving
                time_loop_start.append(t1-t_init)
                desired_force_vector.append(desired_force)
                current_force_vector.append(current_force)
                voltage_vector.append(voltage)
                control_effort_vector.append(control_effort)
                filtered_voltage_vector.append(filtered_voltage)
                filtered_force_vector.append(filtered_force)

                # Send data to plot
                # client.send_array(current_force_vector[-1])

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

        else:
            for value in traj:
                t1 = time.time() # get current time
                voltage = chan.voltage # read adc voltage
                [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
                # convert adc voltage to a force
                current_force = 0.161*math.exp(1.506*voltage) - 0.161
                filtered_force = 0.161*math.exp(1.506*filtered_voltage) - 0.161
                desired_force = value

                # if voltage >= 0.0 and voltage <= 1.3: 
                #     current_force = (0.2/1.3)*voltage 
                #     filtered_force = (0.2/1.3)*filtered_voltage
                # elif voltage > 1.3 and voltage <= 3.1: 
                #     current_force = 0.4444*voltage - 0.3778
                #     filtered_force = 0.4444*filtered_voltage - 0.3778
                # elif voltage > 3.1: 
                #     current_force = 6.4286*voltage - 18.9286
                #     filtered_force = 6.4286*filtered_voltage - 18.9286

                # get a control effort based on force error    
                [control_effort, previous_error, kp_term, ki_term , kd_term,
                error_sum, error_derivative, differentiation_points] = robot.PID_control(desired_force,filtered_force,previous_error,
                                                                                        error_sum,error_derivative,differentiation_points,loop_dt)
                
                # send control effort as a pwm signal
                valve_pwm.value = control_effort + 0.5  #May have to scale/normalize btwn 0 & 1

                # increase loop counter
                iterations = iterations + 1

                # Append data for saving
                time_loop_start.append(t1-t_init)
                desired_force_vector.append(desired_force)
                current_force_vector.append(current_force)
                voltage_vector.append(voltage)
                control_effort_vector.append(control_effort)
                filtered_voltage_vector.append(filtered_voltage)
                filtered_force_vector.append(filtered_force)

                # Send data to plot
                # client.send_array(current_force_vector[-1])

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

    # Save data as excel sheet
    book = xlsxwriter.Workbook(file_save_path)
    sheet1 = book.add_worksheet('Sheet 1')
    sheet1.write(0,0,'Time Loop Start')
    sheet1.write(0,1,'Desired Force')
    sheet1.write(0,2,'Actual Force')
    sheet1.write(0,3,'ADC Voltage')
    sheet1.write(0,4,'Control Effort')

    sheet1.write(0,5,'Iterations')
    sheet1.write(1,5,iterations)

    sheet1.write(0,6,'Trajectory')
    sheet1.write(0,7,'Filtered Voltage')
    sheet1.write(0,8,'Filtered Force')

    i = 1
    final_traj = traj.tolist()
    if testing_flag == True:
        for index in range(len(current_force_vector)):
            sheet1.write(i,0,time_loop_start[index])
            sheet1.write(i,1,desired_force_vector[index])
            sheet1.write(i,2,current_force_vector[index])
            sheet1.write(i,3,voltage_vector[index])
            sheet1.write(i,4,control_effort_vector[index])
            sheet1.write(i,6,'N/A')
            sheet1.write(i,7,filtered_voltage_vector[index])
            sheet1.write(i,8,filtered_force_vector[index])
            i = i+1
    else:
        for index in range(len(current_force_vector)):
            sheet1.write(i,0,time_loop_start[index])
            sheet1.write(i,1,desired_force_vector[index])
            sheet1.write(i,2,current_force_vector[index])
            sheet1.write(i,3,voltage_vector[index])
            sheet1.write(i,4,control_effort_vector[index])
            sheet1.write(i,6,final_traj[index])
            sheet1.write(i,7,filtered_voltage_vector[index])
            sheet1.write(i,8,filtered_force_vector[index])
            i = i+1

    book.close()

except KeyboardInterrupt:
    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    print('Writing Data\n')

    # Save data as excel sheet
    book = xlsxwriter.Workbook(file_save_path)
    sheet1 = book.add_worksheet('Sheet 1')
    sheet1.write(0,0,'Time Loop Start')
    sheet1.write(0,1,'Desired Force')
    sheet1.write(0,2,'Actual Force')
    sheet1.write(0,3,'ADC Voltage')
    sheet1.write(0,4,'Control Effort')

    sheet1.write(0,5,'Iterations')
    sheet1.write(1,5,iterations)

    sheet1.write(0,6,'Trajectory')
    sheet1.write(0,7,'Filtered Voltage')
    sheet1.write(0,8,'Filtered Force')

    i = 1
    final_traj = traj.tolist()
    if testing_flag == True:
        for index in range(len(current_force_vector)):
            sheet1.write(i,0,time_loop_start[index])
            sheet1.write(i,1,desired_force_vector[index])
            sheet1.write(i,2,current_force_vector[index])
            sheet1.write(i,3,voltage_vector[index])
            sheet1.write(i,4,control_effort_vector[index])
            sheet1.write(i,6,'N/A')
            sheet1.write(i,7,filtered_voltage_vector[index])
            sheet1.write(i,8,filtered_force_vector[index])
            i = i+1
    else:
        for index in range(len(current_force_vector)):
            sheet1.write(i,0,time_loop_start[index])
            sheet1.write(i,1,desired_force_vector[index])
            sheet1.write(i,2,current_force_vector[index])
            sheet1.write(i,3,voltage_vector[index])
            sheet1.write(i,4,control_effort_vector[index])
            sheet1.write(i,6,final_traj[index])
            sheet1.write(i,7,filtered_voltage_vector[index])
            sheet1.write(i,8,filtered_force_vector[index])
            i = i+1

    book.close()

    print('Script Stopped. You Hit Ctrl+C')


