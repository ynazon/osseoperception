#!/usr/bin/env python3

#This is a test script for controlling the WR pneumatic powered perturbation robot

# Import Packages
import xlsxwriter
import robot_fxns as robot
import time
# import RPi.GPIO as gpio
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import gpiozero
from rtplot import client
import math
import numpy as np


### Initialize Real Time Plotting
# Step 1: Configure the IP address
# ip = '192.168.1.44'
# client.configure_ip(ip) #44
# # client.local_plot()

# Step 2: Initialize the plots
# Initialize one plot with 1 traces

plot_config1 = {'names' : ['Force'],
                'colors' : ['r'],
                'line_style': [''],
                'title' : "Robot Force",
                'ylabel': "Output Force (N)",
                'xlabel': "Time (s)",
                'yrange': [-10,25]
                }

client.initialize_plots(plot_config1)

# #Send 1000 datapoints
# for i in range(1000):

#     # Step 3: Send the data
#     # Send the number 5 a thousand times
#     client.send_array([5])



# Initalize Start Pin
start_pin = 17 # Pin 11 is GPIO 17
input_pin = gpiozero.Button(start_pin, pull_up=False)

# Configure stop pin to high
stop_pin = 27 # Pin 13 is GPIO 27
output_pin = gpiozero.LED(stop_pin)
output_pin.on()

# Initialize ADC
scl_pin = 3
sda_pin = 2
i2c = busio.I2C(scl_pin, sda_pin)
ads = ADS.ADS1015(i2c)
ads.gain = 2/3 # Set adc max value to 6.144 V
chan = AnalogIn(ads, ADS.P0) # Create single-ended input on channel 0

# Initialize PWM
pwm_freq = 5000.0
valve_pwm_pin = 18
valve_pwm = gpiozero.PWMOutputDevice(valve_pwm_pin, active_high=True, initial_value=1.0, frequency=pwm_freq)

# pressure_pwm_freq = 500.0
# pressure_pwm_pin = 13
# pressure_pwm = gpiozero.PWMOutputDevice(pressure_pwm_pin, active_high=True, initial_value=0.99, frequency=pressure_pwm_freq)

# Initialize Data Saving Variables
testing_flag = True
test_time = 60.0 # In seconds
name = 'input_test_t1'
date = '9_23_25'
trial_name = name + '_' + date + '.xlsx'
save_location = '/home/pi/osseoperception/test_scripts/test_data/9_23_25/'
file_save_path = save_location + trial_name
print('Trial name is: ',name)

# Initialize Control Variables
sampling_freq = 250.0 # In Hz
loop_dt = 1.0/sampling_freq
desired_force = 10.0 #in Newtons
iterations = 0

desired_force_vector = []#[desired_force]
current_force_vector = []#[0.0]
voltage_vector = []#[0.0]
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
traj_type = 'fgwn' # options: 'vibe_n_hold' # 'on_off' # 'traj' # 'fgwn' # 'sine'
traj = robot.make_trajectory(sampling_freq,traj_type)

# Control Robot
try:
    
    while(input_pin.is_pressed == False):
        pass

    # valve_pwm.value = 0.5
    # time.sleep(1.0)
    # Give input command to start script
    question = '1'
    # question = input('Set up for perturbation experiment. When you are ready to continue type " 1 " (the number one) without parentheses. ')
    
    # Get load cell offset
    # force_offset = robot.load_cell_zero()
    force_offset = 0.0
    
    # Initialize loop time
    t_init = time.time()
    if question == '1':
        if testing_flag == True:
            traj = []
            while(time.time()-t_init < test_time):
            # for value in traj:
                t1 = time.time() # get current time
                # pressure_pwm.value = 0.99 # Set constant max pressure
                voltage = chan.voltage # read adc voltage
                [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
                # # convert adc voltage to a force for an FSR
                # current_force = 0.161*math.exp(1.506*voltage) - 0.161
                # filtered_force = 0.161*math.exp(1.506*filtered_voltage) - 0.161

                current_force = (148.26*voltage - 391.9) - force_offset 
                filtered_force = (148.26*filtered_voltage - 391.9) - force_offset

                # get a control effort based on force error    
                [control_effort, previous_error, kp, ki, kd, kp_term, ki_term , kd_term,
                error_sum, error_derivative, differentiation_points] = robot.PID_control(desired_force,filtered_force,previous_error,
                                                                                        error_sum,error_derivative,differentiation_points,loop_dt)
                
                # send control effort as a pwm signal
                valve_pwm.value = control_effort + 0.5  #May have to scale/normalize btwnS 0 & 1

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
                traj.append(desired_force)

                # Send data to plot
                client.send_array([filtered_force])

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

        else:
            for desired_force in traj:
                t1 = time.time() # get current time
                # pressure_pwm.value = 0.99 # Set constant max pressure
                voltage = chan.voltage # read adc voltage
                [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
                # # convert adc voltage to a force for an FSR
                # current_force = 0.161*math.exp(1.506*voltage) - 0.161
                # filtered_force = 0.161*math.exp(1.506*filtered_voltage) - 0.161

                current_force = (148.26*voltage - 391.9) - force_offset 
                filtered_force = (148.26*filtered_voltage - 391.9) - force_offset

                # get a control effort based on force error    
                [control_effort, previous_error, kp, ki, kd, kp_term, ki_term , kd_term,
                error_sum, error_derivative, differentiation_points] = robot.PID_control(desired_force,current_force,previous_error,
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
                client.send_array([filtered_force])

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

    # Configure stop pin to low
    output_pin.off()
    
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

    sheet1.write(0,9,'kp')
    sheet1.write(1,9,kp)

    sheet1.write(0,10,'ki')
    sheet1.write(1,10,ki)

    sheet1.write(0,11,'kd')
    sheet1.write(1,11,kd)

    i = 1
    if testing_flag == True:
        for index in range(len(current_force_vector)):
            sheet1.write(i,0,time_loop_start[index])
            sheet1.write(i,1,desired_force_vector[index])
            sheet1.write(i,2,current_force_vector[index])
            sheet1.write(i,3,voltage_vector[index])
            sheet1.write(i,4,control_effort_vector[index])
            sheet1.write(i,6,traj[index])
            sheet1.write(i,7,filtered_voltage_vector[index])
            sheet1.write(i,8,filtered_force_vector[index])
            i = i+1
    else:
        final_traj = traj.tolist()
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

    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    # pressure_pwm.value = 0.0
    # pressure_pwm.close()

    output_pin.close()
    input_pin.close()

except KeyboardInterrupt:
    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    # pressure_pwm.value = 0.0
    # pressure_pwm.close()

    output_pin.close()
    input_pin.close()

    if question == '1':
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

        sheet1.write(0,9,'kp')
        sheet1.write(1,9,kp)

        sheet1.write(0,10,'ki')
        sheet1.write(1,10,ki)

        sheet1.write(0,11,'kd')
        sheet1.write(1,11,kd)

        i = 1
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
            final_traj = traj.tolist()
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


