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
import serial
import inputs
from scipy.signal import butter, lfilter, lfilter_zi


### Initialize Real Time Plotting
# Step 1: Configure the IP address
# ip = '192.168.1.44'
# client.configure_ip(ip) #44
# # client.local_plot()

# Step 2: Initialize the plots
# Initialize one plot with 1 traces

plot_config1 = {'names' : ['Actual Force','Desired Force'],
                'colors' : ['r','b'],
                'line_style': ['',''],
                'title' : "Robot Force",
                'ylabel': "Output Force (N)",
                'xlabel': "Time (s)",
                'yrange': [-10,25]
                }


plot_config2 = {'names' : ['Actual Force'],
                'colors' : ['r'],
                'line_style': [''],
                'title' : "Robot Force",
                'ylabel': "Output Force (N)",
                'xlabel': "Time (s)"
                }

client.initialize_plots(plot_config1)

# #Send 1000 datapoints
# for i in range(1000):

#     # Step 3: Send the data
#     # Send the number 5 a thousand times
#     client.send_array([5])

# Configure the serial port
ser = serial.Serial(
    port='/dev/ttyAMA0',  # or /dev/ttyAMA10, etc.
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

received_data = ""

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

signal_pwm_freq = 1000.0
signal_pwm_pin = 13
signal_pwm = gpiozero.PWMOutputDevice(signal_pwm_pin, active_high=True, initial_value=0.0, frequency=signal_pwm_freq)

# Initialize Data Saving Variables
testing_flag = False
test_time = 5.0 # In seconds
name = 'vibe_n_hold_test_t3_70psi' # 'sine_freq_1hz_amp_15nm_t1'
date = '2_9_26'
trial_name = name + '_' + date + '.xlsx'
save_location = '/home/pi/osseoperception/test_scripts/test_data/2_9_26/'
file_save_path = save_location + trial_name
print('Trial name is: ',name)

# Initialize Control Variables
sampling_freq = 250.0 # In Hz
loop_dt = 1.0/sampling_freq
desired_force = 20.0 #in Newtons
iterations = 0

desired_force_vector = []
current_force_vector = []
voltage_vector = []
control_effort_vector = []
time_loop_start = []
# desired_trajectory_vector = []

differentiation_points = [0.0,0.0,0.0,0.0]
filter_points = [0.0,0.0,0.0,0.0]
filtered_voltage_vector = []
filtered_force_vector = []

previous_error = 0.0
error_sum = 0.0
error_derivative = 0.0

# Create Control Trajectory
traj_type = 'vibe_n_hold' # options: 'vibe_n_hold' # 'on_off' # 'traj' # 'fgwn' # 'sine' # 'step'
traj = robot.make_trajectory(sampling_freq,traj_type)

# Create lowpass filter
fc = 20.0
order = 2

# Normalize cutoff
wn = fc / (sampling_freq / 2)

# Design Butterworth filter
b, a = butter(order, wn, btype='low')

# Initialize filter state
zi = lfilter_zi(b, a)

# Control Robot
try:
# --------------- Serial Comms Start ----------------
    # received_data = ser.readline().decode('utf-8').strip()

    # while(received_data != 'w'):
    #     received_data = ser.readline().decode('utf-8').strip()
    #     print(f"Waiting to start. Received: {received_data}")
    #     # pass

# --------------- Keyboard Input Start ----------------
    # print("Waiting to start. Press 'S' to start.")
    # while received_data != 'KEY_S':
    #     events = inputs.get_key()
    #     for event in events:
    #         if event.ev_type == 'Key':
    #             if event.state == 1:  # Key pressed
    #                 received_data = event.code
    #                 print(f'Key {event.code} pressed')
    #         else:
    #             print("Waiting to start. Press 'S' to start.")

# --------------- User Input Start ----------------

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
            max_traj_value = desired_force
            while(time.time()-t_init < test_time):
                t1 = time.time() # get current time
                # normalized_traj_value = desired_force/max_traj_value
                # signal_pwm.value = normalized_traj_value # Send desired trajectory as a pwm signal
                voltage = chan.voltage # read adc voltage
                filtered_voltage, zi = robot.realtime_lowpass_filter(voltage, b, a, zi)
                # [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
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
                traj.append(desired_force)
                # desired_trajectory_vector.append(signal_pwm.value) # normalized_traj_value

                # Send data to plot
                client.send_array([filtered_force,desired_force])
                # client.send_array([filtered_force])

                # Send data to robot via serial
                # data_to_send = str(desired_force) + '\n'
                # ser.write(data_to_send.encode('utf-8'))

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

        else:
            max_traj_value = max(traj)
            for desired_force in traj:
                t1 = time.time() # get current time
                # normalized_traj_value = desired_force/max_traj_value
                # signal_pwm.value = normalized_traj_value # Send desired trajectory as a pwm signal
                voltage = chan.voltage # read adc voltage
                filtered_voltage, zi = robot.realtime_lowpass_filter(voltage, b, a, zi)
                # [filtered_voltage,filter_points] = robot.ma_filter(voltage,filter_points)
                
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
                # desired_trajectory_vector.append(signal_pwm.value) # normalized_traj_value
                
                # Send data to plot
                client.send_array([filtered_force,desired_force])

                # # Send data to robot via serial
                # data_to_send = str(desired_force) + '\n'
                # ser.write(data_to_send.encode('utf-8'))

                # This while loop sets the sampling rate at sampling_freq
                while (time.time()-t1 < loop_dt):
                    pass

    # # Configure stop pin to low
    # output_pin.off()

    # # Send Stop Signal Through Serial
    # data_to_send = 'w'
    # ser.write(data_to_send.encode('utf-8'))
    
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

    sheet1.write(0,12,'Max Trajectory Value/Scaling Factor')
    sheet1.write(1,12,max_traj_value)

    # sheet1.write(0,13,'Normalized Desired Voltage Trajectory (0 to 1)')

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
            # sheet1.write(i,13,desired_trajectory_vector[index])
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
            # sheet1.write(i,13,desired_trajectory_vector[index])
            i = i+1

    book.close()

    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    signal_pwm.value = 0.0
    signal_pwm.close()

    output_pin.close()
    input_pin.close()

except KeyboardInterrupt:
    print('\nStopping PWM\n')
    valve_pwm.value = 0.5
    valve_pwm.close()

    signal_pwm.value = 0.0
    signal_pwm.close()

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

        sheet1.write(0,12,'Max Trajectory Value/Scaling Factor')
        sheet1.write(1,12,max_traj_value)

        # sheet1.write(0,13,'Normalized Desired Voltage Trajectory (0 to 1)')

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
                # sheet1.write(i,13,desired_trajectory_vector[index])
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
                # sheet1.write(i,13,desired_trajectory_vector[index])
                i = i+1

        book.close()

    print('Script Stopped. You Hit Ctrl+C')


