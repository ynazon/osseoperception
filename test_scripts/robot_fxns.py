#!/usr/bin/env python3

# Import Packages
# import gpiozero
import numpy as np
from scipy.signal import butter, lfilter, filtfilt
import math
import time
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

def PID_control(desired_force,current_force,previous_error,error_over_time,error_derivative,estimation_pts,dt):
	# Constants
	max_control_effort = 2000
	scaling_factor = -0.5
	kp = 50.0 # 850, 1050
	ki = 0.0 #
	kd = 0.0 # 10 works
	ped_gain = 0.0 # past error derivative gain term
	ced_gain = 1.0 # current error derivative gain term
	# Note: ced_gain + ped_gain = 1.0

	# Calculate error during this loop
	current_error = desired_force - current_force 

	# Add Current error to error over time
	error_over_time = current_error + error_over_time

	# Calculate kp term
	kp_term = kp*current_error

	# Calculate ki term
	error_integral = (0.5)*(error_over_time)*dt
	ki_term = ki*error_integral

	# Calculate kd term
	# current_error_derivative = (current_error-previous_error)/dt
	current_error_derivative = (1.0/(10.0*dt))*(-2*estimation_pts[0] - estimation_pts[1] + estimation_pts[2] + 2*estimation_pts[3])
	error_derivative = ced_gain*current_error_derivative + ped_gain*error_derivative # This form of error derivative is a way to low pass filter my overall kd term
	kd_term = kd*error_derivative 

	estimation_pts = [estimation_pts[1],estimation_pts[2],estimation_pts[3],current_error]

	u = kp_term + ki_term + kd_term
	if u < -max_control_effort:
		u = -max_control_effort
	elif u > max_control_effort:
		u = max_control_effort

	# Normalize control effort
	u = (u/max_control_effort)*scaling_factor

	# if u > 0.5:
	# 	u = 0.5
	# elif u < -0.5:
	# 	u = -0.5

	previous_error = current_error
		
	return [u, previous_error, kp, ki, kd, kp_term, ki_term , kd_term, error_over_time, error_derivative, estimation_pts]

def ma_filter(new_value,old_array):
	old_array.append(new_value)
	old_array.pop(0)
	filtered_value = sum(old_array)/len(old_array)
	return [filtered_value, old_array]

def make_trajectory(start_value,end_value,sampling_freq,time_length_of_trajectory,trajectory_type):
	if trajectory_type == 'sine':
		x = np.linspace(start_value,end_value,num = math.floor(sampling_freq*time_length_of_trajectory))
		scaling_term = 5 # in Nm
		frequency = 1 # in Hz
		omega = (1.0/frequency) * 2 * math.pi # in Rad/s
		offset = 0 # in Nm
		trajectory = scaling_term* np.sin(omega*x) + offset
	elif trajectory_type == 'traj':
		trajectory = []
		application_interval = 10 # t0 in seconds, 2 sec
		rest_interval = 1.5*application_interval # in seconds
		force_levels = [5,10,15,20] # in Newtons
		ramp_sizes = [1/4,1/2,3/4,1]
		for force in force_levels:
			ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval*ramp_sizes[force_levels.index(force)]))
			hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval))
			ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval*ramp_sizes[force_levels.index(force)]))
			zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*rest_interval))
			trajectory_cycle = np.concatenate((ramp_up,hold,ramp_down,zero_hold))
			trajectory = np.concatenate((trajectory,trajectory_cycle))
	elif trajectory_type == 'fgwn':
		# Create Gaussian Noise signal
		number_of_samples = math.floor(time_length_of_trajectory * sampling_freq)
		mean = 0
		std_dev = 1
		scaling_term = 5 # in Nm
		gaussian_noise = np.random.normal(loc=mean, scale=std_dev, size=int(number_of_samples)) # Gaussian Noise
				
		# Filter Signal
		cutoff_freq = 20.0 # Cutoff frequency (Hz)
		nyquist_freq = 0.5*sampling_freq # Nyquist Frequency
		normalized_cutoff = cutoff_freq/nyquist_freq
		order = 4

		# Get the filter coefficients
		[b,a] = butter(order, normalized_cutoff, btype='low', analog=False)

		# Apply the Filter
		fgwn = lfilter(b, a, gaussian_noise) # Apply lowpass filter
		
		# Normalize & Scale Gaussian Noise Signal
		max_value  = abs(max(fgwn, key=abs))
		norm_n_scaled_signal = []
		for i in range(len(fgwn)):
			norm_n_scaled_signal.append(scaling_term*(fgwn[i]/max_value))

		trajectory = np.array(norm_n_scaled_signal)
		
	else:
		x = np.linspace(start_value,end_value,num = math.floor(sampling_freq*time_length_of_trajectory))
		trajectory = x
	return trajectory

def load_cell_zero():
	# Initialize ADC
	scl_pin = 3
	sda_pin = 2
	i2c = busio.I2C(scl_pin, sda_pin)
	ads = ADS.ADS1015(i2c)
	ads.gain = 2/3 # Set adc max value to 6.144 V
	chan = AnalogIn(ads, ADS.P0) # Create single-ended input on channel 0

	read_time = 1 # in seconds
	load_cell_voltage = []
	t_init = time.time()

	while (time.time() - t_init < read_time):
		load_cell_voltage.append(chan.voltage)

	avg_voltage = sum(load_cell_voltage)/len(load_cell_voltage)
	force_offset = 148.26*avg_voltage - 391.9
	return force_offset
		

# def robot_cleanup(pwm_pin,pwm_freq):
# 	valve_pwm = gpiozero.PWMOutputDevice(pwm_pin, active_high=False, initial_value=0.5, frequency=pwm_freq)
# 	valve_pwm.value = 0.5
# 	valve_pwm.close()
# 	print('\nStopping PWM')
# 	return