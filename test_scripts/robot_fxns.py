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
import random

def PID_control(desired_force,current_force,previous_error,error_over_time,error_derivative,estimation_pts,dt):
	# Constants
	max_control_effort = 1000 #2000 1000
	scaling_factor = -1.0 #-0.5
	kp = 110.0 # 120# 150 #110 is best for SF=-1.0, MCE=1000 & MA filter # 2000 is best for SF=-0.25, MCE=4000 & MA filter
	ki = 0.0 # 100
	kd = 1.2 # 1.0 or 10.0 works
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

	if u > 0.5:
		u = 0.5
	elif u < -0.5:
		u = -0.5

	previous_error = current_error
		
	return [u, previous_error, kp, ki, kd, kp_term, ki_term , kd_term, error_over_time, error_derivative, estimation_pts]

def ma_filter(new_value,old_array):
	old_array.append(new_value)
	old_array.pop(0)
	filtered_value = sum(old_array)/len(old_array)
	return [filtered_value, old_array]


def realtime_lowpass_filter(x, b, a, zi):
    y, zi = lfilter(b, a, [x], zi=zi)
    return y[0], zi


def make_trajectory(sampling_freq,trajectory_type):
	if trajectory_type == 'sine':
		start_value = 0.0
		end_value = 2*math.pi
	
		scaling_term = 1 # in Nm
		sine_wave_frequency = 1 # in Hz
		time_length_of_trajectory = 1.0/sine_wave_frequency
		offset = 10.0 # in Nm
		
		x = np.linspace(start_value,end_value,num = math.floor(sampling_freq*time_length_of_trajectory))
		sine_trajectory = scaling_term * np.sin(x) + offset

		zero_addendum_time = 2.0 # time in seconds
		zero_addendum = np.linspace(offset,offset,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = []
		
		# Add zero hold at the beginning of the trajectory
		trajectory = np.concatenate((zero_addendum,sine_trajectory))

	elif trajectory_type == 'block_burst_traj':
		# This is a trajectory for each run
		start_value = 0.0
		end_value = 0.0
		run_time = 10 # in mins
		run_time = run_time * 60 # in seconds
		rest_interval = 5.0 # in seconds
		force_levels = [5,10,15,20] # in Newtons
		ramp_size = 1/2 # as a fraction of application interval
		number_of_sections = 5 # a section is comprised of x blocks and 1 rest interval
		number_of_blocks_per_section = len(force_levels)
		number_of_applications_per_block = 4

		block_duration = (run_time - rest_interval*number_of_sections)/(number_of_blocks_per_section*number_of_sections)
		application_interval = block_duration/(3*number_of_applications_per_block) # t0 in seconds
		trajectory = []
		
		# # Add zero hold at the beginning of the trajectory
		# zero_addendum_time = 3.0 # time in seconds
		# zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
		# trajectory = np.concatenate((trajectory,zero_addendum))

		for _ in range(number_of_sections):
			rest_period = np.linspace(0.0,0.0,num = math.floor(sampling_freq*rest_interval))
			random.shuffle(force_levels) # Randomize force level
			trajectory = np.concatenate((trajectory,rest_period))
			for force in force_levels:
				zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*application_interval))
				ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval*ramp_size))
				hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval))
				ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval*ramp_size))
				trajectory_cycle = np.concatenate((zero_hold,ramp_up,hold,ramp_down))
				block = []
				for _ in range(number_of_applications_per_block):
					block = np.concatenate((block,trajectory_cycle))
				trajectory = np.concatenate((trajectory,block))

	elif trajectory_type == 'traj':
		start_value = 0.0
		end_value = 0.0

		application_interval = 5.0 # t0 in seconds, 10 sec
		rest_interval = 1.5*application_interval # in seconds
		force_levels = [5,10,15,20] # in Newtons
		ramp_sizes = [1/4,1/2,3/4,1] # as a fraction of application interval

		zero_addendum_time = 3.0 # time in seconds
		zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = []
		
		# Add zero hold at the beginning of the trajectory
		trajectory = np.concatenate((trajectory,zero_addendum))

		for force in force_levels:
			ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval*ramp_sizes[force_levels.index(force)]))
			hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval))
			ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval*ramp_sizes[force_levels.index(force)]))
			zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*rest_interval))
			trajectory_cycle = np.concatenate((ramp_up,hold,ramp_down,zero_hold))
			trajectory = np.concatenate((trajectory,trajectory_cycle))

	elif trajectory_type == 'fgwn':
		time_length_of_trajectory = 60.0 # in seconds
		scaling_term = 5.0 # in Nm
		offset = 10.0 # in Nm
		number_of_samples = math.floor(time_length_of_trajectory * sampling_freq)

		# Create Gaussian Noise signal
		mean = 0 # Dont change, should be 0 for signal
		std_dev = 1
		gaussian_noise = np.random.normal(loc=mean, scale=std_dev, size=int(number_of_samples)) # Gaussian Noise

		# Add zero hold at the beginning of the trajectory
		zero_addendum_time = 2.0 # time in seconds
		zero_addendum = np.linspace(mean,mean,num = math.floor(sampling_freq*zero_addendum_time))
		gaussian_noise = np.concatenate((zero_addendum,gaussian_noise))
				
		# Filter Signal
		cutoff_freq = 10.0 # Cutoff frequency (Hz)
		nyquist_freq = 0.5*sampling_freq # Nyquist Frequency
		normalized_cutoff = cutoff_freq/nyquist_freq
		order = 4

		# Get the filter coefficients
		[b,a] = butter(order, normalized_cutoff, btype='low', analog=False)

		# Apply the Filter
		fgwn = filtfilt(b, a, gaussian_noise) # Apply lowpass filter
		
		# Normalize & Scale Gaussian Noise Signal
		max_value  = abs(max(fgwn, key=abs))
		norm_n_scaled_signal = []
		for i in range(len(fgwn)):
			norm_n_scaled_signal.append((scaling_term*(fgwn[i]/max_value)) + offset)

		trajectory = np.array(norm_n_scaled_signal)

		# trajectory = fgwn + offset

	elif trajectory_type == 'vibe_n_hold':
		start_value = 0.0
		end_value = 0.0
		time_length_of_trajectory = 1.0 # in seconds
		type_of_wave = 'timed' # full or timed
	
		# Define sine wave parameters
		scaling_term = 1.0 # in Nm
		frequency = 2.0 # in Hz
		offset = 5.0 # in Nm
		if type_of_wave == 'full':
			time_length_of_sine = 1.0/frequency # in seconds
			x = np.linspace(0.0,2*math.pi,num = math.floor(sampling_freq*time_length_of_sine))
			sine_traj = scaling_term * np.sin(x) + offset
		else:
			time_length_of_sine = 1.0 # in seconds
			x = np.linspace(0.0,2*math.pi,num = math.floor(sampling_freq*time_length_of_sine))
			sine_traj = scaling_term * np.sin(frequency*x) + offset
			
		# Define hold parameters
		time_length_of_hold = 1.0 # in seconds
		x = np.linspace(offset,offset,num = math.floor(sampling_freq*time_length_of_hold))
		hold_traj = x

		# Add zero hold at the beginning of the trajectory
		zero_addendum_time = 2.0 # time in seconds
		zero_addendum = np.linspace(offset,offset,num = math.floor(sampling_freq*zero_addendum_time))

		# Combine sine & hold trajectories
		number_of_vibration_and_hold_cycles = 3
		trajectory = []

		trajectory_cycle = np.concatenate((zero_addendum,hold_traj,sine_traj))
		for i in range(number_of_vibration_and_hold_cycles):
			trajectory = np.concatenate((trajectory,trajectory_cycle))
		
	elif trajectory_type == 'on_off':
		start_value = 0.0
		end_value = 0.0
		# time_length_of_trajectory = 1.0 # in seconds

		# Define "on" parameters
		torque_hold = 15.0 # in Nm
		time_length_of_hold = 2.0 # in seconds
		x = np.linspace(torque_hold,torque_hold,num = math.floor(sampling_freq*time_length_of_hold))
		on_hold_traj = x
		
		# Define "off" parameters
		off_time_length_of_hold = 3.0 # in seconds
		x = np.linspace(0.0,0.0,num = math.floor(sampling_freq*off_time_length_of_hold))
		off_hold_traj = x

		# Combine on & off trajectories
		number_of_on_off_cycles = 1
		zero_addendum_time = 0.0 # time in seconds
		zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = []

		trajectory_cycle = np.concatenate((off_hold_traj,on_hold_traj))
		for i in range(number_of_on_off_cycles):
			trajectory = np.concatenate((trajectory,trajectory_cycle))

		# Add zero hold at the end of the trajectory
		trajectory = np.concatenate((trajectory,zero_addendum))

	elif trajectory_type == 'step':
		# Define step parameters
		desired_step = 10.0 # in Nm
		time_length_of_step = 5.0 # in seconds
		x = np.linspace(desired_step,desired_step,num = math.floor(sampling_freq*time_length_of_step))
		on_hold_traj = x
		
		# Define "zero hold" parameters
		off_time_length_of_hold = 3.0 # time of 0N hold before step in seconds
		x = np.linspace(0.0,0.0,num = math.floor(sampling_freq*off_time_length_of_hold))
		off_hold_traj = x

		trajectory = np.concatenate((off_hold_traj,on_hold_traj))

	else:
		start_value = 0.0
		end_value = 0.0
		time_length_of_trajectory = 1.0 # in seconds
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