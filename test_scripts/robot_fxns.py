#!/usr/bin/env python3

# Import Packages
# import gpiozero
import numpy as np
from scipy.signal import butter, lfilter, filtfilt
import math
import time
import busio
# import adafruit_ads1x15.ads1015 as ADS
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import random

def PID_control(desired_force,current_force,previous_error,error_over_time,error_derivative,estimation_pts,dt):
	# Constants kp-110, ki-0, kd-1.2
	max_control_effort = 1000 #2000 1000
	scaling_factor = -1.0 #-0.5
	kp = 80.0 # 120# 150 #110 is best for SF=-1.0, MCE=1000 & MA filter # 2000 is best for SF=-0.25, MCE=4000 & MA filter
	ki = 95.0 # 180
	kd = 0.35 # 1.0 or 10.0 works 1.2
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

def balanced_latin_square(n):
    # Williams Design algorithm for balanced squares
    # This works most efficiently when n is even
    if n % 2 == 0:
        pattern = []
        j = 0
        h = 0
        for i in range(n):
            if i % 2 == 0:
                pattern.append(j)
                j += 1
            else:
                pattern.append(n - 1 - h)
                h += 1

        square = []
        for i in range(n):
            row = [(val + i) % n + 1 for val in pattern]
            square.append(row)
        return square
    
    else:
        pattern = []
        j = 0
        h = 0
        for i in range(n):
            if i % 2 == 0:
                pattern.append(j)
                j += 1
            else:
                pattern.append(n - 1 - h)
                h += 1

        square = []
        for i in range(n):
            row = [(val + i) % n + 1 for val in pattern]
            square.append(row)

        for i in range(n):
            square.append(square[i][::-1])

        random.shuffle(square)
        return square


def make_ls(sampling_freq,ls_type,operating_condition):

	if (ls_type != 'ls1') and (ls_type !='ls2') and (ls_type !='ls3') and (ls_type !='ls4'):
		print("ls_type is not 'ls1' or 'ls2' or 'ls3' or 'ls4'. Please reinput ls_type value")
		return
	else:
		if (operating_condition != 'ramp_const') and (operating_condition != 'hold_const'):
			print("operating_condition is not 'ramp_const' or 'hold_const'. Please reinput operating_condition value")
			return
		else:
			# Initialize values
			start_value = 0.0
			end_value = 0.0
			force_levels = [0,7,14,21] # in Newtons
			number_of_applications_per_block = 4
			trajectory = []

			if operating_condition == 'ramp_const':
				force_rate = 1/15
				block_duration = 14 # in sec
				application_interval = np.zeros(len(force_levels))
				ramp_time_scalar = np.zeros(len(force_levels))
				for i in range(0,len(force_levels)):
					application_interval[i] = (1/2)*((block_duration/number_of_applications_per_block)-2*force_rate*force_levels[i])
					ramp_time_scalar[i] = force_rate*force_levels[i]
					# application_interval[i] =  block_duration/((number_of_applications_per_block)*(2)*(1+ramp_size[i]))  

			else:
				ramp_size = (5/4)*np.zeros(len(force_levels)) # as a fraction of application interval 
				application_interval = 1.0*np.zeros(len(force_levels)) # Pre set application interval in sec 
				
			# Define Latin Squares
			ls = {
				'ls1': 
				[[0, 1, 3, 2],
				[1, 2, 0, 3],
				[2, 3, 1, 0],
				[3, 0, 2, 1]],

				'ls2':
				[[3, 0, 2, 1],
				[2, 3, 1, 0],
				[1, 2, 0, 3],
				[0, 1, 3, 2]],

				'ls3':
				[[2, 3, 1, 0],
				[1, 2, 0, 3],
				[0, 1, 3, 2],
				[3, 0, 2, 1]],

				'ls4':
				[[3, 0, 2, 1],
				[0, 1, 3, 2],
				[1, 2, 0, 3],
				[2, 3, 1, 0]],

				'ls5':
				[[1, 0, 3, 2],
				[3, 1, 2, 0],
				[0, 2, 1, 3],
				[2, 3, 0, 1]],

				'ls6':
				[[3, 1, 2, 0],
				[1, 0, 3, 2],
				[2, 3, 0, 1],
				[0, 2, 1, 3]]
			}

			# Add ramp to zero hold at the beginning of the trajectory
			ramp_addendum_time = 3.0 # time in seconds
			ramp_addendum = np.linspace(-15.0,0.0,num = math.floor(sampling_freq*ramp_addendum_time))
			trajectory = np.concatenate((trajectory,ramp_addendum))

			# zero hold at the beginning of the trajectory
			zero_addendum_time = 30.0 # time in seconds
			zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
			trajectory = np.concatenate((trajectory,zero_addendum))

			# Define current latin square for use
			current_square = ls[ls_type]
			for i in range(0,len(current_square)):
				for j in range(0,len(current_square[i])):
					force = force_levels[current_square[i][j]]
					zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*application_interval[current_square[i][j]]))
					hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval[current_square[i][j]]))
					if operating_condition == 'ramp_const':
						ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval[current_square[i][j]]*ramp_size[current_square[i][j]]))
						ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval[current_square[i][j]]*ramp_size[current_square[i][j]]))
					else:
						ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*ramp_time_scalar[current_square[i][j]]))
						ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*ramp_time_scalar[current_square[i][j]]))
					trajectory_cycle = np.concatenate((zero_hold,ramp_up,hold,ramp_down))
					block = []
					for _ in range(number_of_applications_per_block):
						block = np.concatenate((block,trajectory_cycle))
					trajectory = np.concatenate((trajectory,block))

			# Add ramp to zero hold at the end of the trajectory
			end_zero_addendum_time = 5.0 # time in seconds
			end_zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*end_zero_addendum_time))
			trajectory = np.concatenate((trajectory,end_zero_addendum))
			return trajectory

def make_trajectory(sampling_freq,trajectory_type):
	if trajectory_type == 'sine':
		start_value = 0.0
		end_value = 2*math.pi
	
		scaling_term = 5.0 # in Nm
		sine_wave_frequency = 0.75 # in Hz
		time_length_of_trajectory = 1.0/sine_wave_frequency
		offset = 10.0 # in Nm
		amt_of_reps = 10
		
		x = np.linspace(start_value,end_value,num = math.floor(sampling_freq*time_length_of_trajectory))
		sine_trajectory = scaling_term * np.sin(x) + offset

		zero_addendum_time = 5.0 # time in seconds
		zero_addendum = np.linspace(offset,offset,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = []
		
		# Add zero hold at the beginning of the trajectory
		trajectory = np.concatenate((trajectory,zero_addendum))
		for _ in range(amt_of_reps):
			trajectory = np.concatenate((trajectory,sine_trajectory))

	elif trajectory_type == 'block_burst_traj':
		# This is a trajectory for each run
		start_value = 0.0
		end_value = 0.0
		run_time = 10 # in mins
		# run_time = run_time * 60 # in seconds
		rest_interval = 5.0 # in seconds
		force_levels = [0,5,10,15,20] # in Newtons
		ramp_size = 3/4 # as a fraction of application interval
		number_of_sections = 1 # a section is comprised of x blocks and 1 rest interval
		number_of_applications_per_block = 4

		# --------------------------------------------------------------------
		# # Calculate block duration and application interval
		# number_of_blocks_per_section = len(force_levels)
		# block_duration = (run_time - rest_interval*number_of_sections)/(number_of_blocks_per_section*number_of_sections)
		# application_interval = block_duration/(3*number_of_applications_per_block) # t0 in seconds
		# --------------------------------------------------------------------

		# --------------------------------------------------------------------
		# Pre set application interval
		application_interval = 1.0 # in sec
		# --------------------------------------------------------------------
		
		latin_square = []
		trajectory = []
		
		# Add ramp to zero hold at the beginning of the trajectory
		zero_addendum_time = 3.0 # time in seconds
		zero_addendum = np.linspace(-15.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = np.concatenate((trajectory,zero_addendum))

		for idx in range(number_of_sections):
			# rest_period = np.linspace(0.0,0.0,num = math.floor(sampling_freq*rest_interval))
			# random.shuffle(force_levels) # Randomize force level
			# trajectory = np.concatenate((trajectory,rest_period))
			latin_square_row = []
			for force in force_levels:
				zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*application_interval))
				ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval*ramp_size))
				hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval))
				ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval*ramp_size))
				trajectory_cycle = np.concatenate((zero_hold,ramp_up,hold,ramp_down))
				block = []
				for _ in range(number_of_applications_per_block):
					block = np.concatenate((block,trajectory_cycle))
				# trajectory = np.concatenate((trajectory,block))
				latin_square_row = np.concatenate((latin_square_row,block))
			latin_square.append(latin_square_row)
			pop_value = force_levels.pop(0)
			force_levels.append(pop_value)

		random_array = np.arange(0, number_of_sections)
		random.shuffle(random_array)

		for idx in random_array:
			trajectory = np.concatenate((trajectory,latin_square[idx]))

	elif trajectory_type == 'latin_square_traj':
		# This is a trajectory for each run
		start_value = 0.0
		end_value = 0.0
		# force_levels = [0,5,10,15,20] # in Newtons
		force_levels = [1,8,15,22] # in Newtons
		ramp_size = 3/4 # as a fraction of application interval
		number_of_applications_per_block = 4

		application_interval = 1.0 # in sec		

		trajectory = []
		
		# Add ramp to zero hold at the beginning of the trajectory
		ramp_addendum_time = 3.0 # time in seconds
		ramp_addendum = np.linspace(-15.0,0.0,num = math.floor(sampling_freq*ramp_addendum_time))
		trajectory = np.concatenate((trajectory,ramp_addendum))

		# Add ramp to zero hold at the beginning of the trajectory
		zero_addendum_time = 30.0 # time in seconds
		zero_addendum = np.linspace(0.0,0.0,num = math.floor(sampling_freq*zero_addendum_time))
		trajectory = np.concatenate((trajectory,zero_addendum))

		square = balanced_latin_square(len(force_levels))
		for i in range(len(square)):
			for j in range(len(square[i])):
				idx = square[i][j] - 1
				force = force_levels[idx]
				zero_hold = np.linspace(end_value,end_value,num = math.floor(sampling_freq*application_interval))
				ramp_up = np.linspace(start_value,force,num = math.floor(sampling_freq*application_interval*ramp_size))
				hold = np.linspace(force,force,num = math.floor(sampling_freq*application_interval))
				ramp_down = np.linspace(force,end_value,num = math.floor(sampling_freq*application_interval*ramp_size))
				trajectory_cycle = np.concatenate((zero_hold,ramp_up,hold,ramp_down))
				block = []
				for _ in range(number_of_applications_per_block):
					block = np.concatenate((block,trajectory_cycle))
				trajectory = np.concatenate((trajectory,block))
		return trajectory

	elif trajectory_type == 'traj':
		start_value = 0.0
		end_value = 0.0

		application_interval = 1.0 # t0 in seconds, 10 sec
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
		cutoff_freq = 5.0 # Cutoff frequency (Hz)
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
		torque_hold = 7.0 # in Nm
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
		desired_step = 0.0 # in Nm
		time_length_of_step = 2.0 # in seconds
		x = np.linspace(desired_step,desired_step,num = math.floor(sampling_freq*time_length_of_step))
		on_hold_traj = x
		
		# Define "zero hold" parameters
		off_time_length_of_hold = 2.0 # time of 0N hold before step in seconds
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
	ads = ADS.ADS1115(i2c)
	ads.gain = 2/3 # Set adc max value to 6.144 V
	chan = AnalogIn(ads, ADS.P0) # Create single-ended input on channel 0

	read_time = 1.0 # in seconds
	load_cell_voltage = []
	t_init = time.time()

	while (time.time() - t_init < read_time):
		load_cell_voltage.append(chan.voltage)

	avg_voltage = sum(load_cell_voltage)/len(load_cell_voltage)
	force_offset = 146.532248766305*avg_voltage - 391.181161611298
	return force_offset
	# return avg_voltage
		

# def robot_cleanup(pwm_pin,pwm_freq):
# 	valve_pwm = gpiozero.PWMOutputDevice(pwm_pin, active_high=False, initial_value=0.5, frequency=pwm_freq)
# 	valve_pwm.value = 0.5
# 	valve_pwm.close()
# 	print('\nStopping PWM')
# 	return