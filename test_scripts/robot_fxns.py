#!/usr/bin/env python3

# Import Packages
# import gpiozero
import numpy as np
import math

def PID_control(desired_force,current_force,previous_error,error_over_time,error_derivative,estimation_pts,dt):
	# Constants
	max_control_effort = 1000
	scaling_factor = -0.5
	kp = 150.0 # 0.05, bi:7, uni: 1.8,150,90
	ki = 0.0 # 0.6, uni: 15.0, 450, 8.0
	kd = 0.65 # uni: 0.6
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
		
	return [u, previous_error, kp_term, ki_term , kd_term, error_over_time, error_derivative, estimation_pts]

def ma_filter(new_value,old_array):
	old_array.append(new_value)
	old_array.pop(0)
	filtered_value = sum(old_array)/len(old_array)
	return [filtered_value, old_array]

def make_trajectory(start_value,end_value,sampling_freq,time_length_of_trajectory,trajectory_type):
	x = np.linspace(start_value,end_value,num = math.floor(sampling_freq*time_length_of_trajectory))
	if trajectory_type == 'sine':
		trajectory = np.sin(x)
	elif trajectory_type == 'traj':
		trajectory = []
		application_interval = 0.5 # t0 in seconds, 2 sec
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
	else:
		trajectory = x
	return trajectory

# def robot_cleanup(pwm_pin,pwm_freq):
# 	valve_pwm = gpiozero.PWMOutputDevice(pwm_pin, active_high=False, initial_value=0.5, frequency=pwm_freq)
# 	valve_pwm.value = 0.5
# 	valve_pwm.close()
# 	print('\nStopping PWM')
# 	return