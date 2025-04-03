#!/usr/bin/env python3

# # Import Packages
# import gpiozero

def PID_control(desired_force,current_force,previous_error,error_over_time,error_derivative,estimation_pts,dt):
	# Constants
	kp = 0.05 # 0.05,
	ki = 0.6 # 0.6
	kd = 0.0 # 
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
	if u < -0.5:
		u = -0.5
	elif u > 0.5:
		u = 0.5

	previous_error = current_error
		
	return [u, previous_error, kp_term, ki_term , kd_term, error_over_time, error_derivative, estimation_pts]

# def robot_cleanup(pwm_pin,pwm_freq):
# 	valve_pwm = gpiozero.PWMOutputDevice(pwm_pin, active_high=False, initial_value=0.5, frequency=pwm_freq)
# 	valve_pwm.value = 0.5
# 	valve_pwm.close()
# 	print('\nStopping PWM')
# 	return