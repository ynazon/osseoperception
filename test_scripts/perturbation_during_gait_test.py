# This is a test script for applying perturbations during gait

# ----------- Stuff for ADC ------------------------
# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15
# -------------------------------------------------------

import wiringpi2 as wp
import yves_spi_lib_4 as lib 
from time import sleep, time 
import xlsxwriter
import csv
import exoskeleton_functions as exo
import linear_encoder_library as linear
import rotary_encoder_library as rotary
import os
import xlrd
import math

#Initialize PWM Signal and output pins
wp.wiringPiSetupPhys()  # Set pin numbers to be the same as the raspberry pi pinout
wp.pinMode(12,2)      # set pin 12 to a hardware pwm pin, hardware pwm only works on GPIO port 12
print 'pwm pin established'
wp.pinMode(33,1)      # set pin 33 to an output pin. This pin enables/disables the motor to run
print 'motor enable pin established'
wp.pinMode(37,1)      # set pin 37 to an output pin. This pin is the chip select pin for the ROTARY encoder counter
print 'rotary chip select pin established'
wp.pinMode(38,1)      # set pin 38 to an output pin. This pin is the chip select pin for the LINEAR encoder counter
print 'linear chip select pin established'

wp.pinMode(15,0)      # set pin 15 to an input pin. This pin is the left toe foot switch
print 'left toe foot switch established'
wp.pinMode(13,0)      # set pin 13 to an input pin. This pin is the left heel foot switch
print 'left heel foot switch established'
wp.pinMode(11,0)      # set pin 11 to an input pin. This pin is the left 5th metatarsal foot switch
print 'left 5th metatarsal foot switch established'
wp.pinMode(18,0)      # set pin 18 to an input pin. This pin is the left 1st metatarsal foot switch
print 'left 1st metatarsal foot switch established'

wp.pwmSetRange(1000) # set range for pwm. This goes from 0 to 1000 now instead of 1024
wp.pwmSetClock(2)
wp.pwmSetMode(0) # Puts pwm in traditional pwm mode (freq. doesn't change with output
wp.pwmWrite(12,0)     # duty cycle between 0 and 1000; 0 = off and 1000 = constantly on
wp.pwmWrite(12,500)
print('PWM initialized')

# trial_name = 'adc_w_exo_on_same_pi_test_t1.xlsx'

# ----------- Stuff for ADC -----------------------------
# Create an ADS1115 ADC (16-bit) instance.
# adc = Adafruit_ADS1x15.ADS1115()
# adc_type = 16
# print('ADC initialized')

# Or create an ADS1015 ADC (12-bit) instance.
adc = Adafruit_ADS1x15.ADS1015()
adc_type = 12
print('ADC initialized')

# Note you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 2/3

if GAIN == 2/3:
	gain_voltage = 6.144
elif GAIN == 1:
	gain_voltage = 4.096
elif GAIN == 2:
	gain_voltage = 2.048
elif GAIN == 4:
	gain_voltage = 1.024
elif GAIN == 8:
	gain_voltage = 0.512
elif GAIN == 16:
	gain_voltage = 0.256

if adc_type == 12:
	data_rate = 3300
	print 'ADC Type: ' + str(adc_type) + 'bit ADC @ ' + str(data_rate) + ' samples per second (SPS)'
elif adc_type == 16:
	data_rate = 860
	print 'ADC Type: ' + str(adc_type) + 'bit ADC @ ' + str(data_rate) + ' samples per second (SPS)'
# ----------------------------------------------------------

trial_name = 'exo_walk_WITH_pert_2_17_21_t3.xlsx'
print trial_name
print ''
lookup_table_name = '/home/pi/code/knee_exoskeleton/lookup_tables/absolute_lookup_table_2_12_21.xlsx'
perturbation_vector_file_name = '/home/pi/code/knee_exoskeleton/trajectories/perturbation_vectors/25_perts_per_tp_vector_v3.csv'
# perturbation_vector_file_name = '/home/pi/code/knee_exoskeleton/trajectories/perturbation_vectors/test_perturbation_onset_vector.csv'
# gait_perturbation_trajectory_name = '/home/pi/code/knee_exoskeleton/trajectories/gait_perturbation_trajectories/fs_100/ramp_and_hold_torque_perturbation_at_100_hz.csv'

loop_dt = 0.01

if loop_dt == 0.01:
	gait_perturbation_trajectory_name = '/home/pi/code/knee_exoskeleton/trajectories/gait_perturbation_trajectories/fs_100/ramp_and_hold_torque_perturbation_at_100_hz.csv'
elif loop_dt == 0.002:
	gait_perturbation_trajectory_name = '/home/pi/code/knee_exoskeleton/trajectories/gait_perturbation_trajectories/fs_500/ramp_and_hold_torque_perturbation_at_500_hz.csv'

pos_info_vector = [0.0]
sum_of_all_error_vector = [0.0]
error_derivative = 0.0
error_sum = 0.0
loop_error = [0.0]
time_step = [0.0]
control_effort = [0.0]
time_loop_start = []

differentiation_points = [0.0,0.0,0.0,0.0]
previous_error = 0.0

''' ----------- This is step response testing code ------------ '''
# Step Response Test Code

try:
	iterations = 0
	prev_error = 0.0
	current_torque = 0.0
	velocity = 0.0
	torque = []
	rotary_pos_vec = []

	left_toe_switch_pin = 15
	left_heel_switch_pin = 13
	left_first_switch_pin = 18
	left_fifth_switch_pin = 11
	left_footswitch = []
	
	# Import lookup table data
	loc = lookup_table_name
	wb = xlrd.open_workbook(loc)
	sheet = wb.sheet_by_index(0)

	linear_lookup_table_values = []
	for i in range(1,sheet.nrows): 
		linear_lookup_table_values.append(int(sheet.cell_value(i,0)))

	rotary_lookup_table_values = []
	for i in range(1,sheet.nrows): 
		rotary_lookup_table_values.append(int(sheet.cell_value(i,1)))

	max_value = rotary.Counts_to_degree(rotary_lookup_table_values[-1])
	min_value = rotary.Counts_to_degree(rotary_lookup_table_values[0])
	encoder_latency_dt = exo.Calculate_Encoder_Latency()
	

	print 'Rotary Encoder Position: ',rotary.Counts_to_degree(rotary.Read_Counter())
	print 'Linear Encoder Position',linear.Counts_to_mm(linear.Read_Counter())
	print 'Max Value: ',max_value
	print 'Min Value: ',min_value

	torque_offset = exo.determine_torque_offset(loop_dt)

	question = input('Set up exo for perturbation experiment. When you are ready to continue type " 1 " (the number one) without parentheses.')

	if question == 1:
		# Initialize Variables
		avg_gait_cycle_time = 1.2513
		# avg_gait_cycle_time = 1.7
		footswitch_data = [3,3,3,3,3,3]
		heel_strike_time = []
		current_heel_strike_start_time = 0
		part_of_gait_cycle = []
		# updated_gait_cycle_times = [0.0,0.0340,0.2041,0.5271,0.8502,1.0543,1.2753,1.4794,avg_gait_cycle_time] 
		updated_gait_cycle_times = [0.0,0.0250,0.1502,0.3879,0.6257,0.7758,0.9385,1.0886,1.2513]
		all_avg_gait_cycle_times = []
		heel_strike_value = 0
		walking_start = 0

		high_position_setpoint = 117.0
		low_position_setpoint = 2.0
		perturbations_done = 0
		offset_perturbation_vector = [0,0,0,0,0,0]
		perturbation_vector = exo.Create_Perturbation_Vector(offset_perturbation_vector,perturbation_vector_file_name)
		scaling_term = 7.0
		perturbation_vector_index = 0

		g1_left_goniometer = []
		g2_right_goniometer = []


		# print perturbation_vector
		# question = input('Hol up wait a minute')
# --------------------------------------------------------------------------------------------------------------------------
		# # # Test 1 - Test at all Timing points and test direction
		# timing_pt = 6
		# perturbation_vector = [0,0,timing_pt,0,0,0,-timing_pt,0]
		# scaling_term = 7.0
		# trial_number = 3
		# date = '12_12_20'
		# trial_name = 'test_1_100_hz_treadmill_walking_w_pert_at_tp_' + str(timing_pt) + '_t' + str(trial_number) + '_' + date + '.xlsx'
# --------------------------------------------------------------------------------------------------------------------------
		# # Test 2 - Test the effect of perturbation frequency
		# perturbation_vector = [0,0,4,8,0,6,2,0]
		# perturbation_vector_type = 'back_to_back'

		# perturbation_vector = [0,0,4,0,8,0,2,0]
		# perturbation_vector_type = 'one_apart'

		# perturbation_vector = [0,0,4,0,0,6,0,0]
		# perturbation_vector_type = 'two_apart'

		# scaling_term = 7.0
		# trial_number = 3
		# date = '12_12_20'
		# trial_name = 'test_2_walking_w_perts_' + perturbation_vector_type + '_t' + str(trial_number) + '_' + date + '.xlsx'
# --------------------------------------------------------------------------------------------------------------------------
		# # Test 3 - Test the effect of perturbation amplitude
		# perturbation_vector = [0,0,3,0,0,6,0,0]
		# # scaling_term = 5.0
		# # scaling_term = 7.0
		# scaling_term = 10.0

		# trial_number = 1
		# date = '12_12_20'
		# trial_name = 'test_3_walking_w_pert_amp_' + str(scaling_term) + '_Nm' + '_t' + str(trial_number) + '_' + date + '.xlsx'

		perturbation_fs_vector = []
		perturbation_start_time_vector = []
		perturbation_end_time_vector = []

		perturbation_signal = exo.Create_Torque_Trajectory(scaling_term,gait_perturbation_trajectory_name)

		flag = 0
		pos_prev = rotary.Counts_to_degree(rotary.Read_Counter())
		print 'Start Walking'
		t_prev = 0.0
		t_init = time()

		while (perturbations_done == 0): # or time()-t_init < 30.0):
			t1 = time()
			linear_pos_in_counts = linear.Read_Counter()
			rotary_pos = rotary.Counts_to_degree(rotary.Read_Counter())
			velocity = (rotary_pos - pos_prev)/loop_dt 
			current_torque = exo.Calculate_Torque(linear_pos_in_counts,rotary_pos,linear_lookup_table_values,rotary_lookup_table_values,pos_prev,encoder_latency_dt,loop_dt)
			current_torque = current_torque - torque_offset
			flag = exo.Check_For_Flags(current_torque,velocity,max_value,min_value,flag) 
			if rotary_pos > high_position_setpoint:
				[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.impedance_control(high_position_setpoint,rotary_pos,velocity,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)
			elif rotary_pos < low_position_setpoint:
				[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.impedance_control(low_position_setpoint,rotary_pos,velocity,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)
			else:
				[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.torque_simple_ff_PID_control(0.0,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt,velocity)
				# [control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.torque_PID_control(0.0,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)


			pwm_signal = control_input + 500.0
			wp.digitalWrite(33,1) # Set motor enable pin high
			wp.pwmWrite(12,int(pwm_signal))
			t_prev = t1
			pos_prev = rotary_pos 

			# ------ ADC Stuff --------------
			# adc_value = adc.read_adc(0, gain=GAIN, data_rate=860)
			# adc_value1 = adc.read_adc(1, gain=GAIN, data_rate=860)
			# adc_value2 = adc.read_adc(2, gain=GAIN, data_rate=860)
			# adc_value3 = adc.read_adc(3, gain=GAIN, data_rate=860)
			# adc_voltage = gain_voltage * float(adc_value/32767.0)
			# print str(adc_voltage) + ' V'
			#--------------------------------

			# Read heel footswitch
			read_footswitch = wp.digitalRead(left_heel_switch_pin)
			left_footswitch.append(read_footswitch)
			footswitch_data.append(read_footswitch)
			footswitch_data = footswitch_data[1:len(footswitch_data)]
			heel_strike_value = exo.heelstrike_detector(footswitch_data)
			# print footswitch_data

			if walking_start == 1:
				# Detect Heel Strike
				if heel_strike_value == 1:
					current_heel_strike_start_time = time()
					heel_strike_time.append(current_heel_strike_start_time)
					part_of_gait_cycle.append('ICO')

					# Read perturbation vector and apply perturbation if vector element != 0 
					# If perturbation vector is positive and not zero then apply a positive perturbation
					if (perturbation_vector_index < len(perturbation_vector) and perturbation_vector[perturbation_vector_index] > 0):
						print 'in pos pert loop'
						while (time() < current_heel_strike_start_time + updated_gait_cycle_times[perturbation_vector[perturbation_vector_index]-1] + 0.5*(updated_gait_cycle_times[perturbation_vector[perturbation_vector_index]] - updated_gait_cycle_times[perturbation_vector[perturbation_vector_index]-1])):
							t1 = time()
							linear_pos_in_counts = linear.Read_Counter()
							rotary_pos = rotary.Counts_to_degree(rotary.Read_Counter())
							velocity = (rotary_pos - pos_prev)/loop_dt 
							current_torque = exo.Calculate_Torque(linear_pos_in_counts,rotary_pos,linear_lookup_table_values,rotary_lookup_table_values,pos_prev,encoder_latency_dt,loop_dt)
							current_torque = current_torque - torque_offset
							flag = exo.Check_For_Flags(current_torque,velocity,max_value,min_value,flag)
							if rotary_pos > high_position_setpoint:
								[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.impedance_control(high_position_setpoint,rotary_pos,velocity,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)
							elif rotary_pos < low_position_setpoint:
								[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.impedance_control(low_position_setpoint,rotary_pos,velocity,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)
							else:
								[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.torque_simple_ff_PID_control(0.0,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt,velocity)
								# [control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.torque_PID_control(0.0,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)

							pwm_signal = control_input + 500.0
							wp.digitalWrite(33,1) # Set motor enable pin high
							wp.pwmWrite(12,int(pwm_signal))
							t_prev = t1
							pos_prev = rotary_pos

							if (t1 > current_heel_strike_start_time) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[1]):
								part_of_gait_cycle.append('ICO')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[1]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[2]):
								part_of_gait_cycle.append('LRE')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[2]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[3]):
								part_of_gait_cycle.append('MST')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[3]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[4]):
								part_of_gait_cycle.append('TST')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[4]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[5]):
								part_of_gait_cycle.append('PSW')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[5]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[6]):
								part_of_gait_cycle.append('ISW')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[6]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[7]):
								part_of_gait_cycle.append('MSW')
							elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[7]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[8]*1.3):
								part_of_gait_cycle.append('TSW')
							else:
								part_of_gait_cycle.append('ERR')

							iterations = iterations + 1
							time_loop_start.append(t1)
							torque.append(current_torque)
							rotary_pos_vec.append(rotary_pos)
							
							# For 16 bit ADC
							# g1_left_gonio_value = adc.read_adc(0, gain=GAIN, data_rate=860)
							# g2_right_gonio_value = adc.read_adc(3, gain=GAIN, data_rate=860)

							# For 12 Bit ADC
							g1_left_gonio_value = adc.read_adc(0, gain=GAIN, data_rate=data_rate)
							g2_right_gonio_value = adc.read_adc(3, gain=GAIN, data_rate=data_rate)

							g1_left_goniometer.append(g1_left_gonio_value)
							g2_right_goniometer.append(g2_right_gonio_value)
							 

							while (time()-t1 < loop_dt):
								pass
								# This while loop makes the sampling rate ~500 Hz 

						print'at pos pert'

						torque_shift = exo.Calculate_Torque(linear_pos_in_counts,rotary_pos,linear_lookup_table_values,rotary_lookup_table_values,pos_prev,encoder_latency_dt,loop_dt)
						torque_shift = torque_shift - torque_offset

						number_of_loops = 0
						perturbation_start_time = time()

						for element in perturbation_signal:
							t1 = time()
							number_of_loops = number_of_loops + 1
							iterations = iterations + 1
							linear_pos_in_counts = linear.Read_Counter()
							rotary_pos = rotary.Counts_to_degree(rotary.Read_Counter())
							velocity = (rotary_pos - pos_prev)/loop_dt 
							current_torque = exo.Calculate_Torque(linear_pos_in_counts,rotary_pos,linear_lookup_table_values,rotary_lookup_table_values,pos_prev,encoder_latency_dt,loop_dt)
							current_torque = current_torque - torque_offset
							flag = exo.Check_For_Flags_During_Perturbation(current_torque,max_value,min_value,flag)
							[control_input,previous_error,kp_term,ki_term,kd_term,delta_t,error_sum,error_derivative,differentiation_points] = exo.torque_PID_control(element+torque_shift,current_torque,previous_error,t_prev,t1,error_sum,error_derivative,differentiation_points,loop_dt)
							
							pwm_signal = control_input + 500.0
							wp.digitalWrite(33,1) # Set motor enable pin high
							wp.pwmWrite(12,int(pwm_signal))
							t_prev = t1
							pos_prev = rotary_pos

							time_loop_start.append(t1)
							torque.append(current_torque)
							rotary_pos_vec.append(rotary_pos)
							g1_left_gonio_value = adc.read_adc(0, gain=GAIN, data_rate=data_rate)
							g2_right_gonio_value = adc.read_adc(3, gain=GAIN, data_rate=data_rate)

							g1_left_goniometer.append(g1_left_gonio_value)
							g2_right_goniometer.append(g2_right_gonio_value)
							
							part_of_gait_cycle.append('PRT')
							while (time()-t1 < loop_dt):
								pass
							# 	# This while loop makes the sampling rate ~500 Hz

						perturbation_end_time = time()
						perturbation_fs = number_of_loops/(perturbation_end_time - perturbation_start_time)

						perturbation_start_time_vector.append(perturbation_start_time)
						perturbation_end_time_vector.append(perturbation_end_time)
						perturbation_fs_vector.append(perturbation_fs)
						perturbation_vector_index = perturbation_vector_index + 1

						print 'pert_vec_index in pos pert loop:',perturbation_vector_index

						if perturbation_vector_index >= len(perturbation_vector):
							perturbations_done = 1
							print 'perturbation vector done in pos pert loop'
							continue
					
					

					else:
						# If perturbation vector is 0 (no perturbation) just add 1 to the index
						# Calculate Update Gait Cycle Times
						updated_gait_cycle_times = exo.gait_phase_time_calculator(avg_gait_cycle_time,current_heel_strike_start_time - heel_strike_time[-2])
						avg_gait_cycle_time = updated_gait_cycle_times[-1]
						all_avg_gait_cycle_times.append(avg_gait_cycle_time)
						perturbation_vector_index = perturbation_vector_index + 1
						print 'pert_vec_index in reg loop:',perturbation_vector_index

						if perturbation_vector_index >= len(perturbation_vector):
							perturbations_done = 1
							print 'perturbation vector done in non pert loop'
							continue

				elif (t1 - current_heel_strike_start_time) > avg_gait_cycle_time*3.0:
				# Check if too much time has passed (30% past T_gc) to end walking
					part_of_gait_cycle.append('ERR')
					# adc_value = adc.read_adc(0, gain=GAIN, data_rate=860)
					# adc_channel_0.append(adc_value)
					walking_start = 0
					perturbations_done = 1
					print 'perturbation vector done bc you stopped walking'
				else:
				# Do gait phase detection
					if (t1 > current_heel_strike_start_time) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[1]):
						part_of_gait_cycle.append('ICO')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[1]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[2]):
						part_of_gait_cycle.append('LRE')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[2]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[3]):
						part_of_gait_cycle.append('MST')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[3]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[4]):
						part_of_gait_cycle.append('TST')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[4]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[5]):
						part_of_gait_cycle.append('PSW')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[5]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[6]):
						part_of_gait_cycle.append('ISW')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[6]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[7]):
						part_of_gait_cycle.append('MSW')
					elif (t1 > current_heel_strike_start_time + updated_gait_cycle_times[7]) and (t1 < current_heel_strike_start_time + updated_gait_cycle_times[8]*1.3):
						part_of_gait_cycle.append('TSW')
					else:
						part_of_gait_cycle.append('ERR')
			else:
			# Check to see if walking has started
				if heel_strike_value == 1:
					current_heel_strike_start_time = time()
					heel_strike_time.append(current_heel_strike_start_time)
					# Calculate Update Gait Cycle Times
					updated_gait_cycle_times = exo.gait_phase_time_calculator(avg_gait_cycle_time,avg_gait_cycle_time)
					avg_gait_cycle_time = updated_gait_cycle_times[-1]
					all_avg_gait_cycle_times.append(avg_gait_cycle_time)
					part_of_gait_cycle.append('ICO')
					walking_start = 1
					perturbation_vector_index = perturbation_vector_index + 1
					print 'pert_vec_index in walking start loop:',perturbation_vector_index
					# print 'we started_walking'
				else:
					part_of_gait_cycle.append('ERR')

			iterations = iterations + 1
			if perturbation_vector_index >= len(perturbation_vector):
				perturbations_done = 1
				print 'perturbation vector done pos 3'
			time_loop_start.append(t1)
			torque.append(current_torque)
			rotary_pos_vec.append(rotary_pos)

			g1_left_gonio_value = adc.read_adc(0, gain=GAIN, data_rate=data_rate)
			g2_right_gonio_value = adc.read_adc(3, gain=GAIN, data_rate=data_rate)

			g1_left_goniometer.append(g1_left_gonio_value)
			g2_right_goniometer.append(g2_right_gonio_value)

			flag = exo.Check_For_Flags(current_torque,velocity,max_value,min_value,flag) 
			
			while (time()-t1 < loop_dt):
				pass
				# This while loop makes the sampling rate ~500 Hz 

		t_end = time()

		wp.pwmWrite(12,500)

		avg_fs = iterations/(t_end-t_init)

		print('footswitch vector length: ',len(left_footswitch))
		print('torque vector length: ',len(torque))
		print('Sampling Frequency: ',avg_fs)

		wp.pwmWrite(12,0)
		wp.pinMode(12,0) # pin 12 is an input pin now
		wp.digitalWrite(33,0)
		wp.digitalWrite(37,1)
		wp.pinMode(33,0)
		wp.pinMode(37,0)
		print('Done Testing')

		book = xlsxwriter.Workbook(trial_name)
		sheet1 = book.add_worksheet('Sheet 1')
		sheet1.write(0,0,'Sensed Torque')
		sheet1.write(0,1,'Rotary Output Position')
		sheet1.write(0,2,'Left Footswitch Data')
		sheet1.write(0,3,'Heel Strike Time')
		sheet1.write(0,4,'Avg Gait Cycle Time')
		sheet1.write(0,5,'Gait Phase')
		sheet1.write(0,6,'t begin vector')

		sheet1.write(0,7,'Encoder Latency')
		sheet1.write(1,7,encoder_latency_dt)
		sheet1.write(0,8,'Flags: 0 - No Issue | 1 - Torque | 2 - Velocity | 3 - Position')
		sheet1.write(1,8,flag)
		sheet1.write(0,9,'Sampling Frequency')
		sheet1.write(1,9,avg_fs)
		sheet1.write(0,10,'Torque Offset')
		sheet1.write(1,10,torque_offset)

		sheet1.write(0,11,'Perturbation Start Times')
		sheet1.write(0,12,'Perturbation End Times')
		sheet1.write(0,13,'Perturbation Sampling Frequency')

		sheet1.write(0,14,'Perturbations Done: 0 - Not Complete | 1 - Complete')
		sheet1.write(1,14,perturbations_done)

		sheet1.write(0,15,'Left Goniometer')
		sheet1.write(0,16,'Right Goniometer')

		sheet1.write(0,17,'Perturbation Size')
		sheet1.write(1,17,scaling_term)


		# sheet1.write(0,15,'ADC Reading in Bits')

		i = 0
		for n in range(len(torque)):
			i = i+1
			sheet1.write(i,0,torque[n])
			sheet1.write(i,1,rotary_pos_vec[n])
			sheet1.write(i,5,part_of_gait_cycle[n])
			sheet1.write(i,6,time_loop_start[n]-t_init)
			sheet1.write(i,15,g1_left_goniometer[n])
			sheet1.write(i,16,g2_right_goniometer[n])
			# sheet1.write(i,15,adc_channel_0[n])

		i = 0
		for n in range(len(left_footswitch)):
			i = i+1
			sheet1.write(i,2,left_footswitch[n])

		i = 0
		for n in range(len(heel_strike_time)):
			i = i+1
			sheet1.write(i,3,heel_strike_time[n]-t_init)

		i = 0
		for n in range(len(all_avg_gait_cycle_times)):
			i = i+1
			sheet1.write(i,4,all_avg_gait_cycle_times[n])

		i = 0
		for n in range(len(perturbation_fs_vector)):
			i = i+1
			sheet1.write(i,11,perturbation_start_time_vector[n]-t_init)
			sheet1.write(i,12,perturbation_end_time_vector[n]-t_init)
			sheet1.write(i,13,perturbation_fs_vector[n])

		book.close()


		
except KeyboardInterrupt:
    wp.pwmWrite(12,0)
    wp.pinMode(12,0) # pin 12 is an input pin now
    wp.digitalWrite(33,0)
    wp.digitalWrite(37,1)
    wp.pinMode(33,0)
    wp.pinMode(37,0)
    print('Script Stopped. You Hit Ctrl+C')

wp.pwmWrite(12,0)
wp.pinMode(12,0) # pin 12 is an input pin now
wp.digitalWrite(33,0)
wp.digitalWrite(37,1)
wp.pinMode(33,0)
wp.pinMode(37,0)