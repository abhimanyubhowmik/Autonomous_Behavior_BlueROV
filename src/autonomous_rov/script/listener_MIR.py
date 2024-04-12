#!/usr/bin/env python3
import math
from os import kill
import string
import numpy as np
from yaml import FlowEntryToken

import rospy
import tf
from std_msgs.msg import Int16
from std_msgs.msg import Float64
from std_msgs.msg import Float64
from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
#from waterlinked_a50_ros_driver.msg import DVL
#from waterlinked_a50_ros_driver.msg import DVLBeam
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import LaserScan
from mavros_msgs.srv import CommandLong
from geometry_msgs.msg import Twist
import sys
import argparse

# ---------- Global Variables ---------------

set_mode = [0]*3
set_mode[0] = True   # Mode manual
set_mode[1] = False  # Mode automatic without correction
set_mode[2] = False  # Mode with correction

#Conditions
init_a0 = True
init_p0 = True
arming = False

angle_wrt_startup = [0]*3
angle_roll_a0 = 0.0
angle_pitch_a0 = 0.0
angle_yaw_a0 = 0.0
angle_yaw_final = 0.0
depth_wrt_startup = 0
depth_p0 = 0

angle_yaw_search_limit = [-45, 45]
angle_yaw_search_speed = 0.1

enable_depth = False 
enable_ping = True 
pinger_confidence = 0
pinger_distance = 0

Vmax_mot = 1900
Vmin_mot = 1100

# Linear/angular velocity 
u = 0               # linear surge velocity 
v = 0               # linear sway velocity
w = 0               # linear heave velocity 

p = 0               # angular roll velocity
q = 0               # angular pitch velocity 
r = 0               # angular heave velocity 

# Control commands
forward_speed = 0.00001
Correction_depth = 1500
Correction_yaw = 1500
Correction_surge = 1500

# State of the robot
state = "descent"
print("Initial state: ", state)

num_thrusters = 4

# ---------- Functions---------------

def joyCallback(data):
	global arming
	global set_mode
	global init_a0
	global init_p0
	global Sum_Errors_Vel
	global Sum_Errors_angle_yaw
	global Sum_Errors_depth

	# Joystick buttons
	btn_arm = data.buttons[7]  # Start button
	btn_disarm = data.buttons[6]  # Back button
	btn_manual_mode = data.buttons[3]  # Y button
	btn_automatic_mode = data.buttons[2]  # X button
	btn_corrected_mode = data.buttons[0]  # A button

	# Disarming when Back button is pressed
	if (btn_disarm == 1 and arming == True):
		arming = False
		armDisarm(arming)

	# Arming when Start button is pressed
	if (btn_arm == 1 and arming == False):
		arming = True
		armDisarm(arming)

	# Switch manual, auto and correction mode
	if (btn_manual_mode and not set_mode[0]):
		set_mode[0] = True
		set_mode[1] = False
		set_mode[2] = False		
		rospy.loginfo("Mode manual")
	if (btn_automatic_mode and not set_mode[1]):
		set_mode[0] = False
		set_mode[1] = True
		set_mode[2] = False		
		rospy.loginfo("Mode automatic")
	if (btn_corrected_mode and not set_mode[2]):
		init_a0 = True
		init_p0 = True
		# set sum errors to 0 here, ex: Sum_Errors_Vel = [0]*3
		set_mode[0] = False
		set_mode[1] = False
		set_mode[2] = True
		rospy.loginfo("Mode correction")

def armDisarm(armed):
	# This functions sends a long command service with 400 code to arm or disarm motors
	if (armed):
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 1, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Arming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except arming")
	else:
		rospy.wait_for_service('mavros/cmd/command')
		try:
			armService = rospy.ServiceProxy('mavros/cmd/command', CommandLong)
			armService(0, 400, 0, 0, 0, 0, 0, 0, 0, 0)
			rospy.loginfo("Disarming Succeeded")
		except rospy.ServiceException as e:
			rospy.loginfo("Except disarming")	


def velCallback(cmd_vel):
	global set_mode

	# Only continue if manual_mode is enabled
	if (set_mode[1] or set_mode[2]):
		return

	# Extract cmd_vel message
	roll_left_right 	= mapValueScalSat(cmd_vel.angular.x)
	yaw_left_right 		= mapValueScalSat(-cmd_vel.angular.z)
	ascend_descend 		= mapValueScalSat(cmd_vel.linear.z)
	forward_reverse 	= mapValueScalSat(cmd_vel.linear.x)
	lateral_left_right 	= mapValueScalSat(-cmd_vel.linear.y)
	pitch_left_right 	= mapValueScalSat(cmd_vel.angular.y)

	setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse, lateral_left_right)

def pingerCallback(data):
	global pinger_confidence
	global pinger_distance
	global Correction_depth
	global Correction_yaw
	global Correction_surge
	global sum_error_distance
	global time_pinger_prev
	global dist_prev
	global vel_prev
	global init_a0
	global angle_yaw_final
	global angle_yaw_search_speed
	global state
	global start_search
	global t_search_left_start
	global num_thrusters
	confidence_threshold = 80
	desired_distance = 1200 # In millimeter
	distance_error = 10
	Kp = 0.00
	Ki = 0.00
	Kd = 0.00
	alpha = 0.1
	beta = 0.005

	pinger_distance = data.data[0]
	pinger_confidence = data.data[1]
	#print(pinger_distance)
	# print(pinger_confidence)
	# Continue only in correction mode
	if set_mode[2] != True:
		return

	if pinger_confidence <= confidence_threshold:
		return

	if state == "descent":
		f = 0

	if state == "forward":
		#print(pinger_distance)	
		if pinger_distance <= desired_distance:
			state = "stop"
			print("state: stop")
			f = 0
			sum_error_distance = 0
			time_pinger_prev = rospy.Time.now()
			dist_prev = pinger_distance
			vel_prev = 0
		else:
			f = forward_speed		
	elif state == "stop":
		error = -(desired_distance - pinger_distance)
		f = 0
		state = "search_left"
		print("state: search_left")
		start_search = True
		t_search_left_start = rospy.Time.now()

		'''
		if abs(error) <= distance_error:
			f = 0
			state = "search_left"
			#f = 0
			# Start rotating counterclockwise
			#init_a0 = True
			#Correction_yaw = computePWM(-angle_yaw_search_speed)
		else:
			time_pinger_current = rospy.Time.now()
			sampeling_period = (time_pinger_current - time_pinger_prev).to_sec()

			#alpha-beta filter
			dist_eval = dist_prev + sampeling_period * vel_prev
			r = pinger_distance - dist_eval
			dist_eval = dist_eval + alpha*r

			vel_eval = vel_prev + beta*r/sampeling_period
			vel_prev = vel_eval

			dist_dot = vel_eval #(z_eval - z_prev)/sampeling_period
			dist_prev = dist_eval
		


			sum_error_distance = sum_error_distance + error * sampeling_period

			f = Kp * error + Ki * sum_error_distance + Kd * (0 - dist_dot)
			print("pinger dist", pinger_distance)
			print(f)
		'''
	elif state == "search_left" or state == "search_right":
		# Check that 5 seconds have passed
		time_passed = rospy.Time.now() - t_search_left_start
		if time_passed.to_sec() >= 10:
			print(pinger_distance)		
			if pinger_distance > desired_distance:
				state = "forward"
				print("state: forward")
				init_a0 = True
				f = forward_speed
			else:
				f = 0
				state = "search_left"
				print("state: search_left")
				start_search = True
				t_search_left_start = rospy.Time.now()
		else:
			f = 0

	Correction_surge = computePWM(f*num_thrusters)
	pub_surge_thrust.publish(f)	
	setOverrideRCIN(1500, 1500, Correction_depth, Correction_yaw, Correction_surge, 1500)

def OdoCallback(data):
	global angle_roll_a0
	global angle_pitch_a0
	global angle_yaw_a0
	global angle_wrt_startup
	global angle_yaw_final
	global init_a0
	global p
	global q
	global r
	global sum_error_yaw
	global Correction_yaw
	global Correction_depth
	global Correction_surge
	global angle_yaw_search_limit
	global angle_yaw_des
	global start_search
	
	num_thrusters = 4
	Kp = 0.000001
	Ki = 0
	#angle_yaw_des = 0
	t_final = 0 #rospy.Duration.from_sec(20.0)
	t_final_int = 0
	angle_yaw_error = 0.5

	orientation = data.orientation
	angular_velocity = data.angular_velocity

	# extraction of yaw angle
	q = [orientation.x, orientation.y, orientation.z, orientation.w]
	euler = tf.transformations.euler_from_quaternion(q)
	angle_roll = euler[0]
	angle_pitch = euler[1]
	angle_yaw = euler[2]

	if (init_a0):
		# at 1st execution, init
		angle_roll_a0 = angle_roll
		angle_pitch_a0 = angle_pitch
		angle_yaw_a0 = angle_yaw
		angle_yaw_final = 0 # math.pi/2
		init_a0 = False
		sum_error_yaw = 0

	angle_wrt_startup[0] = ((angle_roll - angle_roll_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[1] = ((angle_pitch - angle_pitch_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_wrt_startup[2] = ((angle_yaw - angle_yaw_a0 + 3.0*math.pi)%(2.0*math.pi) - math.pi) * 180/math.pi
	angle_yaw_current = angle_wrt_startup[2]
	
	angle = Twist()
	angle.angular.x = angle_wrt_startup[0]
	angle.angular.y = angle_wrt_startup[1]
	angle.angular.z = angle_wrt_startup[2]

	pub_angle_degre.publish(angle)

	# Extraction of angular velocity
	p = angular_velocity.x
	q = angular_velocity.y
	r = angular_velocity.z

	vel = Twist()
	vel.angular.x = p
	vel.angular.y = q
	vel.angular.z = r
	pub_angular_velocity.publish(vel)

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return

	# Only continue if correction_mode is enabled
	if (set_mode[2] != True):
		return

	if state == "descent":
		angle_yaw_des = 0

	if state == "forward":
		angle_yaw_des = 0


	# For all states except search robot should keep the heading
	if state == "search_left":
		if start_search:
			angle_yaw_des = angle_yaw_des + 50
			start_search = False

		# setup depth servo control here sum_error_yaw
		error = (angle_yaw_des - angle_yaw_current)
		error = computeYawError(error)
		f = (Kp * error) / num_thrusters

		#print(f)

		'''
		if angle_yaw_current >= angle_yaw_search_limit[0]:
			state == "search_right"
			init_a0 = True
			f = angle_yaw_search_speed
		else:
			f = -angle_yaw_search_speed
		'''
	elif state == "search_right":
		if angle_yaw_current >= angle_yaw_search_limit[1]:
			state == "search_left"
			init_a0 = True
			f = -angle_yaw_search_speed
		else:
			f = angle_yaw_search_speed
	else: # All other states where we do constant heading
		if t_final != 0:
			a2 = 3*(angle_yaw_final - 0)/t_final_int**2
			a3 = -2*(angle_yaw_final - 0)/t_final_int**3
			current_t = rospy.Time.now() - t_init
			
			sampeling_period = 1/58 #current_t - t_previous
			t_previous = current_t
			if current_t < t_final:
				angle_yaw_des = angle_yaw_a0 + a2*current_t.to_sec()**2 + a3*current_t.to_sec()**3
			elif current_t >= t_final:
				angle_yaw_des = angle_yaw_final
			

			error = (angle_yaw_des - angle_yaw_current)
			error = computeYawError(error)

			sum_error_yaw = sum_error_yaw + error * sampeling_period

			# setup depth servo control here
			# P Controller
			f = Kp * (error) / num_thrusters
			# PI Controller
			#f = (Kp* error + Ki * sum_error_yaw) / num_thrusters 

		else:
			# setup depth servo control here sum_error_yaw
			error = (angle_yaw_des - angle_yaw_current)
			#print(error)
			error = computeYawError(error)
			#print(error)
			f = (Kp * error) / num_thrusters
	
		pub_traj_yaw.publish(angle_yaw_des)
	
	# Send messages to the topics
	pub_yaw_thrust.publish(f*num_thrusters)
	pub_yaw.publish(angle_yaw_current)

	# Send PWM commands to motors
	# yaw command to be adapted using sensor feedback
	f = -f	
	Correction_yaw = computePWM(f)
	#print(Correction_yaw)
	if state == "forward":
		Correction_surge = computePWM(forward_speed)

	setOverrideRCIN(1500, 1500, Correction_depth, Correction_yaw, Correction_surge, 1500)

def computeYawError(error):
	if abs(error) > 180:
		sign = 1 if error >= 0 else -1
		error = - sign * (360 - abs(error))
	
	return error

def DvlCallback(data):
	global set_mode
	global u
	global v
	global w

	u = data.velocity.x  # Linear surge velocity
	v = data.velocity.y  # Linear sway velocity
	w = data.velocity.z  # Linear heave velocity

	Vel = Twist()
	Vel.linear.x = u
	Vel.linear.y = v
	Vel.linear.z = w
	pub_linear_velocity.publish(Vel)

def PressureCallback(data):
	global depth_p0
	global depth_wrt_startup
	global init_p0
	global t_init
	global t_previous
	global sum_error_depth
	global z_final
	global vel_prev
	global z_prev
	global Correction_depth
	global Correction_yaw
	global Correction_surge
	global forward_speed
	global state

	rho = 1000.0 # 1025.0 for sea water
	g = 9.80665
	#z_final = 0.5
	num_thrusters = 4
	floatibility =  0.344
	t_final = rospy.Duration.from_sec(20.0)
	t_final_int = 20.0
	kp = 120
	ki = 10
	kd = 2
	alpha = 0.1
	beta = 0.005
	error_threshold = 0.15
	

	# Only continue if manual_mode is disabled
	if (set_mode[0]):
		return
	elif (set_mode[1]):
		# Only continue if automatic_mode is enabled
		# Define an arbitrary velocity command and observe robot's velocity
		f = -(floatibility + 14.7) /num_thrusters
		PWM = computePWM(f)

		setOverrideRCIN(1500, 1500, int(PWM), 1500, 1500, 1500)
		return

	pressure = data.fluid_pressure

	if (init_p0):
		# 1st execution, init
		depth_p0 = (pressure - 101300)/(rho*g)
		t_init = rospy.Time.now()
		z_prev = depth_p0
		vel_prev = 0
	
		z_final = 0.5
		
		sum_error_depth = 0
		t_previous = t_init
		init_p0 = False

	depth_wrt_startup = (pressure - 101300)/(rho*g) - depth_p0	

	if t_final != 0:
		a2 = 3*(z_final - 0)/t_final_int**2
		a3 = -2*(z_final - 0)/t_final_int**3
		current_t = rospy.Time.now() - t_init
		
		current_sampling = rospy.Time.now()
		sampeling_period = (current_sampling - t_previous)
		#print(sampeling_period)
		sampeling_period = sampeling_period.to_sec() #current_t - t_previous
		#print(sampeling_period)

		t_previous = current_sampling
		if current_t < t_final:
			z_des = 0 + a2*current_t.to_sec()**2 + a3*current_t.to_sec()**3
			z_dot_des = 0 + 2*a2*current_t.to_sec() + 3*a3*current_t.to_sec()**2
		elif current_t >= t_final:
			z_des = z_final
			z_dot_des = 0
			
			
		#alpha-beta filter
		z_eval = z_prev + sampeling_period * vel_prev
		r = depth_wrt_startup - z_eval
		z_eval = z_eval + alpha*r

		vel_eval = vel_prev + beta*r/sampeling_period
		vel_prev = vel_eval

		z_dot = vel_eval #(z_eval - z_prev)/sampeling_period
		z_prev = z_eval


		error = (z_des-depth_wrt_startup)
		sum_error_depth = sum_error_depth + (error * sampeling_period)
		#print("error:", error)
		#print("error sum:", sum_error_depth)
		#print("period: ", sampeling_period)
		


		# setup depth servo control here
		# P Controller
		#f = kp* (error + floatibility) / num_thrusters
		# PI Controller
		#f = (kp*error + ki*sum_error + floatibility) / num_thrusters 
		#PID Controller
		f = (kp * error + ki * sum_error_depth + floatibility + kd * (z_dot_des - z_dot))/ num_thrusters
		#print(f) 
		
	else:
		# setup depth servo control here
		f = (kp*(z_final-depth_wrt_startup) + floatibility )/ num_thrusters # + 
	
	# Send messages to the topics
	pub_traj_depth.publish(z_des)
	pub_heave_thrust.publish(f*num_thrusters)
	pub_depth.publish(depth_wrt_startup)

	# Change sign for this particular robot	
	f = - f

	# update Correction_depth
	Correction_depth = computePWM(f)
	# Send PWM commands to motors
	#Correction_depth = int(Correction_depth)

	# Change state if the robot is descending and reached the desired depth
	current_t = rospy.Time.now() - t_init
	if current_t >= t_final:
		if state == "descent" and error <= error_threshold:
			#pass
			state = "forward"
			print("state: forward")
			Correction_surge = computePWM(forward_speed)
			pub_surge_thrust.publish(forward_speed)


	# Here we also send Correction_yaw
	setOverrideRCIN(1500, 1500, Correction_depth, Correction_yaw, Correction_surge, 1500)

def computePWM(force):
	if force < 0 :
		# PWM = 1464 + ((1464 - 1100)/28.48)*force
		PWM = 1464 + 12.781 * force
	elif force > 0 :
		# PWM = 1536 + (1892 - 1536)/36.34*f
		PWM = 1536 + 9.796 * force
	else:
		PWM = 1500

	if PWM > 1900:
		PWM = 1900
	if PWM < 1100:
		PWM = 1100

	return PWM

def mapValueScalSat(value):
	# Correction_Vel and joy between -1 et 1
	# scaling for publishing with setOverrideRCIN values between 1100 and 1900
	# neutral point is 1500
	pulse_width = value * 400 + 1500

	# Saturation
	if pulse_width > 1900:
		pulse_width = 1900
	if pulse_width < 1100:
		pulse_width = 1100

	return int(pulse_width)


def setOverrideRCIN(channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
	# This function replaces setservo for motor commands.
	# It overrides Rc channels inputs and simulates motor controls.
	# In this case, each channel manages a group of motors not individually as servo set

	msg_override = OverrideRCIn()
	msg_override.channels[0] = np.uint(channel_pitch)       # pulseCmd[4]--> pitch	
	msg_override.channels[1] = np.uint(channel_roll)        # pulseCmd[3]--> roll
	msg_override.channels[2] = np.uint(channel_throttle)    # pulseCmd[2]--> heave 
	msg_override.channels[3] = np.uint( channel_yaw)        # pulseCmd[5]--> yaw
	msg_override.channels[4] = np.uint(channel_forward)     # pulseCmd[0]--> surge
	msg_override.channels[5] = np.uint(channel_lateral)     # pulseCmd[1]--> sway
	msg_override.channels[6] = 1500
	msg_override.channels[7] = 1500

	pub_msg_override.publish(msg_override)


def subscriber():
	rospy.Subscriber("joy", Joy, joyCallback)
	rospy.Subscriber("cmd_vel", Twist, velCallback)
	rospy.Subscriber("mavros/imu/data", Imu, OdoCallback)
	rospy.Subscriber("mavros/imu/water_pressure", FluidPressure, PressureCallback)
	#rospy.Subscriber("/dvl/data", DVL, DvlCallback)
	rospy.Subscriber("distance_sonar", Float64MultiArray, pingerCallback)
	rospy.spin()

if __name__ == '__main__':
	armDisarm(False)  # Not automatically disarmed at startup. This line disarmes the robot
	rospy.init_node('autonomous_MIR', anonymous=False)
	pub_msg_override = rospy.Publisher("mavros/rc/override", OverrideRCIn, queue_size = 10, tcp_nodelay = True)
	pub_angle_degre = rospy.Publisher('angle_degree', Twist, queue_size = 10, tcp_nodelay = True)
	pub_depth = rospy.Publisher('depth/state', Float64, queue_size = 10, tcp_nodelay = True)
	pub_traj_depth = rospy.Publisher('depth/traj', Float64, queue_size = 10, tcp_nodelay = True)
	pub_yaw = rospy.Publisher('yaw/state', Float64, queue_size = 10, tcp_nodelay = True)
	pub_traj_yaw = rospy.Publisher('yaw/traj', Float64, queue_size = 10, tcp_nodelay = True)

	pub_angular_velocity = rospy.Publisher('angular_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_linear_velocity = rospy.Publisher('linear_velocity', Twist, queue_size = 10, tcp_nodelay = True)
	pub_heave_thrust = rospy.Publisher('robot/heave_thrust', Float64, queue_size = 10, tcp_nodelay = True)
	pub_yaw_thrust = rospy.Publisher('robot/yaw_thrust', Float64, queue_size = 10, tcp_nodelay = True)
	pub_surge_thrust = rospy.Publisher('robot/surge_thrust', Float64, queue_size = 10, tcp_nodelay = True)

	subscriber()

