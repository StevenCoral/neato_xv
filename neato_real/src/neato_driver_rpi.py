#!/usr/bin/env python

# Some explanations about this being a robot-frame controller

import math
import time
import pigpio
from pidf_control import PidfControl, clamp
import encoder_rpi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


mainPi = pigpio.pi()  # Initialize the pigpio module. "sudo pigpiod" must be run prior to this node!


class NeatoMotor:
	def __init__(self, dir_pin1, dir_pin2, pwm_pin, min_output=0, max_output=255, pwm_freq=250):
		self.min_output = min_output
		self.max_output = max_output
		self.dir_pin1 = dir_pin1
		self.dir_pin2 = dir_pin2
		self.pwm_pin = pwm_pin
		mainPi.set_mode(dir_pin1, pigpio.OUTPUT)
		mainPi.set_mode(dir_pin2, pigpio.OUTPUT)
		mainPi.set_mode(pwm_pin, pigpio.OUTPUT)
		mainPi.set_PWM_frequency(pwm_pin, pwm_freq)
		mainPi.set_PWM_dutycycle(pwm_pin, 0)

	def set_motor_output(self, output):
		if abs(output) < self.min_output:
			output = 0
		else:
			output = clamp(output, -self.max_output, self.max_output)

		if output > 0:
			mainPi.write(self.dir_pin1, 0)  # Always start by taking both down to 0
			mainPi.write(self.dir_pin2, 1)
			mainPi.set_PWM_dutycycle(self.pwm_pin, output)
		else:
			mainPi.write(self.dir_pin2, 0)  # Always start by taking both down to 0
			mainPi.write(self.dir_pin1, 1)
			mainPi.set_PWM_dutycycle(self.pwm_pin, -output)


class CallbackHandler:
	# This one should receive inputs from a CommandHandler node.
	def __init__(self):
		self.robotAngularRef = 0
		self.robotLinearRef = 0

	def handle_input(self, data):
		self.robotAngularRef = data.angular.z  # Angular reference for robot frame, rad/s
		self.robotLinearRef = data.linear.x  # Linear reference for robot frame, m/s


TS = 0.01  # sample time in seconds
FILTER = 0.5  # higher=more velocity filtering but also more phase lag

FRONT_REAR_WHEEL_DISTANCE = 0.4
RIGHT_LEFT_WHEEL_DISTANCE = 0.235
SLIP_COEFFICIENT = 1.1
WHEEL_RADIUS = 0.0375
ENC_CPR = 550.0

INT_MAX = 50.0
KF = 8.0
KI = 0.02 #0.05

MAX_WHEEL_VEL = 14  # rad/sec for wheel turn. Maximum for individual wheel, not whole robot
MAX_PWM_OUTPUT = 210
MIN_PWM_OUTPUT = 20
MIN_REF_VELOCITY = 1.0  # In order to reset integrator when changing output signs

# The following are pin numbers in BCM format.
# Direction and PWM pins are crucial when using RPi Motor Driver Board.
RIGHT_WHEEL_ENC = 23
RIGHT_DIR1_PIN = 6
RIGHT_DIR2_PIN = 13
RIGHT_PWM_PIN = 12

LEFT_WHEEL_ENC = 24
LEFT_DIR1_PIN = 21
LEFT_DIR2_PIN = 20
LEFT_PWM_PIN = 26

enc2rad = 2*math.pi/ENC_CPR
linear2wheel = 1/WHEEL_RADIUS
angular2wheel = RIGHT_LEFT_WHEEL_DISTANCE/(2.0*WHEEL_RADIUS)

# Object instance setup:
right_motor = NeatoMotor(RIGHT_DIR1_PIN, RIGHT_DIR2_PIN, RIGHT_PWM_PIN, MIN_PWM_OUTPUT, MAX_PWM_OUTPUT)
right_encoder = encoder_rpi.Encoder(RIGHT_WHEEL_ENC, 7, 0)
encoder_rpi.Encoder.pi.set_pull_up_down(RIGHT_WHEEL_ENC, pigpio.PUD_UP)
right_pid = PidfControl(TS)
right_pid.set_pidf(0, KI, 0, KF)
right_pid.set_extrema(MIN_REF_VELOCITY, INT_MAX)
right_pid.alpha = FILTER

left_motor = NeatoMotor(LEFT_DIR1_PIN, LEFT_DIR2_PIN, LEFT_PWM_PIN, MIN_PWM_OUTPUT, MAX_PWM_OUTPUT)
left_encoder = encoder_rpi.Encoder(LEFT_WHEEL_ENC, 8, 0)
encoder_rpi.Encoder.pi.set_pull_up_down(LEFT_WHEEL_ENC, pigpio.PUD_UP)
left_pid = PidfControl(TS)
left_pid.set_pidf(0, KI, 0, KF)
left_pid.set_extrema(MIN_REF_VELOCITY, INT_MAX)
left_pid.alpha = FILTER

		
if __name__ == '__main__':
	# Software setup:
	right_pos = 0
	right_vel = 0
	right_ref = 0

	left_pos = 0
	left_vel = 0
	left_ref = 0

	linearRef = 0
	linearVel = 0
	angularRef = 0
	angularVel = 0

	robot_yaw = 0
	robot_pos_x = 0
	robot_pos_y = 0

	publishCount = 0
	printCount = 0

	# ROS subscriber, publisher and broadcaster setup:
	rospy.init_node('neato_driver_rpi')
	velocity_inputs = CallbackHandler()

	rospy.Subscriber('cmd_vel', Twist, velocity_inputs.handle_input)
	odom_broadcaster = tf.TransformBroadcaster()
	odom_publisher = rospy.Publisher('odom', Odometry, queue_size=50)
	odom = Odometry()
	odom.header.frame_id = 'odom'
	odom.child_frame_id = 'base_footprint'

	try:
		while not rospy.is_shutdown():

			# Convert from robot velocity to wheel spin velocities:
			angularRef = velocity_inputs.robotAngularRef * angular2wheel
			linearRef = velocity_inputs.robotLinearRef * linear2wheel
			right_ref = linearRef + angularRef
			left_ref = linearRef - angularRef

			# In case the combination of the linear and angular references become too fast
			# for the wheels to cope with, the following condition will reduce the wheel speed
			# to the value defined in variable MAX_WHEEL_VEL while keeping the ratio between
			# commanded linear and angular references.
			#if abs(right_ref)>MAX_WHEEL_VEL or abs(left_ref)>MAX_WHEEL_VEL:
			#	angularTemp = angularRef*MAX_WHEEL_VEL/(linearRef*angularRef+1)
			#	linearTemp = linearRef*MAX_WHEEL_VEL/(angularRef*linearRef+1)
			#	angularRef = angularTemp
			#	linearRef = linearTemp
			#	right_ref = linearRef+angularRef
			#	left_ref = linearRef-angularRef

			# About direction: when letting go of the stick, reference immediately drops to zero,
			# but the wheels keep on turning due to inertia, which leads to a negative reference,
			# which then leads to backward-counting of the steps until wheel stops.
			if right_ref > 0:
				right_encoder.direction = 1
			if right_ref < 0:
				right_encoder.direction = 0
			if left_ref > 0:
				left_encoder.direction = 1
			if left_ref < 0:
				left_encoder.direction = 0

			# enc2rad means the measure is about wheel angle in radians, not movement length in meters
			right_pos = right_encoder.stepCount * enc2rad
			right_output = right_pid.velocity_control(right_ref, right_pos)
			right_motor.set_motor_output(right_output)

			left_pos = left_encoder.stepCount * enc2rad
			left_output = left_pid.velocity_control(left_ref, left_pos)
			left_motor.set_motor_output(left_output)

			# Convert true wheel speeds back to robot frame and calculate position:
			right_vel = right_pid.filtered_vel
			left_vel = left_pid.filtered_vel
			linearVel = (right_vel + left_vel) * WHEEL_RADIUS / 2.0
			angularVel = (right_vel - left_vel) * WHEEL_RADIUS / RIGHT_LEFT_WHEEL_DISTANCE

			robot_yaw += angularVel * TS
			robot_vel_x = linearVel * math.cos(robot_yaw)  # In "world coordinates"
			robot_vel_y = linearVel * math.sin(robot_yaw)
			robot_pos_x += robot_vel_x * TS * SLIP_COEFFICIENT  # slip coefficient should be <1
			robot_pos_y += robot_vel_y * TS * SLIP_COEFFICIENT  # I guess much is lost within the numeric integration

			# Publish odometry and Tf info
			if publishCount > 10:
				current_time = rospy.Time.now()
				odom_quat = tf.transformations.quaternion_from_euler(0, 0, robot_yaw)
				odom_broadcaster.sendTransform((robot_pos_x, robot_pos_y, 0),
												odom_quat,
												current_time,
												'base_footprint',
												'odom')
				odom.header.stamp = current_time
				odom.pose.pose = Pose(Point(robot_pos_x, robot_pos_y, 0), Quaternion(*odom_quat))
				# odom.twist.twist=Twist(Vector3(velX,velY,0),Vector3(0,0,angularVel))  # Extrinsic
				odom.twist.twist = Twist(Vector3(linearVel, 0, 0), Vector3(0, 0, angularVel))  # Intrinsic
				odom_publisher.publish(odom)
				publishCount = 1

			# Debug printing:
			if printCount > 10:
				# print angularRef, linearRef, right_ref, left_ref
				# print right_ref, right_vel, left_ref, left_vel
				# print 'time',time.time(),'vel',right_vel,'ref',right_ref,'err',rightErr,'out',rightOutput
				# print 'pos',right_pos,'vel',right_vel,'ref',right_ref,'out',rightOutput
				# print posX, posY, theta
				# print linGlobalRef, linearVel, angGlobalRef, angularVel #rightOutput, leftOutput
				# print leftIntegral, rightIntegral
				printCount = 1

			publishCount += 1
			printCount += 1
			time.sleep(TS)

	finally:
		mainPi.set_PWM_dutycycle(RIGHT_PWM_PIN, 0)
		mainPi.set_PWM_dutycycle(LEFT_PWM_PIN, 0)
		right_encoder.clear_callbacks()
		left_encoder.clear_callbacks()
		encoder_rpi.Encoder.pi.set_pull_up_down(RIGHT_WHEEL_ENC, pigpio.PUD_OFF)
		encoder_rpi.Encoder.pi.set_pull_up_down(LEFT_WHEEL_ENC, pigpio.PUD_OFF)
		encoder_rpi.Encoder.clear_pigpio()

