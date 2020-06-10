#!/usr/bin/env python

# This file is meant to receive input from various sources,
# allows for custom decision making about what to do with them,
# then transmits the results to the robot differential drive controller.
# The published commands are in ROBOT frame, not wheel.

import time
import rospy
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


TS = 0.02  # Sample time in seconds
FILTER = 0.5 # Higher=more velocity filtering but also more phase lag

# Max and min values for the robot. 
# For best kinematics, maximum angular velocity should be approximately (max linear velocity)*2/(RL wheel distance).
MAX_LINEAR_VEL = 0.5  # m/s for robot advancement.
MAX_ANGULAR_VEL = 1.5708  # rad/sec for robot turning.
MIN_REF_VELOCITY = 0.01  # To negate computations for low & redundant values

# the following definitions have to be measured per RC!
MIN_RC_LINEAR = 1040.0
MAX_RC_LINEAR = 1860.0
MIDDLE_RC_LINEAR = 1450.0  # has to be measured and specified due to possible trimming.
MIN_RC_ANGULAR = 1040.0
MAX_RC_ANGULAR = 1860.0
MIDDLE_RC_ANGULAR = 1450.0
RC_DEADZONE = 100  # You might not want the robot to react to very subtle stick movements.


def mapFloat(value, inMin, inMax, outMin, outMax):
	return (value-inMin)*(outMax-outMin)/(inMax-inMin)+outMin

class callback_handler:

	def __init__(self):
		# channel input from RC in the order: linear, angular, toggle
		self.rc_channel = [1500, MIDDLE_RC_LINEAR, MIDDLE_RC_ANGULAR]
		self.linear_external = 0  # linear input from teleoperation or navigation stack
		self.angular_external = 0  # linear input from teleoperation or navigation stack

	def rc_input(self, data):
		self.rc_channel = data.data
	
	def teleop_input(self, data):
		self.linear_external = data.linear.x
		self.angular_external = data.angular.z
	
	def navigation_input(self, data):
		self.linear_external = data.linear.x
		self.angular_external = data.angular.z

			
if __name__ == '__main__':
	robotLinearRef = 0
	robotAngularRef = 0
	finalLinearRef = 0
	finalAngularRef = 0
	prevLinear = 0
	prevAngular = 0
	publishCount = 0
	printCount = 0

	using_navigation = True
	using_teleop = False
	filter_output = False

	# ROS subscriber, publisher and broadcaster setup:
	rospy.init_node('command_handler')
	robotInputs = callback_handler()
	rospy.Subscriber('/rc_signals', Int32MultiArray, robotInputs.rc_input)

	if using_navigation:
		# Commands given from the navigation stack:
		rospy.Subscriber('/navigation/cmd_vel', Twist, robotInputs.navigation_input)
	else:
		# Commands given from the computer keyboard/joystick:
		rospy.Subscriber('/teleop/cmd_vel', Twist, robotInputs.teleop_input)

	command_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	command_object = Twist()
	command_object.linear.x = 0
	command_object.angular.z = 0

	try:
		while not rospy.is_shutdown():

			inputChooser = robotInputs.rc_channel[0]  # Channel that sets input mode.
			if inputChooser < 1400.0:  # Means "when channel 0 is up, navigation will take over, otherwise RC"

				# Check if the values are valid and outside deadzone.
				# Was not put into a function in order to keep DEFINES at scope.

				temp = robotInputs.rc_channel[1]
				if temp < 1000.0 or temp > 2000.0 or abs(temp - MIDDLE_RC_LINEAR) < RC_DEADZONE:
					robotLinearRef = 0.0
				else:
					robotLinearRef = mapFloat(temp, MIN_RC_LINEAR, MAX_RC_LINEAR, -MAX_LINEAR_VEL, MAX_LINEAR_VEL)

				temp = robotInputs.rc_channel[2]
				if temp < 1000.0 or temp > 2000.0 or abs(temp - MIDDLE_RC_ANGULAR) < RC_DEADZONE:
					robotAngularRef = 0.0
				else:
					robotAngularRef = mapFloat(temp, MIN_RC_ANGULAR, MAX_RC_ANGULAR, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL)

			# Here (or after else statement) is the place to add more channel functionality, if needed #

			else:  # this means "use navigation or teleop control". change it to the topic you see fit:
				robotAngularRef = robotInputs.angular_external
				robotLinearRef = robotInputs.linear_external

			# Here is the place to override any commands, like stopping when coming close to a wall,
			# maybe even issue a position command to turn around 180 deg. if its a complicated set
			# of commands, this whole thing can be done through a completely different node.
			# This will also require a STOP service call to navigation stack!

			# Apply acceleration limits through LPF:
			# Higher filter value = slower acceleration.
			if filter_output:
				finalLinearRef = prevLinear * FILTER + robotLinearRef * (1 - FILTER)
				finalAngularRef = prevAngular * FILTER + robotAngularRef * (1 - FILTER)

				# save variables for the next iteration:
				prevLinear = finalLinearRef
				prevAngular = finalAngularRef

			else:
				finalLinearRef = robotLinearRef
				finalAngularRef = robotAngularRef

			if abs(finalLinearRef) < MIN_REF_VELOCITY:
				finalLinearRef = 0.0
			if abs(finalAngularRef) < MIN_REF_VELOCITY:
				finalAngularRef = 0.0

			# Calculations should be made at a higher rate than publishing commands?
			if publishCount > 10:
				command_object.linear.x = finalLinearRef
				command_object.angular.z = finalAngularRef
				command_publisher.publish(command_object)
				publishCount = 1

			# Debug printing:
			if printCount > 5:
				# print robotInputs.rc_channel
				# print robotInputs.linear_external, robotInputs.angular_external
				# print leftRef, leftVel, rightRef, rightVel
				# print 'time',time.time(),'vel',rightVel,'ref',rightRef,'err',rightErr,'out',rightOutput
				# print 'vel',rightVel,'ref',rightRef,'out',rightOutput, 'int', rightIntegral
				# print 'vel',leftVel,'ref',leftRef,'out',leftOutput, 'int', leftIntegral
				# print linGlobalRef, linearVel, angGlobalRef, angularVel #rightOutput, leftOutput
				# print rightPos, rightVel, leftPos, leftVel

				printCount = 1

			publishCount += 1
			printCount += 1
			time.sleep(TS)

	except KeyboardInterrupt:
		pass
