#!/usr/bin/env python

import pigpio
import ppm_input
import rospy
from std_msgs.msg import *

# pins that input from RC receiver
pins = [14, 15, 18]
num_of_channels = len(pins)
channels = []
values = []
idx = 0
for current_pin in pins:
	channels.append(ppm_input.PPMChannel(current_pin))
	values.append(channels[idx].pulse_width)  # passing by reference?
	idx += 1
		
if __name__== '__main__':
	try:
		rospy.init_node('teleop_rc_rpi', anonymous=True)
		signals_publisher = rospy.Publisher('/rc_signals', Int32MultiArray, queue_size=50)
		rate = rospy.Rate(10)
		signals_object = Int32MultiArray()
		signals_object.layout.dim = MultiArrayDimension('sig', 3, 96)

		while not rospy.is_shutdown():
			signals_object.data = values
			# print signalMsg.data
			signals_publisher.publish(signals_object)
			rate.sleep()

	finally:
		for current_channel in channels:
			current_channel.clear_callbacks()
		ppm_input.PPMChannel.clear_pigpio()
