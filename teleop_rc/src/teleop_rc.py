#!/usr/bin/env python

import pigpio
import ppm_input_rpi
import rospy
from std_msgs.msg import *
		
if __name__== '__main__':
	try:
		rospy.init_node('teleop_rc_rpi', anonymous=True)
		signals_publisher = rospy.Publisher('/rc_signals', Int32MultiArray, queue_size=50)

		# pins that input from RC receiver
		pins = [14, 15, 18]
		num_of_channels = len(pins)
		channels = []

		idx = 0
		for current_pin in pins:
			channels.append(ppm_input_rpi.PPMChannel(current_pin))
			idx += 1

		rate = rospy.Rate(10)
		signals_object = Int32MultiArray()
		signals_object.data = [1500, 1500, 1500]

		while not rospy.is_shutdown():
			for channel_idx in range(num_of_channels):
				signals_object.data[channel_idx] = channels[channel_idx].pulse_width
			
			signals_publisher.publish(signals_object)
			rate.sleep()

	finally:
		for current_channel in channels:
			current_channel.clear_callbacks()
		ppm_input_rpi.PPMChannel.clear_pigpio()
