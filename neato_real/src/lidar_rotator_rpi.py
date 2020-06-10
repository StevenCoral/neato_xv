#!/usr/bin/env python
import rospy
import pigpio
from std_msgs.msg import UInt16

PWM_PIN = 16
START_PWM = 75
pi = pigpio.pi()
pi.set_mode(PWM_PIN, pigpio.OUTPUT)
pi.set_PWM_frequency(PWM_PIN, 250)
pi.set_PWM_dutycycle(PWM_PIN, START_PWM)


class CallbackHandler:
	def __init__(self):
		self.integral = 0
		self.outputPWM = 0
		self.nominalPWM = 70  # Different than START_PWM since it needs less "kick".
		self.maxPWM = 80
		
	def spin_control(self,data):
		error = 300-data.data  # Difference between reference (300) to current RPM
		if abs(error) < 100:
			if error > 0:
				self.integral += 0.1
			else:
				self.integral -= 0.1
			self.outputPWM = self.nominalPWM + self.integral  # A "guessed" voltage + a SLOW integrator

			if self.outputPWM > self.maxPWM:
				self.outputPWM = self.maxPWM
			if self.outputPWM < 0:
				self.outputPWM = 0

			pi.set_PWM_dutycycle(PWM_PIN, int(self.outputPWM))		

		print data.data, self.outputPWM, self.integral
	

if __name__ == '__main__':
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	try:
		rospy.init_node('lidar_speed_controller', anonymous=True)
		lidarData = CallbackHandler()
		rospy.Subscriber("rpms", UInt16, lidarData.spin_control)

		rospy.spin()

	finally:
		pi.set_PWM_dutycycle(PWM_PIN, 0)
		pi.stop()
	
		

