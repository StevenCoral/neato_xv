
# This code requires the pigpio package to be installed, 
# and the pigpiod daemon running in the background for it to work.
# Input is measured between 1000-2000 microseconds,
# while "idle" value for a channel is 1500.

import pigpio


class PPMChannel:
	pi = pigpio.pi()

	# Assuming a standard RC receiver rate of 50Hz (20 millisecond cycle time)
	def __init__(self, input_pin, cycle_time=20000):
		PPMChannel.pi.set_mode(input_pin, pigpio.INPUT)
		self.start = 0
		self.pulse_width = 1500
		self.prev_pulse_width = 1500
		self.callback = PPMChannel.pi.callback(input_pin, pigpio.EITHER_EDGE, self.pulse_ISR)
		self.cycle_time = cycle_time

	def pulse_ISR(self, gpio, level, tick):
		# GPIO argument is a must from pigpio, but not needed here.
		if level:
			# Log tick time:
			self.start = tick
		else:
			# Store the time difference between rising and falling triggers.
			# If time is longer than the cycle time, then we have probably missed a cycle,
			# thus using the previously computed value.
			self.pulse_width = tick-self.start
			if self.pulse_width > self.cycle_time:
				self.pulse_width = self.prev_pulse_width
		self.prev_pulse_width = self.pulse_width
	
	def clear_callbacks(self):
		self.callback.cancel()		
	
	@classmethod
	def clear_pigpio(cls):
		cls.pi.stop()


# Main function will test ranges of channels connected to selected pins:
if __name__ == "__main__":
	import time
	try:
		pins = [14, 15, 18]
		num_of_channels = len(pins)

		channels = []

		idx = 0
		for current_pin in pins:
			channels.append(PPMChannel(current_pin))
			idx += 1

		maxima = num_of_channels * [0]
		minima = num_of_channels * [1e6]

		while True:
			for channel_idx in range(num_of_channels):
				current_value = channels[channel_idx].pulse_width
				if current_value > maxima[channel_idx]:
					maxima[channel_idx] = current_value
				if current_value < minima[channel_idx]:
					minima[channel_idx] = current_value
				print ('ch'+str(channel_idx),
						'max:', maxima[channel_idx],
						'min:', minima[channel_idx],
						'rest:', current_value)
			print(' ')
			time.sleep(1.0)

	except KeyboardInterrupt:
		for current_channel in channels:
			current_channel.clear_callbacks()
		PPMChannel.clear_pigpio()














