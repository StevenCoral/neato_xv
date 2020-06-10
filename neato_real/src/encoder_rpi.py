# The Encoder class must have pigpio daemon (sudo pigpiod) running in the background.
# Preferrably, you should run pigpiod with a maximum sample rate (1 uS).
# This module allows the user to create multiple instances of encoder objects, each one using
# interrupts to count pulses up or down. 
#
# There are four operating modes:
#
# Mode x0: used for cheap, single-channel encoders, and thus the direction of rotation cannot be 
# explicitly determined by the encoder itself. Counting up or down must be decided by the user 
# during runtime, altering the value of the "direction" variable to 1 (forward) or 0 (backwards).
# Note that this usually also requires the user to externally define pullup/down on selected pin,
# and that pinB is not used at all and can be chosen arbitrarily.
# 
# Mode x1: each encoder revolution will count exactly the amount of Pulses-Per-Revolution (PPR)
# that the encoder physically has. Note that this option is subject to the risk of counting fake 
# pulses at certain conditions (When moving back and forth on the pinA trigger without triggering
# the pinB flag,it will count a pulse even though the encoder did not really turn. This can be caused
# by system vibrations and becomes more sensitive as the encoder's physical resolution increases).
# 
# Mode x2: each encoder revolution will count twice the PPR that the encoder physically has.
# This uses the same ISR for pinA, but pinB will not count pulses (therefore its ISR is shorter
# and less likely to interfere with the ISRs of other encoder instances).
# 
# Mode x4: also called "Quadrature Encoder". The encoder will count 4-times the amount of its PPR
# in each revolution.
# This is the default, full-scale, most accurate, and most process-power-demanding mode.
# It also has a simple, non-robust implementation for countering the effect of missed ticks.
#
# Given that Linux is not an RTOS, and assuming a worst-case-scenario of a 100uS delay
# between consecutive interrupts, then the maximum pulse speed is about 10,000 Pulses-Per-Second
# for a single encoder. With multiple (n) encoders attached, you should expect the maximum 
# rate to be actually lower than 10,000/n (missed ticks), mainly due to ISR interference.
# It is recommended that you lower the resolution of your encoders if you experience such issues.

import pigpio


class Encoder:

	pi = pigpio.pi()
	
	def __init__(self, pin1, pin2, mode=4):
		self.encoder_mode = mode
		self.pinA = pin1
		self.pinB = pin2
		Encoder.pi.set_mode(pin1, pigpio.INPUT)
		self.levelA = Encoder.pi.read(pin1)
		self.prevLevelA = not self.levelA
		if mode != 0:
			Encoder.pi.set_mode(pin2, pigpio.INPUT)
			self.levelB = Encoder.pi.read(pin2)
			self.prevLevelB = not self.levelB
		self.stepCount = 0
		self.flag = 0
		self.direction = 1
		self.callbackA = None
		self.callbackB = None
		
		# Encoder mode defaults to quadrature (x4)
		self.setMode(mode)
	
	def pinA_ISR_x0(self, gpio, level, tick):
	# When choosing this option, direction must be specified from outside the class.
	# It is made for 1-channel encoders (usually with a cheap hall-effect sensor).
		if self.direction:
			self.stepCount += 1
		else:
			self.stepCount -= 1
	 
	
	def pinA_ISR_x1(self, gpio, level, tick):
	# Note that choosing this option is subject to the risk of fake counts in some cases.	
		if self.flag:
			if self.levelB:
				self.stepCount -= 1
			else:
				self.stepCount += 1
			self.flag = 0
	
	def pinA_ISR_x2x4(self, gpio, level, tick):
	# The difference between x2 and x4 is only whether pinB increments as well or not.
		if self.prevLevelA == level:
			increment = 2
			#print 'Missed an edge on A' #debug
		else:
			increment = 1
		self.levelA = level
		self.prevLevelA = level
		if level:			
			if self.levelB:
				self.stepCount -= increment
			else:
				self.stepCount += increment
		else:			
			if self.levelB:
				self.stepCount += increment
			else:
				self.stepCount -= increment
				
	def pinB_ISR_x1x2(self, gpio, level, tick):
		self.levelB = level
		self.flag = 1
	
	def pinB_ISR_x4(self, gpio, level, tick):
		if self.prevLevelB == level:
			increment = 2
			#print 'Missed an edge on B' #debug
		else:
			increment = 1
		self.levelB = level
		self.prevLevelB = level
		if level:
			if self.levelA:
				self.stepCount += increment
			else:
				self.stepCount -= increment
		else:
			if self.levelA:
				self.stepCount -= increment
			else:
				self.stepCount += increment
				
	def getPosition(self):
		return self.stepCount
	
	def setPosition(self,value):
		self.stepCount = value
	
	def getMode(self):
		return self.encoder_mode
	
	def setMode(self, mode):	
	# Using this function while the encoder is spinning may produce count errors !	
		if self.callbackA is not None:
			self.callbackA.cancel()

		if self.callbackB is not None:
			self.callbackB.cancel()
			self.callbackB = None

		if mode == 0:
			self.callbackA = Encoder.pi.callback(self.pinA, pigpio.EITHER_EDGE, self.pinA_ISR_x0)
		elif mode == 1:
			self.callbackA = Encoder.pi.callback(self.pinA, pigpio.RISING_EDGE, self.pinA_ISR_x1)
			self.callbackB = Encoder.pi.callback(self.pinB, pigpio.EITHER_EDGE, self.pinB_ISR_x1x2)
		elif mode == 2:
			self.callbackA = Encoder.pi.callback(self.pinA, pigpio.EITHER_EDGE, self.pinA_ISR_x2x4)
			self.callbackB = Encoder.pi.callback(self.pinB, pigpio.EITHER_EDGE, self.pinB_ISR_x1x2)
		else: 
			self.callbackA = Encoder.pi.callback(self.pinA, pigpio.EITHER_EDGE, self.pinA_ISR_x2x4)
			self.callbackB = Encoder.pi.callback(self.pinB, pigpio.EITHER_EDGE, self.pinB_ISR_x4)

		self.encoder_mode = mode
	
	def clear_callbacks(self):
		self.callbackA.cancel()
		if self.encoder_mode != 0:
			self.callbackB.cancel()
	
	@classmethod
	def clear_pigpio(cls):
		cls.pi.stop()

