import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import struct
import os
import datetime
import glob
from time import strftime
#from enum import Enum, auto
import array

class SinkStates:
	CONFIGURING   = 1
	WAITING_READS = 2

class SinkActions:
	CONFIGURATION_DONE = 1

class PacketType:
	CONFIGURE = 1
	DATA = 2

class PacketConfigure:
	def __init__(self, interval, duration):
		self.interval = interval
		self.duration = duration

	def buildPacket(self):
		return struct.pack('<BII', PacketType.CONFIGURE, self.interval, self.duration)

class PacketData:
	def __init__(self):
		self.temperature = 0
		self.humidity = 0
		self.brightness = 0
		self.flow = 0

	def decodePacket(self, message):
  		typePacket, self.temperature, self.humidity, self.brightness, self.flow = struct.unpack('<Bffff', message)

class Sink:

	def __init__(self):
		self.state = SinkStates.CONFIGURING
		self.pipes = [ [ 0xE6, 0xE6, 0xE6, 0xE6, 0xE6, 0xE6 ], [ 0xF6, 0xF6, 0xF6, 0xF6, 0xF6, 0xF6 ] ]
		self.radio = None

	def configureRadio(self):

		GPIO.setmode(GPIO.BCM)

		radio = NRF24(GPIO, spidev.SpiDev())
		# # csn & ce are RF24 terminology. csn = SPI's CE!
		radio.begin(0, 28)
		radio.setPayloadSize(32)
		radio.setChannel(0x50)
		radio.setDataRate(NRF24.BR_1MBPS)
		radio.setPALevel(NRF24.PA_MIN)
		radio.setAutoAck(True)
		radio.enableDynamicPayloads()
		radio.enableAckPayload()

		radio.openReadingPipe(1, self.pipes[0])
		radio.openWritingPipe(self.pipes[1])

		radio.printDetails()

		#time.sleep(2.)	

		self.radio = radio

	def fireAction(self, action):
		if self.state == SinkStates.CONFIGURING and action == SinkActions.CONFIGURATION_DONE:
			self.state = SinkStates.WAITING_READS

	def isRadioAvailable(self, interval):

		start = time.time()

		while (not self.radio.available(0)) and (time.time() - start < interval):
			time.sleep(1e-2)

		return self.radio.available(0)

	def _sendConfigurePacket(self):

		#print 'Waiting radio'
		#while (self.isRadioAvailable(2)):
		#	pass

		pkt = PacketConfigure(5, 3)
		message = pkt.buildPacket()

		print 'Sending configuraton packet'

		#self.radio.startListening()

		self.radio.write(message)

		self.fireAction(SinkActions.CONFIGURATION_DONE)

		self.radio.startListening()

	def _waitReads(self):
		pd = PacketData()
		receivedMessage = []
		self.radio.read(receivedMessage, self.radio.getDynamicPayloadSize())
		bytes = array.array('B', receivedMessage).tostring()
		pd.decodePacket(bytes)

		print '%f %f %f %f' % ( pd.temperature, pd.humidity, pd.brightness, pd.flow )

		print receivedMessage

	def loop(self):

		running = True

		while running:

			if self.state == SinkStates.CONFIGURING:
				self._sendConfigurePacket()
			elif self.state == SinkStates.WAITING_READS:
				if self.isRadioAvailable(2):
					self._waitReads()
				
		
			

sink = Sink()
sink.configureRadio()
sink.loop()

