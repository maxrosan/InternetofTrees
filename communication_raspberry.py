#TITLE : INTERNET OF TREES
#MASTER 1 INTERNSHIP
#AUTHOR : OUMEIMA EL ISBIHANI
#!/usr/bin/env python

import RPi.GPIO as GPIO
from lib_nrf24 import NRF24
import time
import spidev
import struct
import os
import datetime
import glob
import MySQLdb
from time import strftime

#Variables for MySQL
db = MySQLdb.connect(host,user,passwd_mysql,database_name)
cur = db.cursor()

GPIO.setmode(GPIO.BCM)
pipes = [[ 0xE8, 0xE8, 0xF0, 0xF0, 0xE1 ], [0xF0, 0xF0, 0xF0, 0xF0, 0xE1]]

radio = NRF24(GPIO, spidev.SpiDev())
radio.begin(0,17)
radio.setPayloadSize(32)
radio.setChannel(0x50)
radio.setDataRate(NRF24.BR_1MBPS)
radio.setPALevel(NRF24.PA_MIN)
radio.setAutoAck(True)
radio.enableDynamicPayloads()
radio.enableAckPayload()
radio.openReadingPipe(1, pipes[1])
radio.openWritingPipe(pipes[0])

radio.printDetails()
radio.startListening()

message = list("GETDATA")
while len(message) < 32:
	message.append(0)

while True:
	date = time.strftime("%Y-%m-%d")
	datetimeWrite = (time.strftime("%Y-%m-%d ") + time.strftime("%H:%M:%S"))
	print datetimeWrite

	start = time.time()
	radio.write(message)
	print("Sent the message: {}".format(message))
	radio.startListening()

	while not radio.available(0):
		time.sleep(1/100)
		if time.time() - start > 2:
			print("Timed out.")
			break

	receivedMessage = []
	radio.read(receivedMessage, radio.getDynamicPayloadSize())
	print("Received : {}".format(receivedMessage))
	print("Translating the receivedMessage into unicode characters..")
	
	string = ""
	for n in receivedMessage:
		if (n >= 32   and n<= 126 ):
			string += chr(n)
	print("Our received message decodes to: {}".format(string))
	
	radio.stopListening()
	if (string == "CHARGING PROBLEM") :
		battery_status = string
		sql = ("""INSERT INTO measures(battery_status) VALUES (%s)""", (battery_status))
                try:
                        print "Writing to database..."
                        #Execute the SQL command
                        cur.execute(*sql)
                        #commit your changes in the database
                        db.commit()
                        print "Write Complete"
                except:
                        #rollback in case there is any error
                        db.rollback()
                        print "Failed writing to database"
                cur.close()
                db.close()
		break

	
	else:
		if string :
			id = string.split(";")[0]
			temperature = string.split(";")[1]
			humidity = string.split(";")[2]
			brightness = string.split(";")[3]
			sapflow = string.split(";")[4]
			print(temperature)
			print(humidity)
			print(brightness)
			print(sapflow)
			sql = ("""INSERT INTO measures(id,date,timestamp,temperature,luminosity,humidity,sapflow) VALUES (%s,%s,%s,%s,%s,%s,%s)""", (id,date,datetimeWrite,temperature,brightness,humidity,sapflow))
			try:
				print "Writing to database..."
				#Execute the SQL command
				cur.execute(*sql)
				#commit your changes in the database
				db.commit()
				print "Write Complete"
			except:
				#rollback in case there is any error
				db.rollback()
				print "Failed writing to database"
			cur.close()
			db.close()
			break
