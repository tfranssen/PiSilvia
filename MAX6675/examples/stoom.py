#!/usr/bin/python
# coding: utf8


# Can enable debug output by uncommenting:
#import logging
#logging.basicConfig(level=logging.DEBUG)

import time
import threading
import Adafruit_GPIO.SPI as SPI
import lcd_driver
from time import *

import MAX6675.MAX6675 as MAX6675

import PID as PID

import os.path

import RPi.GPIO as GPIO # Import Raspberry Pi GPIO library
from time import sleep # Import the sleep function from the time module
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW) 

#LCD setup
mylcd = lcd_driver.lcd()

#Rotary setup
clk = 33
dt = 35
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
counter = 0
clkLastState = GPIO.input(clk)

targetT = 125
P = 8
I = 0.9
D = 40

pid = PID.PID(P, I, D)
pid.SetPoint = targetT
pid.setSampleTime(1)

def readConfig ():
	global targetT
	with open ('/tmp/pid.conf', 'r') as f:
		config = f.readline().split(',')
		pid.SetPoint = float(config[0])
		targetT = pid.SetPoint
		pid.setKp (float(config[1]))
		pid.setKi (float(config[2]))
		pid.setKd (float(config[3]))

def createConfig ():
	if not os.path.isfile('/tmp/pid.conf'):
		with open ('/tmp/pid.conf', 'w') as f:
			f.write('%s,%s,%s,%s'%(targetT,P,I,D))

createConfig()

# Define a function to convert celsius to fahrenheit.
def c_to_f(c):
        return c * 9.0 / 5.0 + 32.0


# Uncomment one of the blocks of code below to configure your Pi or BBB to use
# software or hardware SPI.

# Raspberry Pi software SPI configuration.
# CLK = 25
# CS  = 24
# DO  = 18
# sensor = MAX6675.MAX6675(CLK, CS, DO)

# Raspberry Pi hardware SPI configuration.
SPI_PORT   = 0
SPI_DEVICE = 0
sensor = MAX6675.MAX6675(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# Loop printing measurements every second.
print 'Press Ctrl-C to quit.'
while True:
	clkState = GPIO.input(clk)
        if clkState != clkLastState:
        	dtState = GPIO.input(dt)
        	if dtState != clkState:
        		targetT += 1
        	else:
        		targetT -= 1
        clkLastState = clkState
	
	temperature = sensor.readTempC()
	print 'Thermocouple Temperature: {0:0.3F}°C / {1:0.3F}°F'.format(temperature, c_to_f(temperature))
	pid.update(temperature)
	targetPwm = pid.output
	targetPwm = max(min( int(targetPwm), 100 ),0)
	#mylcd.lcd_clear()
	print "Target: %.1f C | Current: %.1f C | PWM: %s %%"%(targetT, temperature, targetPwm)
        mylcd.lcd_display_string("Set: %.1f C "%(targetT), 1)
	mylcd.lcd_display_string("T: %.1fC P:%s %% "%(temperature,targetPwm), 2)
	GPIO.output(36, GPIO.HIGH) # Turn on
        sleep(targetPwm/100.)
        GPIO.output(36, GPIO.LOW) # Turn off
        sleep(1-(targetPwm/100.))
        sleep(0.01)
