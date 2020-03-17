#!/usr/bin/python
# -*- coding: utf-8 -*-

from threading import Thread, Event
from time import sleep
import PID as PID
import os.path
import lcd_driver
import Adafruit_GPIO.SPI as SPI
import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library
import MAX6675.MAX6675 as MAX6675

mylcd = lcd_driver.lcd()

GPIO.setwarnings(False)  # Ignore warning for now
GPIO.setmode(GPIO.BOARD)  # Use physical pin numbering
GPIO.setup(36, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(37, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Rotary decoder settings

clk = 33
dt = 35
GPIO.setup(clk, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(dt, GPIO.IN, pull_up_down=GPIO.PUD_UP)
counter = 0
clkLastState = GPIO.input(clk)

# Thermocouple sensor initializing

SPI_PORT = 0
SPI_DEVICE = 0
sensor = MAX6675.MAX6675(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

# PID and Temep ettings

targetPwm = 0
avgtemp = 0
targetT = 100
P = 3.1
I = 0.9
D = 40

# Initialise PID

pid = PID.PID(P, I, D)
pid.SetPoint = targetT
pid.setSampleTime(0.1)


def readConfig():
    global targetT
    with open('/tmp/pid.conf', 'r') as f:
        config = f.readline().split(',')
        pid.SetPoint = float(config[0])
        targetT = pid.SetPoint
        pid.setKp(float(config[1]))
        pid.setKi(float(config[2]))
        pid.setKd(float(config[3]))


def createConfig():
    if not os.path.isfile('/tmp/pid.conf'):
        with open('/tmp/pid.conf', 'w') as f:
            f.write('%s,%s,%s,%s' % (targetT, P, I, D))


createConfig()


class TemperatureThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(TemperatureThread, self).__init__()
        self.i = 0
        self.temphist = [20., 20., 20., 20., 20.]

    def run(self):
        while not self.stop_event.is_set():
            self.main()

             # wait self.interval seconds or until the stop_event is set

            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()
        GPIO.output(36, GPIO.LOW)  # Turn off

    def main(self):
        global targetPwm
        global avgtemp
        temperature = sensor.readTempC()
        self.temphist[self.i % 5] = temperature
        avgtemp = sum(self.temphist) / len(self.temphist)
        #print 'Thermocouple Temperature: {0:0.3F}C'.format(temperature)
        #print round(avgtemp, 2)
        self.i += 1

        # Print temp do display

        mylcd.lcd_display_string('Set: %.1f C ' % targetT, 1)
        mylcd.lcd_display_string('T: %.1fC P:%s %% ' % (avgtemp,
                                 targetPwm), 2)


class PIDThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(PIDThread, self).__init__()

    def run(self):
        while not self.stop_event.is_set():
            self.main()

             # wait self.interval seconds or until the stop_event is set

            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()
        GPIO.output(36, GPIO.LOW)  # Turn off

    def main(self):
        global targetPwm
        global avgtemp
        global targetT
        print (abs(targetT - avgtemp))
        if abs(targetT - avgtemp) <= 10:
            targetPwm = pid.output
            targetPwm = max(min(int(targetPwm), 100), 0)
            pid.update(avgtemp)
            GPIO.output(36, GPIO.HIGH)  # Turn on
            sleep(targetPwm / 100.)
            GPIO.output(36, GPIO.LOW)  # Turn off
            sleep(1 - targetPwm / 100.)
        else:
            GPIO.output(36, GPIO.HIGH)  # Turn on
            sleep(1)

class RotaryThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(RotaryThread, self).__init__()

    def run(self):
        while not self.stop_event.is_set():
            self.main()

             # wait self.interval seconds or until the stop_event is set

            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()

    def main(self):
        global targetT
        global clkLastState
        global pid
        clkState = GPIO.input(clk)
        if clkState != clkLastState:
            dtState = GPIO.input(dt)
            if dtState != clkState:
                targetT += 5
            else:
                targetT -= 5
            print targetT
        pid.SetPoint = targetT
        clkLastState = clkState


if __name__ == '__main__':

    # the workers main function is called and then 5 seconds sleep

    worker = TemperatureThread(interval=0.1)
    worker.start()
    worker2 = RotaryThread(interval=0.01)
    worker2.start()
    worker3 = PIDThread(interval=0.01)
    worker3.start()
    try:
        while True:
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        worker.terminate()
        worker2.terminate()
        worker3.terminate()

			