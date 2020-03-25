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
import paho.mqtt.client as mqtt

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


# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))
    client.subscribe("koffie/system")
    client.subscribe("koffie/temp")


# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    global targetT
    msg.payload = msg.payload.decode("utf-8")
    print(msg.topic+" "+str(msg.payload))
    if (msg.topic == "koffie/temp"):
        targetT = int(msg.payload)
    print(targetT)


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.username_pw_set("thijs", "frans123")
client.connect("192.168.2.5", 1883, 60)

client.loop_start()


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

class LCDThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(LCDThread, self).__init__()
        self.mylcd = lcd_driver.lcd()

    def run(self):
        while not self.stop_event.is_set():
            self.main()
            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()

    def main(self):
        # Print temp do display:
        try:
            self.mylcd.lcd_display_string('Set: %s C ' % round(targetT, 0), 1)
            self.mylcd.lcd_display_string(
                'T: %sC P:%s %% ' % (round(avgtemp, 0), round(targetPwm, 2)),2)
        except IOError:
            print("LCD Write error") 
            self.mylcd.lcd_display_string("LCD Error", 1)


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
        self.i += 1          


class PIDThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(PIDThread, self).__init__()

    def run(self):
        while not self.stop_event.is_set():
            self.main()
            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()
        GPIO.output(36, GPIO.LOW)  # Turn off

    def main(self):
        global targetPwm
        global avgtemp
        global targetT
        #print (targetT - avgtemp)
        if targetT - avgtemp <= 10:
            targetPwm = pid.output
            targetPwm = max(min(int(targetPwm), 100), 0)
            pid.update(avgtemp)
            GPIO.output(36, GPIO.HIGH)  # Turn on
            sleep(targetPwm / 100.)
            GPIO.output(36, GPIO.LOW)  # Turn off
            sleep(1 - targetPwm / 100.)
        elif targetT - avgtemp > 10:
            targetPwm = 100
            GPIO.output(36, GPIO.HIGH)  # Turn on
            sleep(1)
        else:
            targetPwm = 0
            GPIO.output(36, GPIO.LOW)  # Turn off
            sleep(1)


class RotaryThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(RotaryThread, self).__init__()

    def run(self):
        while not self.stop_event.is_set():
            self.main()
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
        pid.SetPoint = targetT
        clkLastState = clkState
        
class MQTTThread(Thread):

    def __init__(self, interval):
        self.stop_event = Event()
        self.interval = interval
        super(MQTTThread, self).__init__()

    def run(self):
        while not self.stop_event.is_set():
            self.main()
            self.stop_event.wait(self.interval)

    def terminate(self):
        self.stop_event.set()

    def main(self):
        global targetT
        global client
        client.publish("koffie/actualtemp",targetT)
       


if __name__ == '__main__':

    # the workers main function is called and then 5 seconds sleep

    worker = TemperatureThread(interval=0.1)
    worker.start()
    worker2 = RotaryThread(interval=0.01)
    worker2.start()
    worker3 = PIDThread(interval=0.01)
    worker3.start()
    worker4 = MQTTThread(interval=5)
    worker4.start()
    worker5 = LCDThread(interval=0.2)
    worker5.start() 
    
    try:
        while True:
            sleep(1)
    except (KeyboardInterrupt, SystemExit):
        worker.terminate()
        worker2.terminate()
        worker3.terminate()
        worker4.terminate() 
        worker5.terminate()        
        client.disconnect()
        client.loop_stop()
