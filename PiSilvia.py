import MAX6675.MAX6675 as MAX6675
import Adafruit_GPIO.SPI as SPI
import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library
import threading

#Initialise Thermocouple and average temperature readings
SPI_PORT = 0
SPI_DEVICE = 0
sensor = MAX6675.MAX6675(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
temphist = [20., 20., 20., 20., 20.]
i = 0

def readtemp():
    threading.Timer(0.1, readtemp).start()
    temperature = sensor.readTempC()
    print 'Thermocouple Temperature: {0:0.3F}C'.format(temperature)
    temphist[i % 5] = temperature
    avgtemp = sum(temphist) / len(temphist)
    print(avgtemp)

readtemp()