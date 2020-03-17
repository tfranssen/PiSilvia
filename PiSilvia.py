import MAX6675.MAX6675 as MAX6675
import Adafruit_GPIO.SPI as SPI
import RPi.GPIO as GPIO  # Import Raspberry Pi GPIO library

SPI_PORT = 0
SPI_DEVICE = 0
sensor = MAX6675.MAX6675(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))
temperature = sensor.readTempC()
print 'Thermocouple Temperature: {0:0.3F}C'.format(temperature)