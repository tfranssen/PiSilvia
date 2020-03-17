#!/usr/bin/ python
import time
import board
import busio
import digitalio
import MAX6675.MAX6675 as MAXLIB

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D5)

sensor = MAXLIB(spi, cs)

temperature = sensor.readTempC()
print('Thermocouple Temperature: {0:0.3F}°C'.format(temperature))

