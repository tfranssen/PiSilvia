#!/usr/bin/ python
import time
import board
import busio.SPI as SPI
import digitalio
import MAX6675.MAX6675 as MAX6675

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
cs = digitalio.DigitalInOut(board.D5)

SPI_PORT   = 0
SPI_DEVICE = 0
sensor = MAX6675.MAX6675(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

temperature = sensor.readTempC()
print('Thermocouple Temperature: {0:0.3F}°C'.format(temperature))

