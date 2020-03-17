import lcd_driver
from time import *

mylcd = lcd_driver.lcd()

mylcd.lcd_display_string("Hello World!", 1)
