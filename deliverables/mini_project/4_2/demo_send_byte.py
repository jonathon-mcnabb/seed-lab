import smbus
import time
import board
import busio
import adafruit_character_lcd.character_lcd_rgb_i2c as character_lcd

# Create the bus connection
bus = smbus.SMBus(1)

# This is the address we setup in the Arduino Program
address = 0x04

# Character LCD size
lcd_columns = 16
lcd_rows = 2

# Initialise I2C bus.
i2c = busio.I2C(board.SCL, board.SDA)

# Initialise the LCD class
lcd = character_lcd.Character_LCD_RGB_I2C(i2c, lcd_columns, lcd_rows)
lcd.clear()

# Set LCD color to white
lcd.color = [10, 10, 10]
time.sleep(1)

# function to write value to the arduino
def writeNumber(value):
    bus.write_byte(address, value)
    return -1

# prompt user for input and turn it into an int to write to arduino
var = input("Enter a Number: ")
var = int(var)
writeNumber(var)
lcd.message = str(var)
time.sleep(1)
