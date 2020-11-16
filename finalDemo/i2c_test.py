from smbus import SMBus
import time

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1
while True:
    bus.write_byte(addr, 0x41) # switch it on
    time.sleep(0.01)
