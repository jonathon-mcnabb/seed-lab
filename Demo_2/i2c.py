from smbus import SMBus

addr = 0x8 # bus address
bus = SMBus(1) # indicates /dev/ic2-1

test = "A-18.32"
b = test.encode('ascii')
print(b)

while True:
	for byte in b:
		bus.write_byte(addr, byte) # switch it on
input("Press return to exit")
bus.write_byte(addr, 0x0) # switch it on
