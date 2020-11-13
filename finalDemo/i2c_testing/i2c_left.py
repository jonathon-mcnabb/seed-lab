from smbus import SMBus
import time

try:
    addr = 0x8
    bus = SMBus(1)

except:
    print("port not available")

time.sleep(3)

def write_to_i2c(bus, value):
    try:
        b = value.encode('ascii')
        for byte in b:
            bus.write_byte(addr, byte)
    except Exception as e:
        print(e)
        print("Write Error")

def read_from_i2c(bus):
    try:
        if bus.in_waiting > 0:
            line = bus.readLine().decode('utf-8').rstrip()
            print(line)
    except:
        print("read error")


value = "LA-14.223"
while True:
    write_to_i2c(bus, value)
    time.sleep(0.01)
