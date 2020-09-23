raspberry:
...
bus = smbus.SMBus(1)
address = 0x04
...
def readNumber():
        x = []
        x.append(bus.read_byte_data(address,200))
        x.append(bus.read_byte_data(address,201))
        x.append(bus.read_byte_data(address,202))
        x.append(bus.read_byte_data(address,203))
        return x

def writeNumber(v1,v2,v3,v4):
        bus.write_i2c_block_data(address,v1,[v2,v3,v4])
        return -1

//example:
writeNumber(0,90,120,180)
values = readNumber()