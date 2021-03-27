import smbus

I2C_ADDRESS = 0x77

bus = smbus.SMBus(1)

#Set all ports in input mode
bus.write_byte(I2C_ADDRESS,0xFF)

#Read all the unput lines
value=bus.read_byte(I2C_ADDRESS)
print "%02X" % value