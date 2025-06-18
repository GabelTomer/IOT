import pigpio

# Respond with the same 16-byte payload
class SimpleI2CSlave:
    def __init__(self, address, data_gpio):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Cannot connect to pigpiod")
        self.address = address
        self.data_gpio = data_gpio
        self.pi.set_mode(data_gpio, pigpio.OUTPUT)
        self.pi.write(data_gpio, 0)
        print(f"I2C slave ready on address 0x{address:X}")

    def close(self):
        self.pi.bsc_i2c(0)  # disable BSC peripheral
        self.pi.stop()
