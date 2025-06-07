import pigpio
import time
import struct

# Respond with the same 16-byte payload
class SimpleI2CSlave:
    def __init__(self, address, data_gpio):
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise Exception("Cannot connect to pigpiod")
        self.handle = self.pi.bsc_i2c(address)
        self.address = address
        self.data_gpio = data_gpio
        self.pi.set_mode(data_gpio, pigpio.OUTPUT)
        self.pi.write(data_gpio, 0)
        print(f"I2C slave ready on address 0x{address:X}")

    def listen_and_respond(self, data_bytes):
        try:
            status, bytes_read, rx_data = self.pi.bsc_i2c(self.address)
            if bytes_read > 0:
                print(f"Received: {rx_data}")
                self.pi.bsc_i2c(self.address, data_bytes)
        except KeyboardInterrupt:
            self.close()

    def close(self):
        self.pi.bsc_i2c(0)  # disable BSC peripheral
        self.pi.stop()
