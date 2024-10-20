from machine import I2C, Pin
import time

# endereço padrão do MCP4725
MCP4725_ADDR = 0x60

# inicializar I2C (SCL = GPIO 5, SDA = GPIO 4)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# função para escrever no MCP4725 (12 bits de dados)
def write_dac(value):
    if value < 0 or value > 4095:
        pass
    
    # divide o valor de 12 bits em dois bytes
    upper_byte = (value >> 4) & 0xFF  # 8 bits mais significativos
    lower_byte = (value & 0x0F) << 4  # 4 bits menos significativos

    # envia os dois bytes para o MCP4725
    i2c.writeto(MCP4725_ADDR, bytes([0x40, upper_byte, lower_byte]))