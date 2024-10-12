from machine import I2C, Pin
import time

# endereço padrão do ADS1115
ADS1115_ADDR = 0x48

# inicializar I2C (SCL = GPIO 5, SDA = GPIO 4)
i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=100000)

# função para escrever ao registrador do ADS1115
def write_ads1115(register, value):
    # o valor é enviado em 2 bytes
    high_byte = (value >> 8) & 0xFF
    low_byte = value & 0xFF
    i2c.writeto(ADS1115_ADDR, bytes([register, high_byte, low_byte]))

# função para ler do registrador de conversão do ADS1115
def read_ads1115():
    # primeiro, indicar que queremos ler do registrador de conversão (0x00)
    i2c.writeto(ADS1115_ADDR, bytes([0x00]))
    # ler os 2 bytes da conversão
    result = i2c.readfrom(ADS1115_ADDR, 2)
    
    # converter os 2 bytes em um número de 16 bits
    raw_value = (result[0] << 8) | result[1]
    
    # o valor é um inteiro de 16 bits sinalizado, ajustar se necessário
    if raw_value > 32767:
        raw_value -= 65536

    return raw_value

# configurar o ADS1115
def configure_ads1115(config_value):        
    write_ads1115(0x01, config_value)