from machine import I2C, Pin, SoftSPI
import ads1115 as ads
import mcp4725 as dac
import os
from sdcard import SDCard
from pid import PID
from time import sleep_ms

# configura I2C
i2c = I2C(0, sda=Pin(4), scl=Pin(5))

# configura SPI
spi = SoftSPI(-1, miso=Pin(10), mosi=Pin(3), sck=Pin(2))
cs = Pin(7)

# objeto para controle PID
pid = PID(scale='ms')

def iniADS1115():
    try:
        ads.configure_ads1115(0xC383)
        ads.read_ads1115()
        return True
    except:
        return False

def iniMCP4725():
    try:
        dac.write_dac(0)
        return True
    except:
        return False
    
def iniSDCard():
    try:        
        sd = SDCard(spi, cs)
        vfs = os.VfsFat(sd)
        os.mount(vfs, '/sd')
        return True
    except:
        return False
    
def configPID():
    pid.Kp = 45
    pid.Ki = 9.91
    pid.Kd = 51.08
    pid.sample_time = 100
    pid.output_limits = (0, 100)
    pid.auto_mode = True
    pid.setpoint = 10

def lerNivel():
    valor = ads.read_ads1115()
    
    nivel = float((0.0052 * valor) - 25.001)
    
    if nivel < 0:
        return 0.0
    if nivel > 100:
        return 100.0
    return nivel

# def lerModo():
#     ads.configure_ads1115(0xF383)
#     sleep_ms(10)
#     valor = ads.read_ads1115()
#     sleep_ms(10)
#     
#     if valor < 10000:
#         return False
#     return True

def calcularPID(nivel):
    return pid(nivel)

def ajustarInversor(valor):
    dac.write_dac(int(valor * 40.95))    

def formatarDados(time, nivel, saida):
    dados = str(time) + ','
    dados += str(nivel) + ','
    dados += str(saida) + ','
    dados += str(pid.setpoint) + ','
    dados += str(pid.Kp) + ','
    dados += str(pid.Ki) + ','
    dados += str(pid.Kd) + '\r\n'    
    return dados

def salvarDados(dados):
    f = open('/sd/dados.csv', 'a')
    f.write(dados)
    f.close()

def verificarSp():
    return pid.setpoint

def alterarSp(sp):
    pid.setpoint = sp