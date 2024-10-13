# bibliotecas
from machine import Pin, Timer
from time import sleep_ms
import funcoesAuxiliares as aux

# variáveis globais
tempo500ms = 0
dados = False
contadorMinuto = 0;
modo = False
incrementaSp = True

# função para contagem de 500 ms via interrupção Timer 0
def millis500(t):
    global tempo500ms
    global dados
    tempo500ms += 1
    dados = True
    
# configura a interrupção do Timer 0
tim0 = Timer(0)
tim0.init(period=500, mode=Timer.PERIODIC, callback=millis500)

# configura GPIO
Pin(12, Pin.OUT).value(0)
led = Pin(13, Pin.OUT)
led.value(0)

while not aux.iniPerifericos():
    led.value(1)
    sleep_ms(250)
    led.value(0)
    sleep_ms(250)

# configura PID
aux.configPID()

while True:
    nivel = aux.lerNivel()
    saida = aux.calcularPID(nivel)
    aux.ajustarInversor(saida)

    if dados == True:
        dados = False
        tim0.deinit()
        
        led.value(not led.value())
        
        if modo == False:            
            print(aux.formatarDados(tempo500ms, nivel, saida), end="")
        else:
            aux.salvarDados(aux.formatarDados(tempo500ms, nivel, saida))
         
        # se não passou 5 min.
        if contadorMinuto < 600:
            contadorMinuto += 1
        else:
            contadorMinuto = 0
            sp = aux.verificarSp()            
             
            if incrementaSp == True:
                if sp < 70:
                    sp += 10                    
                else:                    
                    incrementaSp = False
                    sp -= 10                
            else:
                if sp > 10:
                   sp -= 10                    
                else:
                   incrementaSp = True
                   sp += 10
                     
            aux.alterarSp(sp)
        
        tim0.init(period=500, mode=Timer.PERIODIC, callback=millis500) 
#     modo = aux.lerModo()