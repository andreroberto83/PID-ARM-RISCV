/**************************************************************************************
* Título: Controle PID para planta de processos contínuos                             *
* Autor: André Roberto da Silva                                                       *
* Plataforma: LuatOS                                                                  *
* MCU: ESP32C3                                                                        *
* Núcleo: RISCV                                                                       *
**************************************************************************************/

// bibliotecas
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <SPI.h>
#include <SD.h>
#include "FS.h"

// variáveis globais
bool modo = 0, incrementarSetpoint = true;
int contadorSetpoint = 0;
double setpoint = 10, processValue, controlVariable;
double kp = 45, ki = 9.91, kd = 51.08, ajusteEntrada = 1, ajusteSaida = 1;
unsigned long tempoAtual = 0, tempoAnterior = 0;
unsigned long intervalo = 500;
String dados = "";

// objeto para controle PID
PID controlePID(&processValue, &controlVariable, &setpoint, kp, ki, kd, DIRECT);

// objeto para leitura de nível
Adafruit_ADS1115 ads;

// objeto para controle do inversor
Adafruit_MCP4725 mcp;

// objeto para arquivo de dados
File dataFile;

// saídas digitais
int ledD4 = 12;
int ledD5 = 13;

void setup() {
  // inicialização
  Serial.begin(115200);
  delay(3000);
  Serial.println("Inicializando...");

  // configura e inicializa as saídas digitais
  pinMode(ledD4, OUTPUT);
  pinMode(ledD5, OUTPUT);
  digitalWrite(ledD4, LOW);
  digitalWrite(ledD5, LOW);

  while (!inicializacao()){
    digitalWrite(ledD5, !digitalRead(ledD5));
    delay(250);
  }

  // configura controle PID
  controlePID.SetTunings(kp, ki, kd);
  controlePID.SetOutputLimits(0, 100);
  controlePID.SetMode(AUTOMATIC);  
}

void loop() {  
  processValue = medirNivel();
  controlePID.SetTunings(kp, ki, kd);
  controlePID.Compute();
  ajustarInversor(controlVariable);
  
  // ledD5 heart inverte o estado a cada 500 ms
  // registro de valores a cada 500 ms
  // altera o SP a cada 5 minutos
  tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= intervalo)
  {
    tempoAnterior = tempoAtual;
    digitalWrite(ledD5, !digitalRead(ledD5));

    // se modo 0 envia valores via serial
    // se modo 1 salva valores no cartão SD    
    modo ? salvarDadosSD(SD, "/dados.csv") : enviarValores();      

    // altera o SP entre o intervalo de 10 a 70 com incrementos/decrementos de 10
    if (contadorSetpoint < 600)
      contadorSetpoint++;
    else{
      contadorSetpoint = 0;
      // se chegou no sp mínimo de teste
      if (setpoint <= 10 && !incrementarSetpoint){
        incrementarSetpoint = true;
      }
      // se chegou no sp máximo de teste
      if (setpoint >= 70 && incrementarSetpoint){
        incrementarSetpoint = false;
      }

      // incrementa ou decramenta o sp
      if (setpoint < 70 && incrementarSetpoint)
        setpoint += 10;
      else if (setpoint > 10 && !incrementarSetpoint)
        setpoint -= 10;
    }
  }
  
  // ajusta valores via serial
  if (Serial.available() > 0) Serial.read() == '0' ? modo = false : modo = true;

}