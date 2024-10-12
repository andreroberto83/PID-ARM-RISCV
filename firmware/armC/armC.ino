/**************************************************************************************
* Título: Controle PID para planta de processos contínuos                             *
* Autor: André Roberto da Silva                                                       *
* Plataforma: Raspberry Pi PICO / PICO W                                              *
* MCU: RP2040                                                                         *
* Núcleo: ARM Cortex M0+                                                              *
***************************************************************************************
* --------------------------------------------------------                            *
* |                 PROTOCOLO DE ESCRITA                 |                            *
* --------------------------------------------------------                            *
* |    COMANDO     |     DESCRIÇÃO       |    Range      |                            *
* --------------------------------------------------------                            *
* | $WKPVALOR\r\n  | Seta o valor de Kp  |    0 a 100    |                            *
* --------------------------------------------------------                            *
* | $WKIVALOR\r\n  | Seta o valor de Ki  |    0 a 100    |                            *
* --------------------------------------------------------                            *
* | $WKDVALOR\r\n  | Seta o valor de Kd  |    0 a 100    |                            *
* --------------------------------------------------------                            *
* | $WSPVALOR\r\n  | Seta o valor de SP  |    0 a 100    |                            *
* --------------------------------------------------------                            *
* | $WCIVALOR\r\n  | Correção IN         |    0.0 a 1.0  |                            *
* --------------------------------------------------------                            *
* | $WCOVALOR\r\n  | Correção OUT        |    0.0 a 1.0  |                            *
* --------------------------------------------------------                            *
* | $WMDVALOR\r\n  | Seta o modo         |    0 a 2      |                            *
* --------------------------------------------------------                            *
**************************************************************************************/

// bibliotecas
#include <PID_v1.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP4725.h>
#include <SPI.h>
#include <SD.h>

// pinos de conexão SPI 
const int _MISO = 16;
const int _MOSI = 19;
const int _CS = 17;
const int _SCK = 18;

// protótipos de funções
double medirNivel();
void ajustarInversor(double controlVariable);
void setarParametro();
void formatarDados(double processValue, double controlVariable, double setPoint, double kp, double ki, double kd);
void enviarValores();
void salvarDadosSD();

// variáveis globais
bool modo = 1, erroMedirNivel = 0, erroAjustarInversor = 0, erroSalvarDadosSD = 0, spUp = true;
int contadorSetPoint = 0;
double setPoint = 10, processValue, controlVariable;
double kp = 45, ki = 9.91, kd = 51.08, ajusteIn = 1, ajusteOut = 1;
unsigned long tempoAtual = 0, tempoAnterior = 0;
unsigned long intervalo = 500;
String dados = "";

// objeto para controle PID
PID controlePID(&processValue, &controlVariable, &setPoint, kp, ki, kd, DIRECT);

// objeto para leitura de nível
Adafruit_ADS1115 ads;

// objeto para controle do inversor
Adafruit_MCP4725 mcp;

// objeto para arquivo de dados
File dataFile;

// saídas digitais
int led = 25;
int relayInversor = 9;

void setup() {
  // inicialização
  Serial.begin(115200);
  delay(3000);
  Serial.println("Inicializando...");
  
  // inicializa I2C
  Wire.begin();
  delay(250);

  // configura e inicializa as saídas digitais
  pinMode(led, OUTPUT);
  pinMode(relayInversor, OUTPUT);
  digitalWrite(led, LOW);  
  digitalWrite(relayInversor, LOW);

  // rotina de inicialização ADS1115
  // ganho 1x   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Falha ao inicializar AD!");
    for (int i = 0; i < 15; i++){
      digitalWrite(led, !digitalRead(led));
      delay(250);
    }
    while(1);
  }
  Serial.println("AD inicializado com sucesso!");
 
  // rotina de inicialização DA MCP4725
  if (!mcp.begin(0x60)) {
    Serial.println("Falha ao inicializar DA!");
    for (int i = 0; i < 15; i++){
      digitalWrite(led, !digitalRead(led));
      delay(250);
    }
    while(1);
  }
  mcp.setVoltage(0, false);
  Serial.println("DA inicializado com sucesso!");

  // rotina de inicialização SD Card
  SPI.setRX(_MISO);
  SPI.setTX(_MOSI);
  SPI.setSCK(_SCK);
  if (!SD.begin(_CS)){
    Serial.println("Falha ao inicializar SD Card!");
    for (int i = 0; i < 15; i++){
      digitalWrite(led, !digitalRead(led));
      delay(250);
    }
    while(1);
  }
  Serial.println("SD Card inicializado com sucesso!");

  // inicializa o controle PID
  controlePID.SetTunings(kp, ki, kd);
  controlePID.SetOutputLimits(0, 100);
  controlePID.SetMode(AUTOMATIC);

  // pulso para ligar inversor
  digitalWrite(relayInversor, HIGH);
  delay(1000);
  digitalWrite(relayInversor, LOW);

  // valores indicando que ocorreu um reset
  formatarDados(-1, -1, -1, -1, -1, -1);
  enviarValores();  
  salvarDadosSD(); 
}

void loop() {  
  processValue = medirNivel();
  controlePID.SetTunings(kp, ki, kd);
  controlePID.Compute();

  if (!kp && !ki && !kd)
    ajustarInversor(0.0);  
  else{
    ajustarInversor(controlVariable);
  }    
  
  // led heart inverte o estado a cada 500 ms
  // registro de valores a cada 500 ms
  // altera o SP a cada 5 minutos
  tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= intervalo)
  {
    tempoAnterior = tempoAtual;
    digitalWrite(led, !digitalRead(led));

    // se modo 0 envia valores via serial
    // se modo 1 salva valores no cartão SD
    // se modo 2 envia valores via serial e salva valores no cartão SD
    if (modo == 0 || modo == 2){
      enviarValores();
      if (modo == 2)        
        salvarDadosSD();
    }
    else
      salvarDadosSD();

    // altera o SP entre o intervalo de 10 a 70 com incrementos/decrementos de 10
    if (contadorSetPoint < 600)
      contadorSetPoint++;
    else{
      contadorSetPoint = 0;
      // se chegou no sp mínimo de teste
      if (setPoint <= 10 && !spUp){
        spUp = true;
      }
      // se chegou no sp máximo de teste
      if (setPoint >= 70 && spUp){
        spUp = false;
      }

      // incrementa ou decramenta o sp
      if (setPoint < 70 && spUp)
        setPoint += 10;
      else if (setPoint > 10 && !spUp)
        setPoint -= 10;
    }
  }
  
  // ajusta valores via serial
  if (Serial.available() > 0){
    setarParametro();
  }
}

// função para medição do nível
// retorno 0 a 100 ou -1 erro
double medirNivel(){
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float tensao = ads.computeVolts(adc0);
  // 4 mA = 0 - 20 mA = 100
  double nivel = ((tensao * 6.667) - 4) * 6.25 * ajusteIn;
  // se houve erro na leitura
  if (nivel < 0 || nivel > 100){
    erroMedirNivel = 1;
    digitalWrite(led, HIGH);    
    return -1;
  }
  if (erroMedirNivel){
      erroMedirNivel = 0;
      digitalWrite(led, LOW);
  }        
  return nivel;
}

void ajustarInversor(double out){
  // se valor inválido
  if (out < 0 || out > 100){
    erroAjustarInversor = 1;
    digitalWrite(led, HIGH);
    mcp.setVoltage(uint32_t(0), false);
  }
  else{
    // ajusta a tensão de saída
    double tensao = out * 0.03 * ajusteOut;
    // converte 0 a 3.3 V para 0 a 4095 (12 bits)
    double aux = ((tensao / 3) * 4095.0);
    mcp.setVoltage(uint32_t(aux), false);
    if (erroAjustarInversor){
      erroAjustarInversor = 0;
      digitalWrite(led, LOW);
    }
  }  
}

void setarParametro(){
  String parametro = Serial.readStringUntil('\n');
  Serial.println(parametro);
  if (parametro[0] == '$' && parametro[1] == 'W'){
    if(parametro[2] == 'K'){
      switch (parametro[3]) {
        case 'P' : try {String sAux = parametro.substring(4, 9); double dAux = sAux.toDouble(); kp = dAux;} catch (String e) {Serial.println(e);}; break;
        case 'I' : try {String sAux = parametro.substring(4, 9); double dAux = sAux.toDouble(); ki = dAux;} catch (String e) {Serial.println(e);}; break;
        case 'D' : try {String sAux = parametro.substring(4, 9); double dAux = sAux.toDouble(); kd = dAux;} catch (String e) {Serial.println(e);}; break;
        default: Serial.println("Comando inválido!");
      }
    }
    else if (parametro[2] == 'S'){
      if (parametro[3] == 'P'){
        try{
          String sAux = parametro.substring(4, 9);
          double dAux = sAux.toDouble();
          setPoint = dAux;
        }
        catch (String e) {
          Serial.println(e);
        }
      }
      else
        Serial.println("Comando inválido!");
    }
    else if(parametro[2] == 'C'){
      switch (parametro[3]) {
        case 'I' : try {String sAux = parametro.substring(4, 9); double dAux = sAux.toDouble(); ajusteIn = dAux;} catch (String e) {Serial.println(e);}; break;
        case 'O' : try {String sAux = parametro.substring(4, 9); double dAux = sAux.toDouble(); ajusteOut = dAux;} catch (String e) {Serial.println(e);}; break;
        default: Serial.println("Comando inválido!");
      }
    }
    else if(parametro[2] == 'M'){
      if (parametro[3] == 'D') {
        try {
          String sAux = parametro.substring(4, 9);
          int iAux = sAux.toInt();
          if (iAux >=0 && iAux < 3)
            modo = iAux;
        } 
        catch (String e){
            Serial.println(e);
        }         
      }
      else
        Serial.println("Comando inválido!");
    }
    else
      Serial.println("Comando inválido!");
  }
  else
    Serial.println("Comando inválido!");
}

void formatarDados(double processValue, double controlVariable, double setPoint, double kp, double ki, double kd){
  dados = "";
  dados = dados + millis() + "," + processValue + "," + controlVariable + "," + setPoint + "," + kp + "," + ki + "," + kd;
}

void enviarValores(){
  formatarDados(processValue, controlVariable, setPoint, kp, ki, kd);
  Serial.println(dados);
}

void salvarDadosSD(){
  // cria objeto para abertura de arquivo csv
  dataFile = SD.open("dados.csv", FILE_WRITE);
  // se abriu arquivo inclui dados senão
  // sinaliza o erro de gravação
  if (dataFile){
    formatarDados(processValue, controlVariable, setPoint, kp, ki, kd);
    dataFile.println(dados);
    dataFile.close();
    if (erroSalvarDadosSD){
      erroSalvarDadosSD = 0;
      digitalWrite(led, LOW);
    }    
  }
  else{
    erroSalvarDadosSD = 1;
    digitalWrite(led, HIGH);
  }
}