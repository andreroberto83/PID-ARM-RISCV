// pinos de conexão SPI
const int _SCK = 2;
const int _MISO = 10;
const int _MOSI = 3;
const int _CS = 7;

bool inicializacao(){
  // inicializa I2C
  Wire.begin(4, 5);
  delay(250);

  // inicializa SPI
  SPI.begin(_SCK, _MISO, _MOSI, _CS);
  
  // rotina de inicialização ADS1115
  // ganho 1x   +/- 4.096V  1 bit = 2mV      0.125mV
  ads.setGain(GAIN_ONE);
  if (!ads.begin(0x48)) {
    Serial.println("Falha ao inicializar AD!");
    return false;  
  }
  Serial.println("AD inicializado com sucesso!");
 
  // rotina de inicialização DA MCP4725
  if (!mcp.begin(0x60)) {
    Serial.println("Falha ao inicializar DA!");
    return false;
  }
  mcp.setVoltage(0, false);
  Serial.println("DA inicializado com sucesso!");
  
  // rotina de inicialização SD Card
  if (!SD.begin(_CS)){
    Serial.println("Falha ao inicializar SD Card!");
    return false;
  }
  Serial.println("SD Card inicializado com sucesso!");

  return true;  
}

// função para medição do nível
// retorno 0 a 100 ou -1 erro
double medirNivel(){
  int16_t adc0 = ads.readADC_SingleEnded(0);
  float tensao = ads.computeVolts(adc0);
  // 4 mA = 0 - 20 mA = 100
  double nivel = ((tensao * 6.667) - 4) * 6.25 * ajusteEntrada;
  // se houve erro na leitura
  if (nivel < 0 || nivel > 100) return -1;
  return nivel;
}

void ajustarInversor(double saida){
  // se valor inválido
  if (saida < 0 || saida > 100)
    mcp.setVoltage(uint32_t(0), false);
  else{
    // ajusta a tensão de saída
    double tensao = saida * 0.03 * ajusteSaida;
    // converte 0 a 3.3 V para 0 a 4095 (12 bits)
    double aux = ((tensao / 3) * 4095.0);
    mcp.setVoltage(uint32_t(aux), false);    
  }  
}

void formatarDados(double processValue, double controlVariable, double setpoint, double kp, double ki, double kd){
  dados = "";
  dados = dados + millis() + "," + processValue + "," + controlVariable + "," + setpoint + "," + kp + "," + ki + "," + kd;
}

void enviarValores(){
  formatarDados(processValue, controlVariable, setpoint, kp, ki, kd);
  Serial.println(dados);
}

void salvarDadosSD(fs::FS &fs, const char *path){
  formatarDados(processValue, controlVariable, setpoint, kp, ki, kd);
  // cria objeto para abertura de arquivo csv
  dataFile = fs.open(path, FILE_APPEND);
  if (dataFile){
    dataFile.print(dados);
    dataFile.close(); 
  } 
}