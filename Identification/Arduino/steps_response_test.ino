// Declaração dos pinos
int solarPin = A2; // Pino de entrada do painel solar
int ledPin = 9;    // Pino de saída para a LED

// Parâmetros do sistema
float alpha = 0.1;            // Fator do filtro passa-baixa
float tensaoFiltrada = 0;     // Valor inicial da tensão filtrada
int delayTime = 100;          // Intervalo entre as medições (em milissegundos)
int tempoPorDegrau = 10000;   // Tempo em cada degrau (em milissegundos)

// Configuração inicial
void setup() {
  Serial.begin(9600);          // Inicializa o monitor serial
  pinMode(ledPin, OUTPUT);     // Define o pino da LED como saída

  // Cabeçalho para os dados no monitor serial
  Serial.println("Tempo (ms),Tensão de Entrada (V),Tensão Filtrada (V)");
}

// Loop principal
void loop() {
  // Parâmetros para os degraus
  float tensaoInicial = 0.0; // Tensão inicial (0V)
  float tensaoFinal = 5.0;   // Tensão máxima (5V)
  float incremento = 0.5;    // Incremento de tensão (0,5V)
  
  // Loop para aplicar os degraus
  for (float tensaoEntrada = tensaoInicial; tensaoEntrada <= tensaoFinal; tensaoEntrada += incremento) {
    int valorPWM = (tensaoEntrada / 5.0) * 255; // Converte tensão para PWM (0 a 255)
    analogWrite(ledPin, valorPWM);              // Ajusta o brilho da LED
    unsigned long tempoInicial = millis();     // Marca o tempo inicial do degrau

    // Coleta dados enquanto o degrau está aplicado
    while (millis() - tempoInicial < tempoPorDegrau) {
      leituraTensao(tensaoEntrada); // Mede e registra os valores
      delay(delayTime);             // Intervalo entre medições
    }
  }

  // Aguarda antes de reiniciar o ciclo (opcional)
  analogWrite(ledPin, 0); // Desliga a LED após o último degrau
  delay(30000);           // Aguarda 30 segundos antes de reiniciar
}

// Função para medir e filtrar a tensão do painel solar
void leituraTensao(float tensaoEntrada) {
  unsigned long tempoAtual = millis(); // Marca o tempo atual
  
  // Leitura da tensão no painel solar
  int valorLido = analogRead(solarPin);
  float tensao = (valorLido * 5.0) / 1023.0; // Converte para tensão (0 a 5V)
  
  // Filtro passa-baixa
  tensaoFiltrada = alpha * tensao + (1 - alpha) * tensaoFiltrada;

  // Envia os dados para o monitor serial
  //Serial.print(tempoAtual);          // Tempo (ms)
 // Serial.print(",");
  Serial.print(tensaoEntrada, 2);    // Entrada da LED (tensão)
  Serial.print(",");
  Serial.println(tensaoFiltrada, 2); // Tensão filtrada do painel solar
}
