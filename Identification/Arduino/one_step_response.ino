// Declaração dos pinos
int solarPin = A2; // Pino de entrada do painel solar
int ledPin = 9;    // Pino de saída para a LED

// Parâmetros do sistema
float alpha = 0.1;            // Fator do filtro passa-baixa
float tensaoFiltrada = 0;     // Valor inicial da tensão filtrada
int delayTime = 100;          // Intervalo entre as medições (em milissegundos)
unsigned long tempoTotal = 5000; // Tempo total de coleta após o degrau (em milissegundos)
int tempoEspera = 5000;       // Tempo de espera no nível zero antes do degrau (em milissegundos)

// Configuração inicial
void setup() {
  Serial.begin(9600);          // Inicializa o monitor serial
  pinMode(ledPin, OUTPUT);     // Define o pino da LED como saída

  // Cabeçalho para os dados no monitor serial
  Serial.println("Tempo (ms),Tensão de Entrada (V),Tensão Filtrada (V)");
}

// Loop principal
void loop() {
  Serial.println("Esperando no nível zero...");

  // Espera no nível zero (LED desligada)
  analogWrite(ledPin, 0); // LED em brilho zero
  unsigned long tempoInicial = millis();
  while (millis() - tempoInicial < tempoEspera) {
    leituraTensao(0); // Mede e registra os valores no nível zero
    delay(delayTime);
  }

  Serial.println("Aplicando o degrau...");
  
  // Parâmetros do degrau
  int valorInicial = 0;   // Brilho inicial da LED (0%)
  int valorFinal = 255;   // Brilho final da LED (100%)

  // Aplica o degrau
  aplicarDegrau(valorInicial, valorFinal);

  // Aguarda antes de reiniciar o ciclo (opcional)
  delay(10000); // 10 segundos de espera
}

// Função para aplicar o degrau
void aplicarDegrau(int valorInicial, int valorFinal) {
  analogWrite(ledPin, valorInicial); // Define o brilho inicial
  delay(100); // Curto atraso para estabilizar

  unsigned long tempoInicial = millis(); // Marca o tempo inicial do degrau
  
  analogWrite(ledPin, valorFinal); // Aplica o degrau (brilho final)
  
  // Coleta os dados por um período definido
  while (millis() - tempoInicial < tempoTotal) {
    leituraTensao(valorFinal); // Mede e registra os valores
    delay(delayTime);          // Intervalo entre medições
  }
}

// Função para medir e filtrar a tensão do painel solar
void leituraTensao(int valorEntrada) {
  unsigned long tempoAtual = millis(); // Marca o tempo atual
  
  // Leitura da tensão no painel solar
  int valorLido = analogRead(solarPin);
  float tensao = (valorLido * 5.0) / 1023.0; // Converte para tensão (0 a 5V)
  
  // Filtro passa-baixa
  tensaoFiltrada = alpha * tensao + (1 - alpha) * tensaoFiltrada;

  // Calcula a entrada (brilho normalizado da LED)
  float tensaoEntrada = (valorEntrada / 255.0) * 5.0;

  // Envia os dados para o monitor serial
  //Serial.print(tempoAtual);          // Tempo (ms)
  //Serial.print(",");
  Serial.print(tensaoEntrada, 2);    // Entrada da LED (tensão)
  Serial.print(",");
  Serial.println(tensaoFiltrada, 2); // Tensão filtrada do painel solar
}
