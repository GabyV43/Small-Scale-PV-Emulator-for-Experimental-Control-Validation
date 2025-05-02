#include <MatrixMath.h>

const int solarPin = A2;
const int ledPin = 9;

float alpha = 0.01;
float tensaoFiltrada = 0;
mtx_type I_erro[2] = {0, 0};
float erro = 0;
float Ts = 0.01;
const int N = 2;
float nova_entrada = 0;
float t = 0;
float t_anterior = 0;
float tensao_atual = 0;
float ref = 1;
float tau0 = 1.4627;
float tau1 = 0.02402;
float Ke0  = 0.3624;
float Ke1  = -0.05229;
float tau = 0;
float Ke = 0;
float rho = 0;
float Uc = 0;

// Matriz de estados
mtx_type x[N][1];

// Matrizes LPV
mtx_type Y0[N][N] = {
  {0.2219, -0.0784},
  {-0.0784, 0.0392}
};

mtx_type Y1[N][N] = {
  {0.0116, 0.0730},
  {0.0730, 0.4627}
};

mtx_type W0[1][N] = {
  {1.5510, -0.1440}
};

mtx_type W1[1][N] = {
  {0.0785, -0.0447}
};

mtx_type Prho[N][N]; // Inversa da matriz resultante
mtx_type Wrho[1][N]; // Vetor calculado
mtx_type Krho[1][N]; // Resultado final

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  t = millis();
  if (t - t_anterior >= 100) {
    t_anterior = t;
    malhafechada();
  }
}

float leituraTensao() {
  int valorLido = analogRead(solarPin);
  float tensao = (valorLido * 5.0) / 1023.0;
  tensaoFiltrada = alpha * tensao + (1 - alpha) * tensaoFiltrada;
  return tensaoFiltrada;
}

void malhafechada() {
  tensao_atual = leituraTensao();
  erro = ref - tensao_atual;
  I_erro[1] = I_erro[0] + Ts*erro;

  // Aplica anti-windup ANTES de usar
 // if (I_erro[1] > 5) I_erro[1] = 5;
//  if (I_erro[1] < -5) I_erro[1] = -5;

  // Usa os valores no vetor de estado
  x[0][0] = erro;
  x[1][0] = I_erro[1];

  // Atualiza erro antigo
  I_erro[0] = I_erro[1];

  rho = ref;
  tau = tau0 + tau1 * rho;
  Ke  = Ke0 + Ke1 * rho;

  mtx_type temp[N][N], temp2[N][N], tempW[1][N];

  // Prho = inv(Y0 + rho*Y1)
  Matrix.Copy((mtx_type*)Y1, N, N, (mtx_type*)temp);
  Matrix.Scale((mtx_type*)temp, N, N, rho);
  Matrix.Add((mtx_type*)Y0, (mtx_type*)temp, N, N, (mtx_type*)temp2);
  Matrix.Copy((mtx_type*)temp2, N, N, (mtx_type*)Prho);
  Matrix.Invert((mtx_type*)Prho, N);

  // Wrho = W0 + rho*W1
  Matrix.Copy((mtx_type*)W1, 1, N, (mtx_type*)tempW);
  Matrix.Scale((mtx_type*)tempW, 1, N, rho);
  Matrix.Add((mtx_type*)W0, (mtx_type*)tempW, 1, N, (mtx_type*)Wrho);

  // Krho = Wrho * Prho
  Matrix.Multiply((mtx_type*)Wrho, (mtx_type*)Prho, 1, N, N, (mtx_type*)Krho);

  // Uc = Krho * x
  Matrix.Multiply((mtx_type*)Krho, (mtx_type*)x, 1, N, 1, (mtx_type*)&Uc);

  // Saturação
  if (Uc > 5) Uc = 5;
  if (Uc < 0) Uc = 0;

  nova_entrada = map(Uc * 100, 0, 500, 0, 255);  // 0-5V para 0-255 PWM
  analogWrite(ledPin, nova_entrada);

  // Debug
  Serial.print(Uc);
  Serial.print(", ");
  Serial.print(rho);
  Serial.print(", ");
  Serial.println(tensao_atual);
}
