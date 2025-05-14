#include <MatrixMath.h>

const int ledPin = 9;
const int solarPin = A2;

// Simulação da planta
float y = 0;  // Saída simulada
float Ts = 0.01;
float ref = 1.4;
float tau0 = 1.4627;
float tau1 = 0.02402;
float Ke0  = 0.3624;
float Ke1  = -0.05229;
float tensao_atual;
float alpha = 0.2;
float tensaoFiltrada = 0;

float rho = 0;
float tau = 0;
float Ke = 0;
float erro;
float I_erro[2] = {0, 0};

const int N = 2;
mtx_type x[N][1];
mtx_type Uc[1][1];

// Matrizes LPV
mtx_type Y0[N][N] = {
  {0.2219, -0.0784},
  {-0.0784, 0.0392}
};

mtx_type Y1[N][N] = {
  {0.0116, 0.0730},
  {0.0730, 0.4627}
};

mtx_type W0[1][N] = {1.5510, -0.1440};
mtx_type W1[1][N] = {0.0785, -0.0447};

mtx_type Prho[N][N];
mtx_type Wrho[1][N];
mtx_type Krho[1][N];

unsigned long previousMillis;
unsigned long currentMillis;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  previousMillis = millis();
}

void loop() {
  currentMillis = millis();
  if (currentMillis - previousMillis >= Ts * 1000) {
    previousMillis = currentMillis;
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
  // Atualiza parâmetros da planta
  tensao_atual = leituraTensao();
  rho = ref;
  tau = tau0 + tau1 * rho;
  Ke  = Ke0 + Ke1 * rho;

  erro = ref - y;

    if (Uc[0][0]<0.4 || Uc[0][0]>5){I_erro[1] = I_erro[0];} //anti windup clamping (para de acumular IE quando satura)
    else
    {I_erro[1] = I_erro[0] + Ts * erro;} 
  
  //I_erro[1] = I_erro[0] + Ts * erro;

  // Atualiza estado do controlador
  x[0][0] = erro;
  x[1][0] = I_erro[1];

  // LPV: Krho = (W0 + rho*W1)*(Y0 + rho*Y1)^-1
    mtx_type temp[N][N];
    mtx_type temp2[N][N];

    // Calculando Prho = inv(Y0 + rho*Y1);
    Matrix.Copy((mtx_type*)Y1, N, N, (mtx_type*)temp); //temp = Y1
    Matrix.Scale((mtx_type*)temp, N, N, rho); //temp = rho*temp
    Matrix.Add((mtx_type*)Y0, (mtx_type*)temp, N, N, (mtx_type*)temp2); // temp2 = Y0 + rho*Y1
    
    Matrix.Copy((mtx_type*)temp2, N, N, (mtx_type*)Prho); // Prho = temp2
    Matrix.Invert((mtx_type*)Prho, N); //Prho = inv(Prho)

    // Calculando Wrho = W0 + rho*W1
    mtx_type tempW[1][N];
  
    Matrix.Copy((mtx_type*)W1, 1, N, (mtx_type*)tempW); //tempW = W1
    Matrix.Scale((mtx_type*)tempW, 1, N, rho); // tempW = rho*tempW
    Matrix.Add((mtx_type*)W0, (mtx_type*)tempW, 1, N, (mtx_type*)Wrho); // Wrho = W0+tempW 

    // Calculando Krho = Wrho * Prho
    Matrix.Multiply((mtx_type*)Wrho, (mtx_type*)Prho, 1, N, N, (mtx_type*)Krho); // Wrho*Prho

  if (Krho[0][1] < 0) Krho[0][1] = -Krho[0][1];


  //Krho[0][0] = 2;
  //Krho[0][1] = 2;

  // Uc = Krho * x
  Matrix.Multiply((mtx_type*)Krho, (mtx_type*)x, 1, N, 1, (mtx_type*)Uc);


  // Saturação
  if (Uc[0][0] > 5) Uc[0][0] = 5;
  if (Uc[0][0] < 0) Uc[0][0] = 0;

  // Simula planta: y(k+1) = y(k) - (Ts/tau)*y(k) + (Ts/tau)*Ke*u(k)
  y = y - (Ts / tau) * y + (Ts / tau) * Ke * Uc[0][0];

  // Atualiza integral
  I_erro[0] = I_erro[1];

  // PWM de saída (opcional)
  analogWrite(ledPin, (int)(Uc[0][0] / 5.0 * 255));

  // Print para análise
  Serial.print(Uc[0][0]);
  Serial.print(", ");
  Serial.print(erro);
  Serial.print(", ");
  Serial.print(tensao_atual);
  Serial.print(", ");
  Serial.println(y);
  // Serial.print(", ");
  // Serial.println(Krho[0][0]);
  // Serial.print(", ");
  // Serial.println(Krho[0][1]);
}
