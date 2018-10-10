#include <VarSpeedServo.h>

VarSpeedServo servo1, servo2, servo3, servo4;
int velocidadeMaxima = 20;

/*
 * velocidades[0] é referente a velocidade da base
 * velocidades[1] é referente a velocidade do ombro
 * velocidades[2] é referente a velocidade do cotovelo
 */
int velocidades[3] = {20,20,20};

int angAtualBase = 0;
int angAtualCotovelo = 0;
int angAtualOmbro = 0;


int opcaoMenu;

double posAtualX, posAtualY, posAtualZ = 0;

/*
 * Encontra a maior distância
 */
double compareNumbers(double x, double y, double z) {
  double result = abs(posAtualX - x);
  if (result < abs(posAtualY - y))
    result = abs(posAtualY - y);
  if (result < abs(posAtualZ - z))
    result = abs(posAtualZ - z);

  return result;
}

void calculaVelocidade(int angBase, int angOmbro,  int angCotovelo) {
  /*
   * A variável "maior" vai identificar qual é a maior distância angular
   * 0 -> Ângulo da Base
   * 1 -> Ângulo do Ombro
   * 2 -> Ângulo do Cotovelo
   */
    
  int maior = 0;
  int deltaBase = abs(angBase - angAtualBase);
  int deltaOmbro = abs(angOmbro - angAtualOmbro);
  int deltaCotovelo = abs(angCotovelo - angAtualCotovelo);
  
  int aux = deltaBase;

  if(aux < deltaOmbro) {
    maior = 1;
    aux = deltaOmbro;
  }
  if(aux < deltaCotovelo) {
    maior = 2;
    aux = deltaCotovelo;
  }

  switch(maior) {
    case 0:
      velocidades[0] = velocidadeMaxima;
      velocidades[1] = (velocidadeMaxima*deltaOmbro)/deltaBase;
      velocidades[2] = (velocidadeMaxima*deltaCotovelo)/deltaBase;
      break;
    case 1:
      velocidades[0] = (velocidadeMaxima*deltaBase)/deltaOmbro;
      velocidades[1] = velocidadeMaxima;
      velocidades[2] = (velocidadeMaxima*deltaCotovelo)/deltaOmbro;
      break;    
    case 2:
      velocidades[0] = (velocidadeMaxima*deltaBase)/deltaCotovelo;
      velocidades[1] = (velocidadeMaxima*deltaOmbro)/deltaCotovelo;
      velocidades[2] = velocidadeMaxima;
      break;
    default:
      Serial.println("Deu ruim no calculo da velocidade");
  }
  
}

void movimentaServos(int angBase, int angOmbro,  int angCotovelo) {
  velocidades[0] = 0;
  velocidades[1] = 0;
  velocidades[2] = 0;
  
  calculaVelocidade(angBase, angOmbro, angCotovelo);
  
  servo1.write(angBase,velocidades[0],false);
  servo2.write(angOmbro,velocidades[1],false);
  servo3.write(angOmbro,velocidades[1],false);
  servo4.write(angCotovelo,velocidades[2],true);
  
}

void cinematicaDireta() {
  int auxiliarBase, auxiliarOmbro, auxiliarCotovelo;
  Serial.print("Angulo atual da base: ");
  Serial.println(angAtualBase);
  Serial.println("Agora digite o novo angulo da base: ");
  
  while(!Serial.available());
  auxiliarBase = Serial.parseInt();
  Serial.println(auxiliarBase);

  Serial.print("Angulo atual do ombro: ");
  Serial.println(angAtualOmbro);
  Serial.println("Agora digite o novo angulo do ombro:");
  
  while(!Serial.available());
  auxiliarOmbro = Serial.parseInt();
  Serial.println(auxiliarOmbro);

  Serial.print("Angulo atual do cotovelo: ");
  Serial.println(angAtualCotovelo);
  Serial.println("Agora digite o novo angulo do cotovelo:");

  while(!Serial.available());
  auxiliarCotovelo = Serial.parseInt();
  Serial.println(auxiliarCotovelo);

  movimentaServos(auxiliarBase, auxiliarOmbro, auxiliarCotovelo);
}

void cinematicaInversa(double x, double y, double z) {
  double pi = 3.14159;
  // dados do robo
  double x3 = x; // posição x do robo 
  double y3 = y; // posição y do robo
  double z3 = z; //altura do robo
  double l1 = 0;
  double l2 = 12.5;
  double l3 = 14.5;
  double d1 = 2.5;
  
  // calculo dos auxiliares
  double X3 = pow(x3,2);
  double Y3 = pow(y3,2);
  double Z3 = pow(z3,2);
  double L2 = pow(l2,2);
  double L3 = pow(l3,2);
  double k1 = X3+Y3;
  double r1 = sqrt(k1); //equação 2
  double r2 = z3-d1; // equação 3
  double R1 = pow(r1,2);
  double R2 = pow(r2,2);
  double k2 = R1+R2;
  double r3 = sqrt(k2); //equação 5
  double R3 = pow(r3,2);
  double k3 =(L3-L2-R3)/(-2*l2*r3); 
  double k4 = (R3-L2-L3)/(-2*l2*l3);
  
  // calculo dos angulos
  double alfa1 = acos(k3); //equação 6
  double alfa2 = atan2(r2,r1); // equação 4
  double alfa3 = acos(k4); // equação 8
  double tet1 = atan2(y3,x3); // equação 1
  double tet2 = alfa2 - alfa1; // equação 7
  double conv3 = (alfa3*180)/pi;
  double tet3 = 180 - conv3;
  double conv1 = (tet1*180)/pi;
  double conv2 = (tet2*180)/pi;
  
  // angulos
  int angBase = conv1; 
  int angOmbro = conv2; 
  int angCotovelo = tet3;

  movimentaServos(angBase, angOmbro, angCotovelo);
}

void routePlanner(double x, double y, double z) {
  // Resgatando maior valor 
  double aux = compareNumbers(x, y, z);
  
  //Modificar a variável abaixo para aumentar/diminuir o passo
  double passo = 1/aux;

  double auxX = posAtualX;
  double auxY = posAtualY;
  double auxZ = posAtualZ;
  
  for (double i = 0; i <= 1; i+= passo) {
    auxX = posAtualX + i*(x-posAtualX);
    auxY = posAtualY + i*(y-posAtualY);
    auxZ = posAtualZ + i*(z-posAtualZ);

    cinematicaInversa(auxX, auxY, auxZ);
  }
}

void imprimeMenu() {
  Serial.println("\n\nEscolha uma opcao para comecar:\n1-Cinematica Direta\n2-Cinematica Inversa\n");
}

void Menu (int opcaoMenu) {
  switch (opcaoMenu) {
    case 1:
      cinematicaDireta();            
      break;
    default:
      return;
  }
}

void setup() {
  Serial.begin(9600);

  //Servo da base
  servo1.attach (3);

  //Servos do ombro
  servo2.attach (5);
  servo3.attach (6);

  //Servo do cotovelo
  servo4.attach (9);

}

void loop() {
  imprimeMenu();
  //verifica se tem dados diponível para leitura
  if (Serial.available()){
    opcaoMenu = Serial.parseInt(); //le byte mais recente no buffer da serial
    Serial.println(opcaoMenu);   //reenvia para o computador o dado recebido
    Menu(opcaoMenu);
  }
}
