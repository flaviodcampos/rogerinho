#include <VarSpeedServo.h>

VarSpeedServo serve1, serve2, serve3, serve4;

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

void movimentaServos(double x, double y, double z) {
  
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

    movimentaServos(auxX, auxY, auxZ);
  }
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}
