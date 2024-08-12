#include "Servo.h"

float tensao = 0.00, corrente = 0.00, comando = 0.00, wc = 0.00, potreal, potteor;
int period = 20, cmdESC = 0;

Servo ESC;

void setup() {
  pinMode(0, INPUT_ANALOG);
  pinMode(5, INPUT_ANALOG);
  pinMode(7, INPUT);
  //Serial.begin(115200);
  ESC.attach(6);
}
void loop() {
  wc = pulseIn(7, HIGH);
  tensao = (analogRead(0)*25.00/4096.00);
  corrente = (analogRead(5)*25.00/1024.00);
  comando = (wc - 943.00)/9.34;
  potreal = corrente*tensao;
  potteor = 6.8*comando;
  cmdESC = cmdESC - ((potreal - potteor)*800/680);
  ESC.write(cmdESC);
  /*
  Serial.print("Tensão: ");
  Serial.print(tensao);
  Serial.print("V | Corrente: ");
  Serial.print(corrente);
  Serial.print("A | Comando: ");
  Serial.print(comando);
  Serial.println("%");
  Serial.print("Potência real: ");
  Serial.print(potreal);
  Serial.print("W | Potência teórica: ");
  Serial.print(potteor);
  Serial.print("W ESC: ");
  Serial.println(cmdESC);
  Serial.println("");*/
}
