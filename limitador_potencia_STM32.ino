#include "Servo.h"

#define correnteII 30.0 //corrente de regulagem
#define potlimit 680.0 //limite de potência
#define fcv 168.61 //fator de correção da tensão
#define senseUP 0.001 //velocidade de resposta do limitador subindo (quanto menor mais rápido)
#define senseDOWN 0.005 //velocidade de resposta do limitador descendo (quanto menor mais rápido)
#define ansdelay 500 //coeficiente de atraso na resposta do limitador (muito alto o torna difícil de pilotar, muito baixo torna o limitador instável)
#define trashold 350.0 //trashold de ativação do limitador (em watts)
#define samples 50000 //quantidade de amostras para calibragem
#define window 20 //janela de amostras para média de corrente

int ctt = 0;
int routine = 1;
float soma, soma2, correcao = 3984.05, correcao2 = -21.34;
float a1, a2, a3, a4, a5;
float tensao = 0.00, corrente = 0.00, comando = 0.00, potreal, potteor;
float cmdESC = 0;

Servo ESC;

void setup() {
  pinMode(PA0, INPUT_ANALOG);
  pinMode(PA5, INPUT_ANALOG);
  pinMode(PA7, INPUT);
  pinMode(PB13, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(PA4), potato, RISING);
  digitalWrite(PB13, LOW);
  Serial.begin(115200);
  ESC.attach(PA6,942,1942);
}

void potato(){
  ctt = routine;
}

void loop() {
// Rotina 1
    while (ctt == 1){
      soma = 0;
      Serial.println("Rotina 1, aguarde...");
      digitalWrite(PB13, HIGH);
      for (int ct2 = 0; ct2 < samples; ct2 ++){
        ESC.write(0);
        a5 = analogRead(PA5);
        a4 = analogRead(PA5);
        a3 = analogRead(PA5);
        a2 = analogRead(PA5);
        a1 = analogRead(PA5);
        corrente = (a1+a2+a3+a4+a5)/5.0;
        soma = soma + (corrente);
      }
      correcao = soma/samples;
      Serial.println(correcao);
      delay(2000);
      digitalWrite(PB13, LOW);
      routine = 2;
      ctt = 0;
    }
//Rotina 2
    if (ctt == 2){
      Serial.println("Rotina 2, acelere até:");
      Serial.print(correnteII);
      Serial.println("A");
      digitalWrite(PB13, HIGH);
      delay(1000);
      routine = 3;
      while(ctt == 2){
        cmdESC = (pulseIn(PA7, HIGH) - 942)*0.18;
        ESC.write(cmdESC);
      }
      digitalWrite(PB13, LOW);
    }
//Rotina 3
    while (ctt == 3){
      soma2 = 0;
      Serial.println("Rotina 3, manter em:");
      Serial.print(correnteII);
      Serial.println("A");
      digitalWrite(PB13, HIGH);
      for (int ct2 = 0; ct2 < samples; ct2 ++){
        a5 = analogRead(PA5);
        a4 = analogRead(PA5);
        a3 = analogRead(PA5);
        a2 = analogRead(PA5);
        a1 = analogRead(PA5);
        corrente = (a1+a2+a3+a4+a5)/5.0;
        soma2 = soma2 + ((corrente-correcao)/correnteII);
      }
    correcao2 = soma2/samples;
    Serial.println(correcao2);
    ESC.write(0);
    delay(5000);
    digitalWrite(PB13, LOW);
    routine = 1;
    ctt = 0;
  }
//Rotina 0 (limitador)
  while (ctt == 0){
  for (int ct2 = 0; ct2 < window; ct2++){
    tensao = tensao + (analogRead(PA0)/fcv);
    a1 = a1 + analogRead(PA5);
    ESC.write(cmdESC);
  }
  tensao = tensao/window;
  corrente = ((a1/window) - correcao)/correcao2;
  a1 = 0;
  potreal = corrente*tensao;
  comando = (pulseIn(PA7, HIGH) - 942)/10.0;
  potteor = potlimit*comando/100;
    if (potteor < trashold)cmdESC = comando*0.9;
    else {  cmdESC = cmdESC + (potteor - potreal)/ansdelay;
/*      if (potteor > potreal){
        cmdESC = cmdESC + senseUP*(potteor/potreal);
        ESC.write(cmdESC);
      }
      else{
        cmdESC = cmdESC - senseDOWN*(potreal/potteor);
        ESC.write(cmdESC);
      }*/
    }
  Serial.print("Tensão: ");
  Serial.print(tensao);
  Serial.print("V | Corrente: ");
  Serial.print(corrente);
  Serial.print("A | Comando: ");
  Serial.print(comando);
  Serial.print("% | Potência real: ");
  Serial.print(potreal);
  Serial.print("W | Potência teórica: ");
  Serial.print(potteor);
  Serial.print("W | ESC: ");
  Serial.print(cmdESC/1.8);
  Serial.println("%");
  }
}
