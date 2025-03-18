#include "Servo.h"
#include <EEPROM.h>

#define correnteII 30.0 //corrente de regulagem
#define potlimit 660.0 //limite de potência
//#define fcv 168.61 //fator de correção da tensão (limitador 1)
#define fcv 179.8 //fator de correção da tensão (limitador 2)
#define ansdelay 550 //coeficiente de atraso na resposta do limitador (muito alto o torna difícil de pilotar, muito baixo torna o limitador instável)
#define trashold 5.0 //trashold de ativação do limitador (em watts)
#define samples 50000 //quantidade de amostras para calibragem
#define window 30 //janela de amostras para média de corrente
#define EMA0 0 //endereço de memória EEPROM onde ficará o fator de correção de corrente aritmético
#define EMA1 1 //endereço de memória EEPROM onde ficará o fator de correção de corrente geométrico
#define EMA2 2 //endereço de memória EEPROM onde ficará a informação se o fator aritmético é ou não negativo
#define EMA3 3 //endereço de memória EEPROM onde ficará a informação se o fator geométrico é ou não negativo

int ctt = 0;
int routine = 1;
float soma, soma2;
int correcao, correcao2;
int negativo, negativo2;
float a1, a2, a3, a4, a5;
float tensao = 0.00, t2 = 0.0, corrente = 0.00, comando = 0.00, potreal, potteor;
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
//  Serial.println("Rodando...");

  //resgatamos os fatores de correção da EEPROM emulada (flash):
  negativo = EEPROM.read(EMA2);
  negativo2 = EEPROM.read(EMA3);
  if (negativo) {correcao = -EEPROM.read(EMA0);}
  else {correcao = EEPROM.read(EMA0);}
  if (negativo2) {correcao2 = -EEPROM.read(EMA1);}
  else {correcao2 = EEPROM.read(EMA1);}
}

void potato(){
  //Essa função foi criada de forma a impedir que um clique no botão avance várias rotinas, pois o attach interrupt é muito rápido
  ctt = routine;
}

void loop() {
// Rotina 1
    while (ctt == 1){
      soma = 0;
      //Serial.println("Rotina 1, aguarde...");
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
      Serial.print("Fator correção 1: ");
      Serial.print(EEPROM.read(EMA0));
      Serial.print(" -> ");
      Serial.println(correcao);
      Serial.print("Negativo: ");
      Serial.print(EEPROM.read(EMA2));
      Serial.print(" -> ");
      if (correcao < 0){
        negativo = 1;
        EEPROM.update(EMA2, negativo);
        EEPROM.update(EMA0, -correcao);
      }
      else{
        negativo = 0;
        EEPROM.update(EMA2, negativo);
        EEPROM.update(EMA0, correcao);
      }
      Serial.println(negativo);
      delay(2000);
      digitalWrite(PB13, LOW);
      routine = 2;
      ctt = 0;
    }
//Rotina 2
    if (ctt == 2){
      //Serial.println("Rotina 2, acelere até:");
      //Serial.print(correnteII);
      //Serial.println("A");
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
      //Serial.println("Rotina 3, manter em:");
      //Serial.print(correnteII);
      //Serial.println("A");
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
    Serial.print("Fator correção 2: ");
    Serial.print(EEPROM.read(EMA1));
    Serial.print(" -> ");
    Serial.println(correcao2);
    Serial.print("Negativo 2? ");
    Serial.print(EEPROM.read(EMA3));
    Serial.print(" -> ");
    if (correcao2 < 0){
        negativo2 = 1;
        EEPROM.update(EMA3, negativo2);
        EEPROM.update(EMA1, -correcao2);
      }
      else{
        negativo2 = 0;
        EEPROM.update(EMA3, negativo2);
        EEPROM.update(EMA1, correcao2);
      }
      Serial.println(negativo2);
    ESC.write(0);
    delay(5000);
    digitalWrite(PB13, LOW);
    routine = 1;
    ctt = 0;
  }
//Rotina 0 (limitador)
  while (ctt == 0){
  for (int ct2 = 0; ct2 < window; ct2++){
    t2 = t2 + (analogRead(PA0)/fcv);
    a1 = a1+analogRead(PA5);
    ESC.write(cmdESC);
  }
  tensao = t2/window;
  //Serial.println(tensao);
  corrente = ((a1/window) - correcao)/correcao2;
  //Serial.print(analogRead(PA5));
  //Serial.println("   ");
  //Serial.println(corrente);
  a1 = 0;
  t2 = 0;
  potreal = (corrente*tensao);
  comando = (pulseIn(PA7, HIGH) - 942)/10.0;
  potteor = potlimit*comando/100;
    if (potteor < trashold)cmdESC = 0;
    else cmdESC = cmdESC + (potteor - potreal)/ansdelay;
    //Serial.print(comando);
    //Serial.print("\t");
    //Serial.print(corrente);
    //Serial.print("\t");
    //Serial.print(tensao);
    //Serial.print("\t");
    //Serial.println(cmdESC);
  }
  
}
