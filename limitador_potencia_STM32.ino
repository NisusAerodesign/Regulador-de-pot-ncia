#include "Servo.h"
#include <EEPROM.h>

#define correnteII 30.0 //corrente de regulagem
#define potlimit 660.0 //limite de potência
//#define fcv 168.61 //fator de correção da tensão (limitador 1)
#define fcv 179.8 //fator de correção da tensão (limitador 2)
#define ansdelay 550 //coeficiente de atraso na resposta do limitador (muito alto o torna difícil de pilotar, muito baixo torna o limitador instável)
#define threshold 5.0 //threshold de ativação do limitador (em watts)
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
  pinMode(PA0, INPUT_ANALOG); //leitura de tensão
  pinMode(PA5, INPUT_ANALOG); //leitura de corrente
  pinMode(PA7, INPUT); //leitura do comando de aceleração por rádio
  pinMode(PB13, OUTPUT); //saída para o LED
  attachInterrupt(digitalPinToInterrupt(PA4), potato, RISING); //botão rotina
  digitalWrite(PB13, LOW);
  Serial.begin(115200);
  ESC.attach(PA6,942,1942); //saida para o esc


  negativo = EEPROM.read(EMA2); //le da EEPROM se o fator de correção aritmético é negativo
  negativo2 = EEPROM.read(EMA3); //le da EEPROM se o fator de correção geométrico é negativo
  if (negativo) { //se o fator aritmético é negativo
    correcao = -EEPROM.read(EMA0); //le e inverte o sinal do fator de correção aritmético
  }
  else {
    correcao = EEPROM.read(EMA0); //le o fator de correção aritmético normalmente
  }
  if (negativo2) { //se o fator geométrico é negativo
    correcao2 = -EEPROM.read(EMA1); //le e inverte o sinal do fator de correção geométrico
  }
  else {
    correcao2 = EEPROM.read(EMA1); //le o fator de correção geométrico normalmente
  }
}

void potato(){ //função para impedir que um clique no botão avance várias rotinas, pois o attach interrupt é muito rápido
  ctt = routine;
}

void loop() {
    while (ctt == 1){ //rotina 1 (calibra o offset do sensor de corrente)
      soma = 0;
      digitalWrite(PB13, HIGH);

      for (int ct2 = 0; ct2 < samples; ct2 ++){ //le sample vezes e faz as médias das leituras e adiciona em soma
        ESC.write(0);
        a5 = analogRead(PA5);
        a4 = analogRead(PA5);
        a3 = analogRead(PA5);
        a2 = analogRead(PA5);
        a1 = analogRead(PA5);
        corrente = (a1+a2+a3+a4+a5)/5.0;
        soma = soma + (corrente);
      }

      correcao = soma/samples; //calcula o offset médio

      Serial.print("Fator correção 1: "); //imprime os valores da correção
      Serial.print(EEPROM.read(EMA0));
      Serial.print(" -> ");
      Serial.println(correcao);
      Serial.print("Negativo: ");
      Serial.print(EEPROM.read(EMA2));
      Serial.print(" -> ");

      if (correcao < 0){ //armazena o valor de correção diferentemente dependendo se ele é positivo ou negativo
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

    if (ctt == 2){ //rotina 2 (acelera até a velocidade alvo)
      digitalWrite(PB13, HIGH);
      delay(1000);
      routine = 3;

      while(ctt == 2){
        cmdESC = (pulseIn(PA7, HIGH) - 942)*0.18;
        ESC.write(cmdESC);
      }

      digitalWrite(PB13, LOW);
    }

    while (ctt == 3){ //rotina 3 (calibração do ganho proporcional da corrente)
      soma2 = 0;
      digitalWrite(PB13, HIGH);

      for (int ct2 = 0; ct2 < samples; ct2 ++){ //le sample vezes e faz as médias das leituras e adiciona em soma2
        a5 = analogRead(PA5);
        a4 = analogRead(PA5);
        a3 = analogRead(PA5);
        a2 = analogRead(PA5);
        a1 = analogRead(PA5);
        corrente = (a1+a2+a3+a4+a5)/5.0;
        soma2 = soma2 + ((corrente-correcao)/correnteII);
      }

    correcao2 = soma2/samples; //calcula o fator de correção médio

    Serial.print("Fator correção 2: "); //imprime os valores da correção
    Serial.print(EEPROM.read(EMA1));
    Serial.print(" -> ");
    Serial.println(correcao2);
    Serial.print("Negativo 2? ");
    Serial.print(EEPROM.read(EMA3));
    Serial.print(" -> ");

    if (correcao2 < 0){ //armazena o valor de correção diferentemente dependendo se ele é positivo ou negativo
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

  while (ctt == 0){ //rotina 0 (limitador)
  for (int ct2 = 0; ct2 < window; ct2++){ //roda window vezes e soma os resultados da corrente e tensao em t2 e a1
    t2 = t2 + (analogRead(PA0)/fcv); //tensao
    a1 = a1+analogRead(PA5); //corrente
    ESC.write(cmdESC);
  }

  tensao = t2/window; //calcula tensao media
  corrente = ((a1/window) - correcao)/correcao2; //calcula corrente media

  a1 = 0; //zera os somatórios
  t2 = 0;

  potreal = (corrente*tensao); //calcula a potencia real
  comando = (pulseIn(PA7, HIGH) - 942)/10.0; //le o comando do radio
  potteor = potlimit*comando/100; //calcula a potencia teorica de acordo com o comando

  Serial.print(potreal); //Imprime os valores (para calibracao)
  Serial.print(" | ");
  Serial.print(potteor);
  Serial.print(" | ");
  Serial.print(corrente);
  Serial.print(" | ");
  Serial.print(tensao);
  Serial.print(" | ");
  Serial.print(comando);

  if (potteor < threshold){ //se a potencia for menor que um threshold definido, o motor nao liga
    cmdESC = 0;
  }
  else{
    cmdESC = cmdESC + (potteor - potreal)/ansdelay; //escreve a potencia para o esc
  } 
  }
}
