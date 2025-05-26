#include "Servo.h"
#include <EEPROM.h>

#include <arm_math.h> //PID (tem que baixar, faz parte da biblioteca CMSIS_DSP)
#include <cmsis_compiler.h> //Provavelmente vai dar erro dizendo que falta essas bibliotecas
#include <cmsis_gcc.h> //Baixar elas do github e incluir (documentos/arduino/libraries/Arduino_CMSIS_DSP/src/tacar aqui dentro)

//https://arm-software.github.io/CMSIS-DSP/main/group__PID.html
//Documentação dos comandos PID

#define correnteII 30.0 //Corrente de regulagem
#define potlimit 660.0 //Limite de potência
//#define fcv 168.61 //Fator de correção da tensão (limitador 1)
#define fcv 179.8 //Fator de correção da tensão (limitador 2)
#define ansdelay 550 //Coeficiente de atraso na resposta do limitador (mantido do codigo antigo, mas não usado com comando PID)
#define threshold 5.0 //Threshold de ativação do limitador (em watts)
#define samples 50000 //Quantidade de amostras para calibragem
#define window 30 //Janela de amostras para média de corrente
#define EMA0 0 //Endereço de memória EEPROM onde ficará o fator de correção de corrente aritmético
#define EMA1 1 //Endereço de memória EEPROM onde ficará o fator de correção de corrente geométrico
#define EMA2 2 //Endereço de memória EEPROM onde ficará a informação se o fator aritmético é ou não negativo
#define EMA3 3 //Endereço de memória EEPROM onde ficará a informação se o fator geométrico é ou não negativo

int ctt = 0;
int routine = 1;
float soma, soma2;
int correcao, correcao2;
int negativo, negativo2;
float a1, a2, a3, a4, a5;
float tensao = 0.00, t2 = 0.0, corrente = 0.00, comando = 0.00, potreal, potteor;
float cmdESC = 0;

Servo ESC;

arm_pid_instance_f32 PID;

void setup(){
  pinMode(PA0, INPUT_ANALOG); //Leitura de tensão
  pinMode(PA5, INPUT_ANALOG); //Leitura de corrente
  pinMode(PA7, INPUT); //Leitura do comando de aceleração por rádio
  pinMode(PB13, OUTPUT); //Saída para o LED da placa (não o do stm)
  attachInterrupt(digitalPinToInterrupt(PA4), potato, RISING); //Botão para as rotinas
  digitalWrite(PB13, LOW);
  Serial.begin(115200);
  ESC.attach(PA6,942,1942); //Saída para o ESC (motor)

  negativo = EEPROM.read(EMA2);
  negativo2 = EEPROM.read(EMA3);
  correcao = negativo ? -EEPROM.read(EMA0) : EEPROM.read(EMA0);
  correcao2 = negativo2 ? -EEPROM.read(EMA1) : EEPROM.read(EMA1);
  //resgatamos os fatores de correção da EEPROM

  //Configuração inicial do PID (o sistema calcula o erro a partir desses valores)
  PID.Kp = 0.8f; //Proporcional (resposta rápida ao erro)
  PID.Ki = 0.2f; //Integral (corrige os erros acumulados ao longo do tempo)
  PID.Kd = 0.05f; //Derivativo (prevê a tendencia e ajuda a estabilizar a resposta)
  arm_pid_init_f32(&PID, 1);

  //COMO CALIBRAR O PID!!!
  //Por Ziegler-Nichols -> Para calibrar começe com Kp baixo (ex:0.1f), e Ki/Kd = 0.0f
  //Aumentar o Kp aos poucos até começar a oscilar continuamente (deve ser oscilação sustentada, sem crescer ou morrer) e anotar o valor do Kp nesse momento, esse é o Kp critico (Kc), depois medir o tempo para um ciclo de oscilação completo, esse é o periodo critico (Pc)
  //Aplicar as formulas (nessa ordem): Kp = 0,6*Kc | Ti = Pc/2 | Td = Pc/8 | Ki = Kp/Ti | Kd = Kp*Td
  //Ti e Td não são usados diretamente no código, só são usados para converter, pq o arm_pid_f32 usa ganhos diretos
  //Atualizar Kp, Ki e Kd no código, depois testar e refinar: se for muito oscilante -> aumente Kd e/ou reduza Kp | se houver ouvershoot -> diminua Ki | se demorar para estabilizar -> aumente Kp um poucos | Se houver erro constante -> aumente Ki | Windup (acumulo excessivo de integral) -> atuador pode estar saturado, limitar integral (anti-windup) | Ruido amplificado -> Reduzir Kd | (existem outros casos mas esses são os principais)
  //Se ainda estiver muito instavel, imprimir o erro, saida pid (pid_output), potreal e potteor podem pode ajudar a visualizar como o sistema reage
  //Ou só coloca valores genéricos e faz por tentativa e erro ;)
}

void potato(){ //Função para impedir que um clique do botão avance varias rotinas, pois o attach e o interrupt é muito rápido
  ctt = routine;
}

void loop() {
  while (ctt == 1){ //Rotina 1 (calibra o offset do sensor de corrente (nada a ver com o offset do PID), "calibra" o sensor)
    soma = 0;
    digitalWrite(PB13, HIGH);
    for (int ct2 = 0; ct2 < samples; ct2 ++){
      ESC.write(0);
      a1 = analogRead(PA5);
      a2 = analogRead(PA5);
      a3 = analogRead(PA5);
      a4 = analogRead(PA5);
      a5 = analogRead(PA5);
      corrente = (a1+a2+a3+a4+a5)/5.0;
      soma += corrente;
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
    } else {
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

  if (ctt == 2){ //Rotina 2 (permite que acelere até a corrente alvo)
    digitalWrite(PB13, HIGH);
    delay(1000);
    routine = 3;
    while(ctt == 2){
      cmdESC = (pulseIn(PA7, HIGH) - 942)*0.18;
      ESC.write(cmdESC);
    }
    digitalWrite(PB13, LOW);
  }

  while (ctt == 3){ //Rotina 3 (calibração do ganho proporcional da corrente)
    soma2 = 0;
    digitalWrite(PB13, HIGH);
    for (int ct2 = 0; ct2 < samples; ct2 ++){
      a1 = analogRead(PA5);
      a2 = analogRead(PA5);
      a3 = analogRead(PA5);
      a4 = analogRead(PA5);
      a5 = analogRead(PA5);
      corrente = (a1+a2+a3+a4+a5)/5.0;
      soma2 += ((corrente - correcao) / correnteII);
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
    } else {
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

  while (ctt == 0){ //Rotina 0 (Limitador/PID)
    for (int ct2 = 0; ct2 < window; ct2++){
      t2 += (analogRead(PA0) / fcv);
      a1 += analogRead(PA5);
      ESC.write(cmdESC);
    }
    tensao = t2/window;
    corrente = ((a1/window) - correcao)/correcao2;
    t2 = 0;
    a1 = 0;
    potreal = (corrente * tensao);
    comando = (pulseIn(PA7, HIGH) - 942)/10.0;
    potteor = potlimit * comando / 100; 
    //O programa lê a corrente, tensão e o comando por rádio para calcular a potencia real e teorica a partir disso

    float pid_input = potteor;
    float pid_output = arm_pid_f32(&PID, pid_input - potreal); //A partir disso o programa calcula o erro entre a pot teorica e a pot real medida (pelo metodo PID)

    if (potteor < threshold) { //Se a potencia for menor que o threshold de ativação, o limitador não liga.
      cmdESC = 0;
    } else {
      cmdESC += pid_output;
      cmdESC = constrain(cmdESC, 0, 100); //Evita valores fora do limite do ESC
    }
  }
}
