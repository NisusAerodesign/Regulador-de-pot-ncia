#include "Servo.h"
#include <EEPROM.h>

// ===== Constantes de reguláveis ===== // Ajustar conforme necessário
#define potlimit 660.0 // Limite de potência, em watts
#define iref 30.0 // Corrente de referência para calibração usada na rotina 3, em amepres
#define fcv 168.61 // Fator multiplicativo de correção da tensão (o valor bruto da leitura é dividido por esse valor)
#define ansdelay 550 // Coeficiente de atraso na resposta do limitador (muito alto o torna difícil de pilotar, muito baixo torna o limitador instável)
#define threshold 5.0 // Threshold de ativação do limitador (em watts) - serve para que o limitador mande sinal zero quando estiver próximo o suficiente de zero
#define samples 50000 // Quantidade de amostras de leitura de tensão e corrente para calibragem
#define window 30 // Janela de amostras para média de corrente na rotina zero (rotina do limitador)
#define radioLow 942// Limite inferior do rádio, medido em us, deve ser ajustado conforme o mínimo do rádio
#define radioHigh 1942// Limite superior do rádio, medido em us, deve ser ajustado conforme o máximo do rádio

// ===== Definições da EEPROM ===== // Não é necessário mexer
#define EMA0 0 //endereço de memória EEPROM onde ficará o fator de correção de corrente aritmético
#define EMA1 1 //endereço de memória EEPROM onde ficará o fator de correção de corrente geométrico
#define EMA2 2 //endereço de memória EEPROM onde ficará a informação se o fator aritmético é ou não negativo
#define EMA3 3 //endereço de memória EEPROM onde ficará a informação se o fator geométrico é ou não negativo

// ===== Variáveis de controle de rotina ===== //
uint8_t ctt = 0;
uint8_t routine = 1;

// ===== Variáveis de leitura e correção do limitador ===== //
float soma, soma2;
int correcao, correcao2;
uint8_t negativo, negativo2;
float a1, a2, a3, a4, a5;
float tensao = 0.00, t2 = 0.0, corrente = 0.00, comando = 0.00, potreal, potteor;
float cmdESC = 0;

// ===== Objeto servo ===== //
Servo ESC; // Nota: usamos a biblioteca servo.h para o controle do ESC, pois ele recebe o mesmo tipo de comando (duty-cycle 2-20 ms)

void basicSetup() {
	pinMode(PA0, INPUT_ANALOG); // Leitura de tensão
	pinMode(PA5, INPUT_ANALOG); // Leitura de corrente
	pinMode(PA7, INPUT); // Leitura do comando de aceleração por rádio
	pinMode(PB13, OUTPUT); // Saída para o LED
	attachInterrupt(digitalPinToInterrupt(PA4), avancaRotina, RISING); // Botão rotina
	digitalWrite(PB13, LOW);
	Serial.begin(115200);
	ESC.attach(PA6,radioLow,radioHigh); // Range de saída para o ESC
}

void leituraEEPROM() {
	negativo = EEPROM.read(EMA2); //le da EEPROM se o fator de correção aritmético é negativo
	negativo2 = EEPROM.read(EMA3); //le da EEPROM se o fator de correção geométrico é negativo

	if (negativo) correcao = -EEPROM.read(EMA0); // se o fator aritmético é negativo, le e inverte o sinal do fator de correção aritmético
	else correcao = EEPROM.read(EMA0); // caso contrário le o fator de correção aritmético normalmente

	if (negativo2) correcao2 = -EEPROM.read(EMA1); //se o fator geométrico é negativo, le e inverte o sinal do fator de correção geométrico
	else correcao2 = EEPROM.read(EMA1); //le o fator de correção geométrico normalmente
}

void setup() {
	basicSetup();
	leituraEEPROM();
}

void avancaRotina(){ // Função para impedir que um clique no botão avance várias rotinas, pois o attach interrupt é muito rápido
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
				cmdESC = (pulseIn(PA7, HIGH) - radioLow)*0.18;
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
				soma2 = soma2 + ((corrente-correcao)/iref);
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
	comando = (pulseIn(PA7, HIGH) - radioLow)/10.0; //le o comando do radio
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
