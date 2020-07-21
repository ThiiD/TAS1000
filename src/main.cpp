#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Wire.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP280.h>

//Definições de debug
#define DEBUG
#define DEBUG_TEMP

//Definições de sensores
#define USANDO_BMP280

//Definições default
#define PRESSAO_MAR 1013.25
#define EIXO_X 0
#define EIXO_Y 1
#define EIXO_Z 2
#define TAMANHO_MEDIA 10
#define SERVO_ABERTO 40
#define SERVO_FECHADO 0
#define TEMPO_RELE 1000
#define TEMPO_ATUALIZACAO 50 //em milisegundos
#define THRESHOLD_DESCIDA 10  //em metros
#define THRESHOLD_SUBIDA 10  //em metros
#define THRESHOLD_SOLENOIDE 450
#define THRESHOLD_SOLENOIDE_GPS 350

//Definições de input
#define PINO_BOTAO 27
#define PINO_BUZZER 35
#define PINO_LED_VERD 33
#define PINO_LED_VERM 32
#define PINO_LED_AZUL 25
#define REC_SOLENOIDE 26
#define REC1_PRINCIPAL 4
#define REC1_SECUNDARIO 12
#define ss 5
#define rst 34
#define dio0 2

//definições de erros
#define ERRO_BMP 'b'
#define ERRO_LORA 'l'

//definição de estados
#define ESTADO_GRAVANDO 'g'
#define ESTADO_FINALIZADO 'f'
#define ESTADO_RECUPERANDO 'r'
#define ESTADO_ESPERA 'e'

//Variáveis de bibliotecas
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);
Adafruit_BMP280 bmp;

//Variáveis de timing
unsigned long millisAtual   = 0;
unsigned long millisUltimo  = 0;
unsigned long atualizaMillis = 0;
unsigned long millisLed   = 0;
unsigned long millisBuzzer  = 0;
unsigned long millisBotao   = 0;
unsigned long millisGravacao  = 0;
unsigned long millisRec = 1000000;

int n = 0;
int m = 0;
int o =  0;
int IMU = 1;

//Variáveis de dados
//int32_t pressaoAtual;
float alturaAtual;
float alturaInicial;
float alturaMaxima =  0;
float mediaAltura = 0;
float alturaMinima;
float alturaInicial_GPS;
float mediaPressao = 0;
float pressaoAtual;
float temperatura;
float mediaTemperatura;
float temperaturaAtual;
float latitude, longitude, alturaAtualGPS, alturaMaximaGPS, alturaGPS2, alturaBMP2;
long latitudeLora, longitudeLora, alturaGpsLora, alturaBMPLora, millisAtualLora, testeParaquedas = 10000000;

//variáveis de controle
bool iniciar = false;
bool inicializado = false;
bool terminou = false;
bool rodando = false;
bool gravando = false;
bool apogeu = false;
bool abriuParaquedas = false;
bool abriuSolenoide = false;
char erro = false;
char statusAtual;
bool estado;
bool descendo = false;
bool descendo_Solenoide = false;
bool salvarInicial = false;
bool subiu = false;

//Arrays de som de erro;





void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
}

