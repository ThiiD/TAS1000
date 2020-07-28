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


void inicializa() {
  
  //Inicializando as portas
  pinMode(PINO_BOTAO, INPUT);
  pinMode(PINO_BUZZER, OUTPUT);
  pinMode(PINO_LED_VERD, OUTPUT);
  pinMode(PINO_LED_VERM, OUTPUT);
  pinMode(PINO_LED_AZUL, OUTPUT);
//  ledcSetup(0, 1E5, 12);
//  ledcAttachPin(PINO_BUZZER, 0);


  LoRa.setPins(ss, rst, dio0);

  //iniciando o servo
  pinMode(REC1_PRINCIPAL, OUTPUT);
  digitalWrite(REC1_PRINCIPAL, LOW);
  pinMode(REC1_SECUNDARIO, OUTPUT);
  digitalWrite(REC1_SECUNDARIO, LOW);
  pinMode(REC_SOLENOIDE, OUTPUT);
  digitalWrite(REC_SOLENOIDE, LOW);
  erro = 0;

  //Inicializando o Altímetro
  if (!bmp.begin(0x76)) {
    erro = ERRO_BMP;
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  //Inicializando o GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  alturaInicial_GPS = gps.altitude.meters();
  alturaInicial =  bmp.readAltitude(PRESSAO_MAR);
  alturaMinima = alturaInicial;

  //iniciar o IMU



  //inicializar LoRa
  if (!LoRa.begin(915E6))
  {
    erro = ERRO_LORA;

    return;
  }




  LoRa.setSyncWord(0xF3);
  if (!erro) {
#ifdef DEBUG
    Serial.println("Nenhum erro iniciando dispositivos, começando o loop do main");
#endif
    statusAtual = ESTADO_ESPERA;
  }

  else {
#ifdef DEBUG
    Serial.print("Altímetro com erro de inicialização código:");
    Serial.println(erro);
#endif
    statusAtual = erro;

    atualizaMillis = millis();
  }


}
void leBotoes() {

  millisAtual = millis();
  estado = digitalRead(PINO_BOTAO);

  //Liga a gravação se em espera
  if (estado && (statusAtual == ESTADO_ESPERA)) {
    statusAtual = ESTADO_GRAVANDO;


  }
}
void adquireDados() {

  //todas as medidas são feitas aqui em sequeência de maneira que os valores
  //sejam temporalmente próximos
  pressaoAtual = bmp.readPressure();
  alturaAtual = bmp.readAltitude(PRESSAO_MAR);
  temperaturaAtual = bmp.readTemperature();

  while (SerialGPS.available() > 0) {
    gps.encode(SerialGPS.read());
  }
  latitude =  gps.location.lat();
  longitude =  gps.location.lng();
  alturaAtualGPS = gps.altitude.meters();

}

void checaCondicoes() {

  if ((statusAtual == ESTADO_GRAVANDO) && !gravando ) {
    alturaMinima = alturaAtual;
    gravando = true;
  }

  if (!alturaInicial_GPS && alturaAtualGPS != 0)
    alturaInicial_GPS = alturaAtualGPS;

  //alturaMinima BMP
  if ((alturaAtual < alturaMinima) && (statusAtual == ESTADO_GRAVANDO))
    alturaMinima = alturaAtual;

  //alturaMaxima BMP
  if (!subiu && (statusAtual == ESTADO_GRAVANDO))
    alturaMaxima = 0;

  //controle de subida BMP
  if ((alturaAtual > alturaMinima + THRESHOLD_SUBIDA) && (statusAtual == ESTADO_GRAVANDO) && !subiu )
    subiu = true;

  //primeira referencia de altura maxima
  if (subiu && (alturaMaxima == 0) && (statusAtual == ESTADO_GRAVANDO))
    alturaMaxima = alturaAtual;

  //verificar a altura máxima BMP
  if ((alturaAtual > alturaMaxima) && (statusAtual == ESTADO_GRAVANDO) && subiu)
    alturaMaxima =  alturaAtual;


  if ((alturaAtualGPS > alturaMaximaGPS) && (statusAtual == ESTADO_GRAVANDO) && subiu)
    alturaMaximaGPS = alturaAtualGPS;

  //Controle de descida do primeiro estágio, usando um threshold para evitar disparos não
  //intencionais
  if ((alturaAtual + THRESHOLD_DESCIDA < alturaMaxima) && (statusAtual == ESTADO_GRAVANDO) && subiu) {
    descendo = true;
    subiu = false;
    statusAtual = ESTADO_RECUPERANDO;
  }

  //Controle de descida do segundo estágio de recuperação
  if ((alturaAtual - alturaInicial < THRESHOLD_SOLENOIDE) && descendo)
  {
    descendo_Solenoide = true;

    if (alturaInicial_GPS != 0 )
      if ((alturaAtualGPS - alturaInicial_GPS <  THRESHOLD_SOLENOIDE_GPS ) && descendo  && (alturaAtualGPS - alturaInicial_GPS > 0))
        descendo_Solenoide = true;
  }

}

void finaliza() {
}

void recupera () {

  //verifica aqui se o foguete já atingiu o apogeu e se está descendo pelas
  //suas variáveis globais de controle e chama a função que faz o acionamento
  //do paraquedas
  if (descendo && !abriuParaquedas) {

    abreParaquedas();

  }

  if (descendo_Solenoide && !abriuSolenoide)
  {
    abreSolenoide();
  }



}

void notifica (char codigo) {

  unsigned int frequencia[10];
#ifdef DEBUG
  Serial.print("Status atual do altímetro:");
  Serial.println(codigo);
#endif

  switch (codigo) {
    case ERRO_BMP:

      frequencia[0] = 261;
      frequencia[1] = 261;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 220;
      frequencia[5] = 220;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 196;
      frequencia[9] = 196;
      if (millisAtual - millisLed > 100) {
        digitalWrite(PINO_LED_AZUL, !digitalRead(PINO_LED_AZUL));
        digitalWrite(PINO_LED_VERD, LOW);
        digitalWrite(PINO_LED_VERM, LOW);
        millisLed = millisAtual;
      }

      break;
    case ERRO_LORA:
      frequencia[0] = 261;
      frequencia[1] = 261;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 220;
      frequencia[5] = 220;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 196;
      frequencia[9] = 196;
      if (millisAtual - millisLed > 100) {
        digitalWrite(PINO_LED_VERM, !digitalRead(PINO_LED_VERM));
        digitalWrite(PINO_LED_AZUL, LOW);
        digitalWrite(PINO_LED_VERD, LOW);
        millisLed = millisAtual;
      }
      break;
    case ESTADO_RECUPERANDO:

      frequencia[0] = 4000;
      frequencia[1] = 4500;
      frequencia[2] = 4000;
      frequencia[3] = 0;
      frequencia[4] = 0;
      frequencia[5] = 0;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 0;
      frequencia[9] = 0;
      if (millisAtual - millisLed > 100) {
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
        digitalWrite(PINO_LED_AZUL, LOW);
        digitalWrite(PINO_LED_VERM, LOW);
        millisLed = millisAtual;
      }

      break;

    //Gravando, pisca um led vermelho como uma câmera e também faz
    //um tom simples.
    case ESTADO_GRAVANDO:

      frequencia[0] = 293;
      frequencia[1] = 293;
      frequencia[2] = 0;
      frequencia[3] = 0;
      frequencia[4] = 0;
      frequencia[5] = 0;
      frequencia[6] = 0;
      frequencia[7] = 0;
      frequencia[8] = 0;
      frequencia[9] = 0;
      if (millisAtual - millisLed > 100) {
        digitalWrite(PINO_LED_AZUL, !digitalRead(PINO_LED_AZUL));
        digitalWrite(PINO_LED_VERD, LOW);
        digitalWrite(PINO_LED_VERM, LOW);
        millisLed = millisAtual;
      }

      break;
    case ESTADO_ESPERA:
      //led verde piscando devagar indicando espera
      if (millisAtual - millisLed > 500) {
        digitalWrite(PINO_LED_VERD, !digitalRead(PINO_LED_VERD));
        millisLed = millisAtual;
      }


      break;

  }
//    if (codigo) {
//      if (frequencia[o] && (statusAtual != ESTADO_ESPERA)) {
//       // tone(PINO_BUZZER, frequencia[o], TEMPO_ATUALIZACAO);
//      }
//      o++;
//      if (o > 9)
//        o = 0;
//  }

}

void abreParaquedas() {
#ifdef DEBUG
  Serial.println("Abrindo o paraquedas!");
#endif
  digitalWrite(REC1_PRINCIPAL, HIGH);
  millisRec = millis();
  abriuParaquedas = 1;

  LoRa.beginPacket();
  LoRa.print("K");
  LoRa.print(testeParaquedas);
  LoRa.endPacket();

}
void abreSolenoide() {
#ifdef DEBUG
  Serial.println("Abrindo solenoide!");
#endif
  digitalWrite(REC_SOLENOIDE, HIGH);
  abriuSolenoide = 1;

  LoRa.beginPacket();
  LoRa.print("S");
  LoRa.print(testeParaquedas);
  LoRa.endPacket();
}
void envia() {
  if ((statusAtual == ESTADO_GRAVANDO) && !gravando ){
  
  latitudeLora = (long) (latitude * 1000000);
  longitudeLora = (long) (longitude * 1000000);
  alturaGpsLora = (long) (alturaAtualGPS * 1000000);
  alturaBMPLora = (long) (alturaAtual * 1000000);
  millisAtualLora = (long) (millisAtual);

    
  if (latitudeLora != 0) {
    LoRa.beginPacket();
    LoRa.print("L");
    LoRa.print(latitudeLora);
    LoRa.endPacket();
  }
  if (longitudeLora != 0) {
    LoRa.beginPacket();
    LoRa.print("M");
    LoRa.print(longitudeLora);
    LoRa.endPacket();
  }
  if (alturaGpsLora != 0) {
    LoRa.beginPacket();
    LoRa.print("A");
    LoRa.print(alturaGpsLora);
    LoRa.endPacket();
  }
  LoRa.beginPacket();
  LoRa.print("H");
  LoRa.print(alturaBMPLora);
  LoRa.endPacket();

  LoRa.beginPacket();
  LoRa.print("T");
  LoRa.print(millisAtualLora);
  LoRa.endPacket();

}
}

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif

#ifdef DEBUG_TEMP
  Serial.begin(115200);

#endif
  //Faz o setup inicial dos sensores de movimento e altura assim
  //como as portas

#ifdef DEBUG
  Serial.println("Iniciando o altímetro");
#endif

  inicializa();

}

void loop() {

  //Recebendo o tempo atual de maneira a ter uma base de tempo
  //para uma taxa de atualização
  millisAtual = millis();

  if ((millis() - millisRec >= TEMPO_RELE) && abriuParaquedas) {
    digitalWrite(REC1_PRINCIPAL, LOW); //COMENTAR LINHA CASO NÃO FOR NECESSÁRIO
    digitalWrite(REC1_SECUNDARIO, HIGH);
  }



  if ((millisAtual - atualizaMillis) >= TEMPO_ATUALIZACAO) {
#ifdef DEBUG_TEMP
    Serial.print("Status atual:");
    Serial.println(statusAtual);
    Serial.print("estado atual de erro:");
    Serial.println(erro);
#endif
    //verifica se existem erros e mantém tentando inicializar
    if (erro) {
      inicializa();
      notifica(erro);
    }

    //Se não existem erros no sistema relacionados a inicialização
    //dos dispositivos, fazer:

    if (!erro) {

#ifdef DEBUG
      Serial.println("Rodando o loop de funções");
#endif

      //Verifica os botões e trata o clique simples e o clique longo
      //como controle de início/fim da gravação.
      leBotoes();

#ifdef DEBUG
      Serial.println("Li os botões");
#endif

      //Recebe os dados dos sensores e os deixa salvo em variáveis
      adquireDados();
#ifdef DEBUG
      Serial.println("Adquiri os dados");
#endif

      //Trata os dados, fazendo filtragens e ajustes.
#ifdef DEBUG
      Serial.println("Tratei os dados");
#endif

      //Envia dados para o receptor no chão
      envia();
#ifdef DEBUG
      Serial.println("Enviei os Dados");
#endif


      //De acordo com os dados recebidos, verifica condições como a
      //altura máxima atingida e seta variáveis de controle de modo
      //que ações consequintes sejam tomadas.
      checaCondicoes();

      //Faz ajustes finais necessários
      finaliza();

      //Caso o voo tenha chegado ao ápice, libera o sistema de recuperação
      recupera();
    }

    //Notifica via LEDs e buzzer problemas com o foguete
    notifica(statusAtual);

    atualizaMillis = millisAtual;
  }





}

