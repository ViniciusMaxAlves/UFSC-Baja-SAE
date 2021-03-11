/*
   Março 2021 - Vinicius Maximiano Alves (Max)
   
   Coleta de dados de combustível e armazenamento no cartão SD

   Este software tem como objetivo apenas coletar dados.
   O tratamento dos dados pode ser feito posteriormente em uma planilha,
   afim de verificar qual o melhor algoritmo para ser implementado no sistema embarcado

   É interessante saber a quantidade inicial de combustível no tanque e a quantidade final
   de combustível, se possível. Dessa forma da para tentar criar algoritmos na planilha
   para ver qual se aproxima mais do esperado na identificação do nível de reserva

   O software coleta uma quantidade de amostras (NUMERO_AMOSTRAS) a cada intervalo de
   tempo definido (INTERVALO_TEMPO), e armazena no cartão SD.

   Esse intervalo serve para evitar pegar muitas amostras desnecessárias, como em momentos
   que o veículo está em uma subida. Em carros de rua é feito dessa mesma forma.
*/

#include <TimerOne.h>
#include <SdFat.h>

#define NUMERO_AMOSTRAS 240     //Quantidade de amostras coletadas após os intervalos de 2 min - 240
#define TEMPO_INTERRUPT 250000  //250 ms - 250000
#define INTERVALO_TEMPO 480     //Esse numero é a quantidade de interrupções geradas para dar o intervalo de tempo desejado (480 * 250ms = 2min) - 480
#define CS_PIN 4
#define PIN_CAPACITIVO 3

uint8_t dadoLido;
unsigned int numAmostrasColetadas;
bool verificarSensor = true;

SdFat sdCard;
SdFile arquivo;

void setup() {
  Serial.begin(9600);
  //pinMode(PIN_CAPACITIVO, INPUT);

  // Inicializa o modulo SD
  if (!sdCard.begin(CS_PIN, SPI_HALF_SPEED))sdCard.initErrorHalt();
  /* 
   * Abre o arquivo Dados_Capacitivo.TXT 
   * Essa biblioteca tem limitação no tamanho do nome do arquivo
   * A função OPEN recebe por paramentro 11 digitos, sendo que ".txt" ocupa 4
   * então sobram 7 digitos para nomear o arquivo
   * 
   * As flags do segundo parametro da função OPEN podem ser encontradas 
   * no arquivo SDBaseFile.h
   */
  if (!arquivo.open("Capacit.txt", O_RDWR | O_CREAT | O_AT_END)){
    sdCard.errorHalt("Erro na abertura do arquivo!");
  }

    /*
       Caso não seja possivel criar um novo arquivo cada vez que o arduino é ligado
       a próxima linha serve como gambiarra para saber onde inicia os novos dados
       Seria interessante um módulo real time clock para armazenar hora de data
    */
    arquivo.println("SISTEMA REINICIADO");
    arquivo.close();
    //Serial.println("SISTEMA REINICIADO");

  Timer1.initialize(TEMPO_INTERRUPT);
  Timer1.attachInterrupt(verificaTempo);
}

void loop() {

}

void verificaTempo() {
  if (!verificarSensor) {
    numAmostrasColetadas++;
    if (numAmostrasColetadas == INTERVALO_TEMPO) {
      numAmostrasColetadas = 0;
      verificarSensor = true;
    }
  } else {
    lerSinalSensor();
  }
}

void lerSinalSensor() {
  if (numAmostrasColetadas < NUMERO_AMOSTRAS) {
    escreveValorLido();
    if (numAmostrasColetadas == NUMERO_AMOSTRAS) {
      finalizaColetaAmostra();
    }
  }
}

void escreveValorLido(){
  dadoLido = digitalRead(PIN_CAPACITIVO);
  arquivo.open("Capacit.txt", O_WRITE | O_AT_END);
  arquivo.println(dadoLido);
  arquivo.close();
  //Serial.println(dadoLido);
  numAmostrasColetadas++;
}

void finalizaColetaAmostra(){
  arquivo.open("Capacit.txt", O_WRITE | O_AT_END);
  arquivo.println("NOVO INTERVALO");
  arquivo.close();
  //Serial.println("NOVO INTERVALO");
  numAmostrasColetadas = 0;
  verificarSensor = false;
}
