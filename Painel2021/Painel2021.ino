/*
 * Código do painel - display crystalfontz
 * Dezembro 2020
 * Vinicius Maximiano Alves (Max)
 * 
 * ------------------------ BUG no TimerOne --------------------------------------
 * A versão da biblioteca TimerOne utilizada é a mais recente (baixada do gitHub)
 * Existe um bug na biblioteca TimerOne (quando da start gera 2 interrupções)
 * 
 * Para resolver o bug na versão mais atual da biblioteca é necessário modificar o código do arquivo TimerOne.h
 * SOLUÇÂO: Comentar/apagar a linha 222 da função start() "TCNT1 = 0;"
 * 
 * A versão mais antiga disponivel no site do Arduino é ainda pior. 
 * Além do bug, ela desativa as interrupções quando da start(), fazendo com que seja
 * necessário sempre configurar a interrupção novamente 
 * SOLUÇÂO: iniciar valtaAtual com -1 e configurar interrupção no calculaTempo()
 * 
 * --------------------------- Funções -------------------------------
 * 
 * void somaTempo()                            //Função chamada na interrupção do Timer1
 * void mostraDadosRecebidos()
 * void configAreaRot()
 * void configAreaVelocidade()
 * void configAreaNumVoltas()
 * void configAreaTempo()
 * void verificaBotaoTempo()
 * void verificaBotaoTrocaTela()
 * void reiniciaTempo()
 * void calculaTempo()
 * void verificaSinalDiferencaVolta()           //Imprime o sinal de positivo ou negativo da diferença dos tempos de volta
 * void transformaEmPositivo()                  //Transforma um numero negativo em complemento de dois em um numero positivo
 * void somaNumVoltas()
 * void printNumVoltas()
 * void printTempo(uint8_t posicaoY, int tempo)
 * void printVelocidade()
 * void printRotacao()
 */
 
#include <openGLCD.h>
#include <TimerOne.h>

#define MAXVAL_ROTACAO 33         //Valor máximo de rotação
#define BUFFER_SIZE 2             //Tamanho do buffer que recebe os dados da UNP
#define BOTAO_TEMPO 2             //Pin botão do cronometro do tempo de volta e contagem de voltas
#define BOTAO_TROCA_TELA 3        //Pin botão de mudança de tela do display
#define LATCH_PIN 12              //Pin connected to ST_CP of 74HC595
#define DATA_PIN 13               //Pin connected to DS of 74HC595
#define CLOCK_PIN 14              //Pin connected to SH_CP of 74HC595

uint8_t led_marcha[4];
uint8_t led_ajustavel1[2];
uint8_t led_ajustavel2[2];
uint8_t led_ajustavel3[2];
uint8_t led_ajustavel4[2];
byte dadosConcatenados = 0;       //Armazena a soma bit a bit das posições selecionadas dos vetores

/*
 * TRUE: mostra rotação
 * FALSE: mostra tempos de volta
 */
boolean trocaTela = true;         //Variavel que indica se é para mostrar o tempo de volta ou a rotação
uint8_t numVoltas = 0;            //Contagem do numero de voltas
uint8_t dadosPainel[BUFFER_SIZE]; //Armazena os dados recebidos da UNP
byte numBytes = 0;                //Numero de bytes recebidos na comunicação serial - UART

/*
 * ------- Variaveis para calculo do tempo de volta -------
 * Armazena os valores em segundos para efetuar os calculos
 */
volatile int voltaAtual = 0;
int ultimaVolta = 0;
int voltaMaisRapida = 0;
int diferencaTempo = 0;

gText textAreaHalfLeft(textAreaLEFT, Callibri15);
gText textAreaVelocidade(textAreaTOPRIGHT, lcdnums14x24); 
gText textAreaNumVoltas(textAreaBOTTOMRIGHT, lcdnums12x16);
	
void setup()
{
  Serial.begin(115200);
  pinMode(BOTAO_TROCA_TELA, INPUT_PULLUP);
  pinMode(BOTAO_TEMPO, INPUT_PULLUP);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  for(int i = 0; i < BUFFER_SIZE; i++){
    dadosPainel[i] = 0;
  }

  led_marcha[0] = B10000000;
  led_marcha[1] = B01000000;
  led_marcha[2] = B01100000;
  led_marcha[3] = B01110000;

  led_ajustavel1[0] = B00000000;
  led_ajustavel1[1] = B00001000;
  
  led_ajustavel2[0] = B00000000;
  led_ajustavel2[1] = B00000100;

  led_ajustavel3[0] = B00000000;
  led_ajustavel3[1] = B00000010;

  led_ajustavel4[0] = B00000000;
  led_ajustavel4[1] = B00000001;
  
  /*
   * Necessário ler o código fonte da biblioteca TimerOne para entender o que as funções fazem
   * As funções mudam de acordo com a versão da biblioteca
   */
  Timer1.initialize(1000000); //contagem de 1s
  Timer1.attachInterrupt(somaTempo);
  Timer1.stop();
  
	GLCD.Init();
	GLCD.ClearScreen(); 
	GLCD.SelectFont(Callibri10);
  configAreaRot();
  configAreaVelocidade();
  configAreaNumVoltas();
}

void loop()
{
  verificaBotaoTempo();
  verificaBotaoTrocaTela();
  
  if(Serial.available()){
    numBytes = Serial.readBytes(dadosPainel, BUFFER_SIZE);
    if(numBytes == BUFFER_SIZE) mostraDadosRecebidos();
  }
}

void somaTempo(){
  voltaAtual++;
  if(trocaTela == false) printTempo(0, voltaAtual);
}

void mostraDadosRecebidos(){
  printVelocidade();
  if(trocaTela) printRotacao();

/*
 * -------------------- Mudar quando a PCB e UNP estiverem prontos -----------------------
 * 
  dadosConcatenados = led_marcha[dadosPainel[X]] + led_ajustavel1[dadosPainel[Y]]
        + led_ajustavel2[dadosPainel[Z]] + led_ajustavel3[dadosPainel[I]] + led_ajustavel4[dadosPainel[J]];
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, dadosConcatenados);  //MODIFICAR DE ACORDO COM AS CONEXÕES DA PCB
  digitalWrite(LATCH_PIN, HIGH);
*/
}

void configAreaRot(){
  for(uint8_t y = 4; y < GLCD.Height; y+=14){
    GLCD.DrawLine(10, y, 14, y);
  }
  for(uint8_t y = 11; y < GLCD.Height; y+=14){
    GLCD.DrawLine(10, y, 12, y);
  }
  byte numRotMostrador = 4;
  for(uint8_t y = 0; y <= 4; y++){
    GLCD.CursorToXY(16, y*14-1); 
    GLCD.print(numRotMostrador);
    numRotMostrador--;
  }
  printRotacao();
}

void configAreaVelocidade(){
  GLCD.DrawRoundRect(GLCD.CenterX+1, 0, GLCD.CenterX-1, GLCD.CenterY-1, 5);
  GLCD.CursorToXY(GLCD.Right-28, 18);
  GLCD.Printf("Km/h");
  printVelocidade();
}

void configAreaNumVoltas(){
  GLCD.DrawRoundRect(GLCD.CenterX+1, GLCD.CenterY+1, GLCD.CenterX-1, GLCD.CenterY-1, 5);
  GLCD.CursorToXY(GLCD.Right-45, GLCD.Bottom-7);
  GLCD.Printf("Voltas"); 
  printNumVoltas();
}

void configAreaTempo(){
  printTempo(0, voltaAtual);
  verificaSinalDiferencaVolta();
  printTempo(1, diferencaTempo);
  printTempo(2, ultimaVolta);
  printTempo(3, voltaMaisRapida);
}

void verificaBotaoTempo(){
  if(digitalRead(BOTAO_TEMPO) == LOW){
    somaNumVoltas();
    calculaTempo();
  }
  /*
   * As proximas linhas servem para não contar mais de um click no botão (efeito bouncing)
   * Aproveitei para calcular o tempo que o botão fica pressionado
   * Caso o botão fique pressionado por mais de 2s o cronometro é pausado e os dados apagados
   */
  long tempoInicial = millis();
  while(digitalRead(BOTAO_TEMPO) == LOW);
  long tempoFinal = millis();
  if(tempoFinal-tempoInicial >= 2000) reiniciaTempo();
  delay(50);
}

void verificaBotaoTrocaTela(){
 if(digitalRead(BOTAO_TROCA_TELA) == LOW){
    textAreaHalfLeft.ClearArea();
    if(trocaTela == true){
      trocaTela = false;
      configAreaTempo();
    }else{
      trocaTela = true;
      configAreaRot();
    }
  }
  /*
   * As proximas duas linhas servem para não contar mais de um click no botão (efeito bouncing)
   */
  while(digitalRead(BOTAO_TROCA_TELA) == LOW);
  delay(50);
}

void reiniciaTempo(){
  Timer1.stop();
  voltaAtual = 0; //Não é uma seção crítica pois as interrupções foram pausadas na linha anterior
  ultimaVolta = 0;
  voltaMaisRapida = 0;
  diferencaTempo = 0;
  numVoltas = 0;
  printNumVoltas();
  if(trocaTela == false) configAreaTempo();  
}

void calculaTempo(){
  /*
   * Seção crítica: 
   * voltaAtual é alterada aqui e na interrupção, podendo gerar uma condição de corrida 
   */
  noInterrupts();
  if(ultimaVolta != 0){ //Se a ultimaVolta é igual a zero ainda não tem volta para comparar
    diferencaTempo = voltaAtual - ultimaVolta;
  }
  ultimaVolta = voltaAtual;
  voltaAtual = 0;
  interrupts();
  if(voltaMaisRapida > ultimaVolta || voltaMaisRapida == 0){ //ultimaVolta = voltaAtual
    voltaMaisRapida = ultimaVolta;
  } 

  Timer1.start();
  
  /*
   * Só printa os valores se estiver na tela dos tempos.
   * Como a voltaAtual é apenas printada no display, não é, nesse caso, uma seção crítica
   */
  if(trocaTela == false) configAreaTempo();
}

void verificaSinalDiferencaVolta(){
  textAreaHalfLeft.CursorTo(0, 1);
  if(diferencaTempo >= 0){
    textAreaHalfLeft.Printf("+"); 
  }else{
    textAreaHalfLeft.Printf("--");
    transformaEmPositivo();
  }
}

/*
 * Tranforma um numero negativo, em complemento de dois, em um numero positivo
 * Necessário para printar a diferencaTempo no display de forma correta
 */
void transformaEmPositivo(){
  diferencaTempo = ~diferencaTempo;  //Operação NOT bit a bit
  diferencaTempo++;
}

void somaNumVoltas(){
  numVoltas++;
  if(numVoltas == 100){
    numVoltas = 0;
  }
  printNumVoltas();
}

void printNumVoltas(){
  textAreaNumVoltas.CursorToXY(20, 6);
  textAreaNumVoltas.Printf("%02d", numVoltas); // mostra o numero de voltas sempre com dois digitos
}

/*
 * Parametros:
 * uint8_t posicaoY - linha em que o tempo será impresso no display
 * int tempo - tempo em segundos
 */
void printTempo(uint8_t posicaoY, int tempo){
  byte segundos = tempo % 60; //resto da divisão por 60
  byte minutos = tempo / 60;
  textAreaHalfLeft.CursorTo(1, posicaoY);
  textAreaHalfLeft.Printf("%02d : %02d", minutos, segundos); 
}

void printVelocidade(){
  textAreaVelocidade.CursorToXY(6, 3);
  textAreaVelocidade.Printf("%02d", dadosPainel[1]); // mostra a valocidade sempre com dois digitos
}

void printRotacao(){
  GLCD.DrawVBarGraph(GLCD.Left, GLCD.Bottom-3, 10, -57, 1, 0, MAXVAL_ROTACAO, dadosPainel[0]);
}
