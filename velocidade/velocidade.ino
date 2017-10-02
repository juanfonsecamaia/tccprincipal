#include <PID_v1.h>
#include "TimerOne.h"
#include <aJSON.h>
aJsonStream serial_stream(&Serial);

unsigned long pulses[3]; // PULSOS ENCODER
int SaidaPWM;
int AnguloRef[3];
unsigned long pulsoatual=0, pulsoanterior=0;

// DEFINICAO PINOS ENCODER A B
int encoderBA = 3;
int encoderBB = 2;

// definição pinos PWM
#define enB  10
#define in1B  12
#define in2B  11


// TEVE PULSO ?  
int BpulsesChanged = 0;
int OpulsesChanged = 0;
int CpulsesChanged = 0;
int vmax = 255;

void setup(){
  Serial.begin(57600);

  Timer1.initialize(50000);  
  Timer1.attachInterrupt(imprime);
   
  
  // PINOS ENCODER COMO ENTRADA
  pinMode(encoderBA, INPUT);
  pinMode(encoderBB, INPUT);
 

  // INTERRUPCAO ENCODER
  attachInterrupt(0, BA_CHANGE, CHANGE);
  attachInterrupt(1, BB_CHANGE, CHANGE);
}//setup

void loop(){
  zerapulso();
  pwm0();

  //* Le dados da porta serial
  if (serial_stream.available()) {
    /* First, skip any accidental whitespace like newlines. */
    serial_stream.skip();
  }
  if (serial_stream.available()) {
    /* Something real on input, let's take a look. */
    aJsonObject *msg = aJson.parse(&serial_stream);
    processMessage(msg);
    aJson.deleteItem(msg);
  }

}

////Exemplo de recebimento --> {"Dados":{"B":100,"O":60,"C":80,"P":50,"G":2}}
void processMessage(aJsonObject *msg)
{
  aJsonObject *dados = aJson.getObjectItem(msg, "Dados");

  aJsonObject *aBase = aJson.getObjectItem(dados, "B");


  if (  aBase != NULL) {  // Caso haja valor no objeto Base
    AnguloRef[0] = aBase->valueint;  // Converte p/ inteiro
    SaidaPWM = constrain(AnguloRef[0], 0, 255); // Garante que os valores não ultrapassem os limites

    
  }
  }

void zerapulso(){
   if (BpulsesChanged != 0) {
    BpulsesChanged = 0;
  }

  }
double tempo = 0;
void imprime(){
  pulsoatual = pulses[0];
double  vel = (pulsoatual - pulsoanterior)/0.05;
  vel = (vel/2200);
  vel = (vel*60);
  
  tempo = tempo + 0.05;
  Serial.print(vel);
  Serial.print("\n");
  pulsoanterior = pulsoatual;
}

void pwm0(){

    analogWrite(10, 255);
      digitalWrite(in1B, LOW);
      digitalWrite(in2B, HIGH);
  }

// ATUALIZACAO DOS PULSOS BASE
void BA_CHANGE(){
  if( digitalRead(encoderBB) == 0 ) {
    if ( digitalRead(encoderBA) == 0 ) {
      // A fell, B is low
      pulses[0]--; // moving reverse
    } else {
      // A rose, B is low
      pulses[0]++; // moving forward
    }
 } else {
    if ( digitalRead(encoderBA) == 0 ) {
      // A fell, B is high
      pulses[0]++; // moving forward
    } else {
      // A rose, B is high
      pulses[0]--; // moving reverse
    }
  }
  // tell the loop that the pulses have changed
  BpulsesChanged = 1;
}

void BB_CHANGE(){
  if ( digitalRead(encoderBA) == 0 ) {
    if ( digitalRead(encoderBB) == 0 ) {
      // B fell, A is low
      pulses[0]++; // moving forward
    } else {
      // B rose, A is low
      pulses[0]--; // moving reverse
    }
 } else {
    if ( digitalRead(encoderBB) == 0 ) {
      // B fell, A is high
      pulses[0]--; // moving reverse
    } else {
      // B rose, A is high
      pulses[0]++; // moving forward
    }
  }
 // tell the loop that the pulses have changed
  BpulsesChanged = 1;
}

