#include <PID_v1.h>
#include "TimerOne.h"
#include <aJSON.h>
aJsonStream serial_stream(&Serial);

int AnguloRef[3]; // ANGULO DE REFERENCIA
int pulses[3]; // PULSOS ENCODER
int SaidaPWM[3];

// DEFINICAO PINOS ENCODER A B
int encoderBA = 3;
int encoderBB = 2;
int encoderOA = 20;
int encoderOB = 21;
int encoderCA = 18;
int encoderCB = 19;
int juan = 0;
int i =0;
int mattraj[501][3] ={{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{0,0,0},{1,-1,1},{1,-1,1},{1,-1,1},{1,-1,1},{1,-1,1},{2,-2,2},{2,-2,2},{2,-2,2},{2,-2,2},{3,-3,3},{3,-3,3},{4,-4,4},{4,-4,4},{5,-5,5},{5,-5,5},{6,-6,6},{7,-7,7},{7,-7,7},{8,-8,8},{9,-9,9},{9,-9,9},{10,-10,10},{11,-11,11},{12,-12,12},{13,-13,13},{14,-14,14},{15,-15,15},{16,-16,16},{17,-17,17},{19,-19,19},{20,-20,20},{21,-21,21},{23,-23,23},{24,-24,24},{26,-26,26},{27,-27,27},{29,-29,29},{30,-30,30},{32,-32,32},{34,-34,34},{35,-35,35},{37,-37,37},{39,-39,39},{41,-41,41},{43,-43,43},{45,-45,45},{47,-47,47},{49,-49,49},{51,-51,51},{54,-54,54},{56,-56,56},{58,-58,58},{61,-61,61},{63,-63,63},{65,-65,65},{68,-68,68},{70,-70,70},{73,-73,73},{76,-76,76},{78,-78,78},{81,-81,81},{84,-84,84},{87,-87,87},{90,-90,90},{93,-93,93},{96,-96,96},{99,-99,99},{102,-102,102},{105,-105,105},{108,-108,108},{111,-111,111},{114,-114,114},{118,-118,118},{121,-121,121},{124,-124,124},{128,-128,128},{131,-131,131},{135,-135,135},{138,-138,138},{142,-142,142},{145,-145,145},{149,-149,149},{152,-152,152},{156,-156,156},{160,-160,160},{163,-163,163},{167,-167,167},{171,-171,171},{175,-175,175},{178,-178,178},{182,-182,182},{186,-186,186},{190,-190,190},{194,-194,194},{198,-198,198},{202,-202,202},{206,-206,206},{210,-210,210},{214,-214,214},{218,-218,218},{222,-222,222},{226,-226,226},{230,-230,230},{234,-234,234},{238,-238,238},{242,-242,242},{246,-246,246},{250,-250,250},{254,-254,254},{259,-259,259},{263,-263,263},{267,-267,267},{271,-271,271},{275,-275,275},{279,-279,279},{283,-283,283},{287,-287,287},{292,-292,292},{296,-296,296},{300,-300,300},{304,-304,304},{308,-308,308},{312,-312,312},{316,-316,316},{320,-320,320},{324,-324,324},{328,-328,328},{332,-332,332},{336,-336,336},{340,-340,340},{344,-344,344},{348,-348,348},{352,-352,352},{356,-356,356},{360,-360,360},{364,-364,364},{368,-368,368},{372,-372,372},{375,-375,375},{379,-379,379},{383,-383,383},{387,-387,387},{390,-390,390},{394,-394,394},{398,-398,398},{401,-401,401},{405,-405,405},{408,-408,408},{412,-412,412},{415,-415,415},{419,-419,419},{422,-422,422},{426,-426,426},{429,-429,429},{432,-432,432},{436,-436,436},{439,-439,439},{442,-442,442},{445,-445,445},{448,-448,448},{451,-451,451},{454,-454,454},{457,-457,457},{460,-460,460},{463,-463,463},{466,-466,466},{469,-469,469},{472,-472,472},{474,-474,474},{477,-477,477},{480,-480,480},{482,-482,482},{485,-485,485},{487,-487,487},{490,-490,490},{492,-492,492},{494,-494,494},{496,-496,496},{499,-499,499},{501,-501,501},{503,-503,503},{505,-505,505},{507,-507,507},{509,-509,509},{511,-511,511},{513,-513,513},{515,-515,515},{516,-516,516},{518,-518,518},{520,-520,520},{521,-521,521},{523,-523,523},{524,-524,524},{526,-526,526},{527,-527,527},{529,-529,529},{530,-530,530},{531,-531,531},{533,-533,533},{534,-534,534},{535,-535,535},{536,-536,536},{537,-537,537},{538,-538,538},{539,-539,539},{540,-540,540},{541,-541,541},{541,-541,541},{542,-542,542},{543,-543,543},{544,-544,544},{544,-544,544},{545,-545,545},{545,-545,545},{546,-546,546},{546,-546,546},{547,-547,547},{547,-547,547},{548,-548,548},{548,-548,548},{548,-548,548},{548,-548,548},{549,-549,549},{549,-549,549},{549,-549,549},{549,-549,549},{549,-549,549},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550},{550,-550,550}};


 
// definição pinos PWM
#define enB  10
#define in1B  12
#define in2B  11
#define enO  4
#define in1O  6
#define in2O  5
#define enC  7
#define in1C  8
#define in2C  9

// ANGUO MAX E MIN
#define Bmin -4400
#define Bmax 4400
#define BmaxGraus 4400

#define Omin -4400
#define Omax 4400
#define OmaxGraus 4400

#define Cmin -4400
#define Cmax 4400
#define CmaxGraus 4400

// TEVE PULSO ?  
int BpulsesChanged = 0;
int OpulsesChanged = 0;
int CpulsesChanged = 0;
int vmax = 200;
const int sampleRate = 1; // Variable that determines how fast our PID loop runs
double Setpoint[3], Input[3], Output[3]; //These are just variables for storingvalues
double aggKp=10, aggKi=6, aggKd=1;
double consKp=50, consKi=50, consKd=20;
PID myPID0(&Input[0], &Output[0], &Setpoint[0], consKp, consKi, consKd, DIRECT);
PID myPID1(&Input[1], &Output[1], &Setpoint[1], consKp, consKi, consKd, DIRECT);
PID myPID2(&Input[2], &Output[2], &Setpoint[2], consKp, consKi, consKd, DIRECT);

void setup(){
  Serial.begin(57600);

  Timer1.initialize(50000);  
  Timer1.attachInterrupt(imprime); 
  
  // PINOS ENCODER COMO ENTRADA
  pinMode(encoderBA, INPUT);
  pinMode(encoderBB, INPUT);
  pinMode(encoderOA, INPUT);
  pinMode(encoderOB, INPUT);
  pinMode(encoderCA, INPUT);
  pinMode(encoderCB, INPUT);  

  // INTERRUPCAO ENCODER
  attachInterrupt(0, BA_CHANGE, CHANGE);
  attachInterrupt(1, BB_CHANGE, CHANGE);
  attachInterrupt(2, OA_CHANGE, CHANGE);
  attachInterrupt(3, OB_CHANGE, CHANGE);
  attachInterrupt(4, CA_CHANGE, CHANGE);
  attachInterrupt(5, CB_CHANGE, CHANGE);

  myPID0.SetMode(AUTOMATIC); //Turn on the PID loop
  myPID0.SetSampleTime(sampleRate); //Sets the sample rate
  myPID0.SetOutputLimits(-255, 255);
  myPID1.SetMode(AUTOMATIC); //Turn on the PID loop
  myPID1.SetSampleTime(sampleRate); //Sets the sample rate
  myPID1.SetOutputLimits(-255, 255);
  myPID2.SetMode(AUTOMATIC); //Turn on the PID loop
  myPID2.SetSampleTime(sampleRate); //Sets the sample rate
  myPID2.SetOutputLimits(-255, 255);

  AnguloRef[0] = 0;
  AnguloRef[1] = 0;
  AnguloRef[2] = 0;
}//setup

void loop(){
  //ZERAR OS PULSOS CHANGE
  zerapulso();
  //PID E PWM CADA MOTOR
  Input[0] = pulses[0];
  Setpoint[0] = AnguloRef[0];
//  double gap0 = abs(Setpoint[0]-Input[0]); //distance away from setpoint
//  if(gap0>10)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID0.SetTunings(consKp, consKi, consKd);
//  }
//  else
//  {
//     myPID0.SetTunings(aggKp, aggKi, aggKd);
//  } 
  myPID0.Compute();
  pwm0();
  Input[1] = pulses[1];
  Setpoint[1] = AnguloRef[1]; 
//  double gap1 = abs(Setpoint[1]-Input[1]); //distance away from setpoint
//  if(gap1>10)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID1.SetTunings(consKp, consKi, consKd);
//  }
//  else
//  {
//     myPID1.SetTunings(aggKp, aggKi, aggKd);
//  }
  myPID1.Compute();
  pwm1();
  Input[2] = pulses[2];
  Setpoint[2] = AnguloRef[2]; 
//  double gap2 = abs(Setpoint[2]-Input[2]); //distance away from setpoint
//  if(gap2>10)
//  {  //we're close to setpoint, use conservative tuning parameters
//    myPID2.SetTunings(consKp, consKi, consKd);
//  }
//  else
//  {
//     myPID2.SetTunings(aggKp, aggKi, aggKd);
//  }
  myPID2.Compute();
  pwm2();

}

void zerapulso(){
   if (BpulsesChanged != 0) {
    BpulsesChanged = 0;
  }
  if (OpulsesChanged != 0) {
    OpulsesChanged = 0;
  }
  if (CpulsesChanged != 0) {
    CpulsesChanged = 0;
  }
  }

void imprime(){
  Serial.print(pulses[0]);
  Serial.print(",");
  Serial.print(mattraj[i][0]);
  Serial.print(pulses[1]);
  Serial.print(",");
  Serial.print(mattraj[i][1]);
  Serial.print(pulses[2]);
  Serial.print(",");
  Serial.print(mattraj[i][2]);
  Serial.print("\n");
juan++;
if(juan==1){
  juan =0;

  if(i<501){
            AnguloRef[0]=mattraj[i][0];
            AnguloRef[1]=mattraj[i][1];
            AnguloRef[2]=mattraj[i][2];
            i++;
    }
  }
}

 
void pwm0(){
  SaidaPWM[0] = abs(Output[0]);
  SaidaPWM[0] = constrain(SaidaPWM[0], 0, vmax);

    analogWrite(enB, SaidaPWM[0]);
    if (Output[0] < 0 ) {
      digitalWrite(in1B, HIGH);
      digitalWrite(in2B, LOW);
    }
    if (Output[0] > 0 ) {
      digitalWrite(in1B, LOW);
      digitalWrite(in2B, HIGH);
    }
  }
  void pwm1(){
  SaidaPWM[1] = abs(Output[1]);
  SaidaPWM[1] = constrain(SaidaPWM[1], 0, vmax);

    analogWrite(enO, SaidaPWM[1]);
    if (Output[1] < 0 ) {
      digitalWrite(in1O, HIGH);
      digitalWrite(in2O, LOW);
    }
    if (Output[1] > 0 ) {
      digitalWrite(in1O, LOW);
      digitalWrite(in2O, HIGH);
    }
  }
  void pwm2(){
  SaidaPWM[2] = abs(Output[2]);
  SaidaPWM[2] = constrain(SaidaPWM[2], 0, vmax);

    analogWrite(enC, SaidaPWM[2]);
    if (Output[2] < 0 ) {
      digitalWrite(in1C, HIGH);
      digitalWrite(in2C, LOW);
    }
    if (Output[2] > 0 ) {
      digitalWrite(in1C, LOW);
      digitalWrite(in2C, HIGH);
    }
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

// ATUALIZACAO DOS PULSOS OMBRO
void OA_CHANGE(){
  if( digitalRead(encoderOB) == 0 ) {
    if ( digitalRead(encoderOA) == 0 ) {
      // A fell, B is low
      pulses[1]--; // moving reverse
    } else {
      // A rose, B is low
      pulses[1]++; // moving forward
    }
 } else {
    if ( digitalRead(encoderOA) == 0 ) {
      // A fell, B is high
      pulses[1]++; // moving forward
    } else {
      // A rose, B is high
      pulses[1]--; // moving reverse
    }
  }
  // tell the loop that the pulses have changed
  OpulsesChanged = 1;
}

void OB_CHANGE(){
  if ( digitalRead(encoderOA) == 0 ) {
    if ( digitalRead(encoderOB) == 0 ) {
      // B fell, A is low
      pulses[1]++; // moving forward
    } else {
      // B rose, A is low
      pulses[1]--; // moving reverse
    }
 } else {
    if ( digitalRead(encoderOB) == 0 ) {
      // B fell, A is high
      pulses[1]--; // moving reverse
    } else {
      // B rose, A is high
      pulses[1]++; // moving forward
    }
  }
 // tell the loop that the pulses have changed
  OpulsesChanged = 1;
}

// ATUALIZACAO DOS PULSOS COTOVELO
void CA_CHANGE(){
  if( digitalRead(encoderCB) == 0 ) {
    if ( digitalRead(encoderCA) == 0 ) {
      // A fell, B is low
      pulses[2]--; // moving reverse
    } else {
      // A rose, B is low
      pulses[2]++; // moving forward
    }
 } else {
    if ( digitalRead(encoderCA) == 0 ) {
      // A fell, B is high
      pulses[2]++; // moving forward
    } else {
      // A rose, B is high
      pulses[2]--; // moving reverse
    }
  }
  // tell the loop that the pulses have changed
  CpulsesChanged = 1;
}

void CB_CHANGE(){
  if ( digitalRead(encoderCA) == 0 ) {
    if ( digitalRead(encoderCB) == 0 ) {
      // B fell, A is low
      pulses[2]++; // moving forward
    } else {
      // B rose, A is low
      pulses[2]--; // moving reverse
    }
 } else {
    if ( digitalRead(encoderCB) == 0 ) {
      // B fell, A is high
      pulses[2]--; // moving reverse
    } else {
      // B rose, A is high
      pulses[2]++; // moving forward
    }
  }
 // tell the loop that the pulses have changed
  CpulsesChanged = 1;
}
