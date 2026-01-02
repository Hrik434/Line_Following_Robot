//Motor
#include <SparkFun_TB6612.h>

int whiteDelay=400;
int baseSpeedA=100;
int baseSpeedB=100;
int thresh=100;

const int offsetA = 1;
const int offsetB = 1;

#define AIN1 9
#define BIN1 6
#define AIN2 10
#define BIN2 7
#define PWMA 11
#define PWMB 5
#define STBY 8

Motor motorR = Motor(AIN1, AIN2, PWMA, offsetA, STBY);  // MotorA
Motor motorL = Motor(BIN1, BIN2, PWMB, offsetB, STBY);  //MotorB

float Ki = 0, Kd = 400, baseSpeed = 105,  MaxSpeed = 145, ForMax = 4.5;
float Kp = (MaxSpeed + baseSpeed) / ForMax, error = 0, lastError = 0, P = 0, I = 0, D = 0, PID = 0, leftPID = 0, rightPID = 0;


//PID
int lasterror=0;
/*float Kp=0.005;
float Kd=3;
float Ki=0;
float I=0;*/
int pidSpeedUA=120;
int pidSpeedUB=120;


//arrays
int sensorState[26];
int thresholdArray[26];
int lowVal[26];
int highVal[26];
int weightage[26]={};
int junc_state[5]={};//left 1 right 2 T 3 + 4 ver l T 5 ver r T 6


//Mux pins
#define S0 2 //D4                        0 1 *2 3 *4 5 *6 7 *8 9 *10 11 12
#define S1 3 //D3
#define S2 4 //D2
#define S3 5 //D13
#define S4 6 //D4                        0 1 *2 3 *4 5 *6 7 *8 9 *10 11 12
#define S5 7 //D3
#define S6 8 //D2
#define S7 12 //D13

//Calibration
#define calibPin 12

//float error=0;
float sum;
float rms;
int num_sensor;
int conditionDelay=44;
int gap_forward_delay=100;
int white=0;

int invert_flag=0;
int reverse_flag=0;
int front_flag=0;
int dot_true=0;
int dot_flag=0;
int t=0;
int t2=0;
int blank_;
float st3=0;
int curve=0;
int time=0;
int blank;
int plus=0;
int dead_end=0;
int it_is_already_reverse=0;


int po_right;
int po_left;
int po_k;
int po_T;
int po_plus;
int po_loop;

int readSensor(int ir){
  if(invert_flag==0){
    if (ir==25){
      digitalWrite(S0, 1);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==1){
    
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==2){
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==3){
    
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==4){
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==5){
      
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==6){
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==7){
      
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==8){
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==9){
      
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    else if (ir==10){
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==11){
      
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      return analogRead(A6);
    }
    
    else if (ir==12){
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      return analogRead(A6);
    }
    else if (ir==13){
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==14){
    
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    else if (ir==15){
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==16){
    
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    else if (ir==17){
      digitalWrite(S4, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==18){
      
      digitalWrite(S4, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    else if (ir==19){
      digitalWrite(S4,0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==20){
      
      digitalWrite(S1, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    else if (ir==21){
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==22){
      
      digitalWrite(S4,1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    else if (ir==23){
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
    else if (ir==24){
      
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      return analogRead(A7);
    }
    
    else if (ir==26){
      digitalWrite(S4, 1);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      return analogRead(A7);
    }
  }

  else{

    if (ir==25){
      digitalWrite(S0, 1);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[25]) return 50;
      else return 500;
    }
    else if (ir==1){
    
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[1]) return 50;
      else return 500;
    }
    else if (ir==2){
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[2]) return 50;
      else return 500;
    }
    else if (ir==3){
    
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[3]) return 50;
      else return 500;
    }
    else if (ir==4){
      digitalWrite(S0, 0);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[4]) return 50;
      else return 500;
    }
    else if (ir==5){
      
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[5]) return 50;
      else return 500;
    }
    else if (ir==6){
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[6]) return 50;
      else return 500;
    }
    else if (ir==7){
      
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[7]) return 50;
      else return 500;
    }
    else if (ir==8){
      digitalWrite(S0, 0);
      digitalWrite(S1, 1);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[8]) return 50;
      else return 500;
    }
    else if (ir==9){
      
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[9]) return 50;
      else return 500;
    }
    else if (ir==10){
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 0);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[10]) return 50;
      else return 500;
    }
    else if (ir==11){
      
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 0);
      if(analogRead(A6)>thresholdArray[11]) return 50;
      else return 500;
    }
    
    else if (ir==12){
      digitalWrite(S0, 1);
      digitalWrite(S1, 0);
      digitalWrite(S2, 1);
      digitalWrite(S3, 1);
      if(analogRead(A6)>thresholdArray[12]) return 50;
      else return 500;
    } 
    else if (ir==13){
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[13]) return 50;
      else return 500;
    }
    else if (ir==14){
    
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[14]) return 50;
      else return 500;
    }
    else if (ir==15){
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[15]) return 50;
      else return 500;
    }
    else if (ir==16){
    
      digitalWrite(S4, 0);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[16]) return 50;
      else return 500;
    }
    else if (ir==17){
      digitalWrite(S4, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[17]) return 50;
      else return 500;
    }
    else if (ir==18){
      
      digitalWrite(S4, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[18]) return 50;
      else return 500;
    }
    else if (ir==19){
      digitalWrite(S4,0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[19]) return 50;
      else return 500;
    }
    else if (ir==20){
      
      digitalWrite(S1, 0);
      digitalWrite(S5, 1);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[20]) return 50;
      else return 500;
    }
    else if (ir==21){
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[21]) return 50;
      else return 500;
    }
    else if (ir==22){
      
      digitalWrite(S4,1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 0);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[22]) return 50;
      else return 500;
    }
    else if (ir==23){
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[23]) return 50;
      else return 500;
    }
    else if (ir==24){
      
      digitalWrite(S4, 1);
      digitalWrite(S5, 0);
      digitalWrite(S6, 1);
      digitalWrite(S7, 1);
      if(analogRead(A7)>thresholdArray[24]) return 50;
      else return 500;
    }
    
    else if (ir==26){
      digitalWrite(S4, 1);
      digitalWrite(S5, 1);
      digitalWrite(S6, 0);
      digitalWrite(S7, 0);
      if(analogRead(A7)>thresholdArray[26]) return 50;
      else return 500;
    }
  }
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(A6,INPUT);
  pinMode(A3,INPUT);
  pinMode(A2,INPUT);
  pinMode(A1,INPUT);
  pinMode(A0,INPUT);
  pinMode(A7, INPUT);
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);

  pinMode(calibPin,INPUT);
  
  while(!digitalRead(calibPin)){
    continue;
  }

  delay(2000);
  motorR.drive(baseSpeedA);
  motorL.drive(baseSpeedB);
  delay(200);
  }

void loop() {

  for(int i=0;i<26;i++){
    Serial.print(readSensor(i));
    Serial.print(" ");
  }
 
    
}