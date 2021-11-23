#include <PID_v1.h>
#include <Servo.h>
#define PIN_IR A0
#define MG946_Pin 3
#define MG946_L 553
#define MG946_M 1424
#define MG946_H 2399
#define N_SAMPLES 100
//1424 923 871 85도 중앙 기준
Servo MG946;

float Kp = 2.5;                //P게인 값
float Ki = 0;                  //I게인 값 
float Kd = 1;                  //D게인 값
double Setpoint, Input, Output, ServoOutput;                                       

int a, b; // unit: mm


int servo_duty(int a){
  return map(a,0,180,MG946_L,MG946_H);
}
int dist_map(int a){
  return map(a,600,0,60,120);
}

float ir_distance2(void){ // return value unit: mm
  float ir_val[N_SAMPLES];
  float tmp;

  // take N_SAMPLES and sort them in an ascending order.
  ir_val[0] = float(analogRead(PIN_IR));
  for (int i=1; i<N_SAMPLES; i++) { 
    delayMicroseconds(10);
    ir_val[i] = float(analogRead(PIN_IR));
    for(int j = i-1; j >= 0; j--) { // i-1 to 0
      if(ir_val[j] > ir_val[j+1]) {
        tmp = ir_val[j];
        ir_val[j] = ir_val[j+1];
        ir_val[j+1] = tmp;
      }
    }
  }

  int cnt = N_SAMPLES/3;
  tmp = 0.0;
  for(int i = 0; i < cnt; i++) {
      tmp += ir_val[i];
  }
  tmp = tmp / (float) cnt;
  tmp = ((6762.0/(tmp-9.0))-4.0);

  return tmp;
}

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);   //PID객체 생성

void setup() {
  Serial.begin(9600);
  Serial.println("start");
  MG946.attach(MG946_Pin);
  MG946.writeMicroseconds(MG946_M); //초기화 각도삽입

  myPID.SetMode(AUTOMATIC);               //PID모드를 AUTOMATIC으로 설정
  myPID.SetOutputLimits(-50,50);          //PID의 값을 최소 -80부터 최대 80까지 설정
}
                   //막대 중앙 위치(Set Point를 15cm로 설정)
void loop() {
  Setpoint = 30;
  float raw_dist = ir_distance2();
  Serial.print("raw_dist:");
  Serial.print(raw_dist);
  Input = raw_dist;
  myPID.Compute(); 
  ServoOutput=87+Output;
  Serial.print(",ServoOutput:");
  Serial.println(ServoOutput);
  MG946.writeMicroseconds(servo_duty(ServoOutput));
  
}
