#include <PID_v1.h>
#include <Servo.h>
#define PIN_IR A0
#define MG946_Pin 3
#define MG946_L 553
#define MG946_M 1424
#define MG946_H 2399
#define N_SAMPLES 300
#define SP 31
//1424 923 871 85도 중앙 기준
Servo MG946;
//20 1 11
float Kp = 1.7;                //P게인 값
float Ki = 0.9;                  //I게인 값
float Kd = 0.95;                  //D게인 값
double Setpoint, Input, Output, ServoOutput;

int a, b; // unit: mm

boolean D = true;

int sort_asc(const void* item1, const void* item2)
{
  float a = *((float*)item1);
  float b = *((float*)item2);
  return a - b;
}

int servo_duty(int a) {
  return map(a, 0, 180, MG946_L, MG946_H);
}

float ir_distance2(void) { // return value unit: mm
  float ir_val[N_SAMPLES];
  float tmp;
  // take N_SAMPLES and sort them in an ascending order.
  ir_val[0] = float(analogRead(PIN_IR));

  for (int i = 1; i < N_SAMPLES; i++) {
    ir_val[i] = float(analogRead(PIN_IR));
  }

  qsort(ir_val, N_SAMPLES, sizeof(ir_val[0]), sort_asc);

  int cnt = N_SAMPLES / 2;
  tmp = 0.0;
  for (int i = 50; i < cnt + 50; i++) {
    tmp += ir_val[i];
  }
  tmp = tmp / (float) cnt;
  tmp = ((6762.0 / (tmp - 9.0)) - 4.0);
  return tmp;
}

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);   //PID객체 생성

void setup() {
  if (D)
  {
    Serial.begin(57600);
    Serial.println("start");
  }
  MG946.attach(MG946_Pin);
  MG946.writeMicroseconds(MG946_M); //초기화 각도삽입

  myPID.SetMode(AUTOMATIC);               //PID모드를 AUTOMATIC으로 설정
  myPID.SetOutputLimits(-15,15);         //PID의 값을 최소 -80부터 최대 80까지 설정
}
void loop() {
  //pid outotune
  //myPID.SetTunings(Kp, Ki, Kd);   
  //myPID.Compute();
  
  Setpoint = SP;
  float raw_dist = ir_distance2();  
  if (abs(raw_dist - SP) < 0.8)
  {
    raw_dist = SP;
  }
  if (D)
  {
    Serial.print("Set Point:");
    Serial.print(SP);
    Serial.print(",raw_dist:");
    Serial.print(raw_dist);
    Serial.print(",raw_ist:");
    Serial.print(600);
  }
  Input = raw_dist;
  myPID.Compute();
  ServoOutput = 85 + Output;
  if (D)
  {
    Serial.print(",ServoOutput:");
    Serial.println(ServoOutput);
  }
  MG946.writeMicroseconds(servo_duty(ServoOutput));

}
