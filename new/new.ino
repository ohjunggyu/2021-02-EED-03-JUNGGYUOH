#include <Servo.h>
#define PIN_IR A0
#define MG946_Pin 3
#define MG946_L 553
#define MG946_M 1424
#define MG946_H 2399
#define N_SAMPLES 300
#define lim 250
#define _INTERVAL_DIST 10   
#define _INTERVAL_SERVO 23  
#define _INTERVAL_SERIAL 100  

float prevError;
float pterm;
float dterm;
float iterm;
float target;
float error;

Servo MG946;

float Kp = 13;                //P게인 값
float Ki = 0.02;                  //I게인 값
float Kd = 180;                  //D게인 값
float ir_dist = 0;
boolean D = true;
int duty_target, duty_curr;
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial; 
bool event_dist, event_servo, event_serial;
double Setpoint, Input, Output, ServoOutput;
unsigned long currentTime, previous_time; 
double Time = 0.294;
double controls = 0;
float dist_target = 31;
float dist_raw, dist_ema, dist_cali; 
float alpha;
int tmp;

int sort_asc(const void* item1, const void* item2)
{
  float a = *((float*)item1);
  float b = *((float*)item2);
  return a - b;
}

float ir_distance2(void)
{
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

void setup() {
  if (D)
  {
    Serial.begin(57600);
    Serial.println("start");
  }
  MG946.attach(MG946_Pin);
  MG946.writeMicroseconds(MG946_M); //초기화 각도삽입
}
void loop() {
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
  if (event_dist) {
    event_dist = false;
    ir_dist = ir_distance2();

    error = (dist_target - ir_dist);
    pterm = Kp * error;
    dterm = Kd * (error - prevError);
    iterm += Ki * error;
    if (abs(iterm) > 70) iterm = 0;
    prevError = error;
    target = pterm + dterm + iterm;

    tmp = target;
    if (target > lim)
    {
      tmp = lim;
    }
    if (target < -lim)
    {
      tmp = -lim;
    }
  }
  if (event_servo) {
    event_servo = false;
    duty_target = 1385 + tmp;
    duty_curr = duty_target;
    MG946.writeMicroseconds(duty_target);
  }
  if (event_serial) {
    event_serial = false;
    Serial.print("IR:");
    Serial.print(ir_dist*10-55);
    Serial.print(",T:");
    Serial.print(dist_target*10-55);
    Serial.print(",P:");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(",D:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",I:");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(",DTC:"); // [2980] “duty_curr” 출력
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
  }


}
