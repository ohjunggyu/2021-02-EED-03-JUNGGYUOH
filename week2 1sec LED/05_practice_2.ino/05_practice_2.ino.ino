#define p7 7

int start = 0;
unsigned long cur = 0;
unsigned long prev = 0;
int TD = 1000;
bool state = HIGH;
int cnt = 0;

void setup() {
  pinMode(p7, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(p7, state);
  delay(1000);
  if (cnt < 11)
  {
    cur = millis();
    if (cur - prev >= TD) {
      prev = cur;
      state = state ? LOW : HIGH;
      Serial.print(cnt);
      cnt++;
      digitalWrite(p7, state);
    }
  }
  digitalWrite(p7, LOW);
}
