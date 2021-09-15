#define PIN_LED 13
unsigned int count, toggle;

void setup() {


  
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(9600);
  while(!Serial){
    ;
  }
  Serial.println("hello world");
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  Serial.println(++count);
  toggle = toggle?false:true;
  digitalWrite(PIN_LED,toggle);
  delay(1000);
}
