
static long apt = 0; 
#define interval 2 // milliseconds
unsigned long oldmil;


int fc = 5; 
float dt = interval/1000.0; 
float lambda = 2*PI*fc*dt;
float calidist = 0.0, filter = 0.0, prev = 0.0;

void setup() {
 Serial.begin(9600);
}


void loop() {
unsigned long dmil = 0;
unsigned long mil = millis();  
if (mil != oldmil) { 
  dmil = mil-oldmil; 
  oldmil = mil;   
} 

apt -= dmil; 

if (apt <= 0) {  
  apt += interval; 

  calidist = analogRead(A0); 
  filter = lambda/(1+lambda)*calidist+1/(1+lambda)*prev; 
  prev = filter;

  Serial.print("og data:");
  Serial.print(calidist);
  Serial.print(", filter:");
  Serial.println(filter);  
  
 }

}
