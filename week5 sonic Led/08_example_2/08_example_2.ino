// Arduino pin assignment
#define PIN_LED 5
#define PIN_TRIG 2
#define PIN_ECHO 3

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 25 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion
int LED_POW = 0;
void setup() {
  // initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  
  // initialize USS related variables
  dist_min = _DIST_MIN;
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

  // initialize serial port
  Serial.begin(57600);

  // initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
  // wait until next sampling time.
  // millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if (millis() < last_sampling_time + INTERVAL) return;

  // get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO, dist_raw);


  // output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.println("Max:400");

  LED_POW = map(dist_raw,100, 300, 0, 255);
  // turn on the LED if the distance is between dist_min and dist_max
  if (dist_raw < dist_min || dist_raw > dist_max) {
    analogWrite(PIN_LED, 255);
  }
  else {
    analogWrite(PIN_LED, LED_POW);
  }

  // do something here
  //delay(50); // Assume that it takes 50ms to do something.

  // update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO, int cur)
{
  float reading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale; // unit: mm
  if (reading < dist_min || reading > dist_max) reading = cur; // return 0 when out of range.
  return reading;
}
