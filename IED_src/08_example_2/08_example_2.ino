// Arduino pin assignment
#define PIN_LED 9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0 // sound velocity at 24 celsius degree (unit: m/s)
#define INTERVAL 100 // sampling interval (unit: ms)
#define _DIST_MIN 100 // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300 // maximum distance to be measured (unit: mm)

// global variables
float timeout; // unit: us
float dist_min, dist_max, dist_raw; // unit: mm
unsigned long last_sampling_time; // unit: ms
float scale; // used for pulse duration to distance conversion

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_TRIG,OUTPUT);
  digitalWrite(PIN_TRIG, LOW); 
  pinMode(PIN_ECHO,INPUT);

// initialize USS related variables
  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  timeout = (INTERVAL / 2) * 1000.0; // precalculate pulseIn() timeout value. (unit: us)
  dist_raw = 0.0; // raw distance output from USS (unit: mm)
  scale = 0.001 * 0.5 * SND_VEL;

// initialize serial port
  Serial.begin(115200);

// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// get a distance reading from the USS
  dist_raw = USS_measure(PIN_TRIG,PIN_ECHO);

// output the read value to the serial port
  Serial.print("Min:0,");
  Serial.print("raw:");
  Serial.print(dist_raw);
  Serial.print(",");
  Serial.println("Max:400");

// turn on the LED if the distance is between dist_min and dist_max
  
  if((dist_raw >= dist_min && dist_raw < (dist_min + 50)) || (dist_raw <= dist_max && dist_raw > (dist_max - 50))) {
    analogWrite(PIN_LED, 254);
    Serial.println("min");
  }
  else if(dist_raw >= (dist_min + 50) && dist_raw <= (dist_max - 50) && dist_raw != (dist_min + 100)){
    analogWrite(PIN_LED, 127);
    Serial.println("middle");
  }
  else if(dist_raw == (dist_min + 100)){
    analogWrite(PIN_LED, 0);
    Serial.println("HIGH");
  }
  else {
    analogWrite(PIN_LED, 255);
    Serial.println("zero");
  }

// do something here
  delay(25); // Assume that it takes 50ms to do something.
  
// update last sampling time
  last_sampling_time += INTERVAL;
}

// get a distance reading from USS. return value is in millimeter.
float USS_measure(int TRIG, int ECHO)
{
  float reading;
  float Creading;
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  reading = pulseIn(ECHO, HIGH, timeout) * scale;
  if(reading <= 300 && reading != 0){
    Creading = reading;
  }
  if(reading < dist_min || reading > dist_max) reading = 0.0; // return 0 when out of range.
  if(reading == 0 || reading > 300)
     reading = Creading;
  return reading;
}
