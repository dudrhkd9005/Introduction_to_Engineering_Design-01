#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10


// configurable parameters
#define _DUTY_MIN 670 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1670 // servo neutral position (90 degree)
#define _DUTY_MAX 2670 // servo full counterclockwise position (180 degree)

#define _POS_START (_DUTY_MIN + 100)
#define _POS_END (_DUTY_MAX - 100)

#define _SERVO_SPEED 100

// servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval
#define _DIST_ALPHA 0.99

unsigned long last_sampling_time; // unit: ms
float duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
float duty_target, duty_curr;
int currentintervel = 20000;
bool isup = true;
bool isplay = false;
int a, b; // unit: mm

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);

  Serial.begin(115200);
  
  duty_target = duty_curr = _POS_START;
  myservo.writeMicroseconds(duty_curr);
  
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (float)(INTERVAL / 1000.0);
  
  pause_time = 1;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  toggle_interval_cnt = toggle_interval;
  
  last_sampling_time = 0;
  a = 96;
  b = 342;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float ema_dist = (_DIST_ALPHA * raw_dist) + (1 - _DIST_ALPHA) * raw_dist;
  float dist_cali = 100 + 300.0 / (b - a) * (ema_dist - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);

  if(!isup) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > 1950) isplay = false;
  }
  else{
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < 800) isplay = false;
  }
  if(!isplay){
    if(dist_cali < 255){
      isup = false;
      isplay = true;
    }
    else if(dist_cali > 255){
      isup = true;
      isplay = true;
    }
  }
  
  Serial.println(duty_curr);
  Serial.println(isup);
  myservo.writeMicroseconds(duty_curr);
  last_sampling_time += INTERVAL;
  
  delay(20);
}
