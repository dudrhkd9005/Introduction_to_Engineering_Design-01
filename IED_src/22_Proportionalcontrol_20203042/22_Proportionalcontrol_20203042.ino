#include <Servo.h>

#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.1

#define _DUTY_MIN 1600
#define _DUTY_NEU 1800
#define _DUTY_MAX 2000

#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

#define _KP 1.3
Servo myservo;

float dist_target;
float dist_raw;

unsigned long last_sampling_time_dist, last_sampling_time_servo,last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

int duty_chg_per_interval;
int duty_target, duty_curr, dist_cali, dist_ema;
int a, b;

float error_curr, error_prev, control, pterm;
float duty_neutral;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  duty_target, duty_curr = _DIST_MIN;
  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
  
  pterm = 0;
  duty_neutral = 1610;
  Serial.begin(115200);

  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);    // [3131] 설정한 서보 스피드에 따른 duty change per interval 값을 변환
  a = 83;
  b = 342;
}
  

void loop() {
    unsigned long time_curr = millis();
    
    if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
    }
    if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
    }
    if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
    }

    if(event_dist) {
        event_dist = false;
        
        dist_raw = ir_distance_filtered();
        
        error_curr = _DIST_TARGET - dist_raw;
        pterm = error_curr;
        control = _KP * pterm;

        duty_target = duty_neutral + control;
    }
  
    if(event_servo) {
      event_servo = false;
      
      if(duty_target > duty_curr) {
              duty_curr += duty_chg_per_interval;
              if(duty_curr > duty_target) duty_curr = duty_target;
      }
      else {
          duty_curr -= duty_chg_per_interval;
          if(duty_curr < duty_target) duty_curr = duty_target;
      }

      myservo.writeMicroseconds(duty_curr);

      event_servo = false;
    }
  
    if(event_serial) {
      event_serial = false;
      Serial.print("dist_ir:");
      Serial.print(dist_raw);
      Serial.print(",pterm:");
      Serial.print(map(pterm,-1000,1000,510,610));
      Serial.print(",duty_target:");
      Serial.print(map(duty_target,1000,2000,410,510));
      Serial.print(",duty_curr:");
      Serial.print(map(duty_curr,1000,2000,410,510));
      Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
    }
}

float ir_distance(void){ // return value unit: mm
    float val;
    float volt = float(analogRead(PIN_IR));
    val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0;
    return val;
}

float ir_distance_filtered(void){ // return value unit: mm
  dist_cali = 100 + 300.0 / (b - a) * (ir_distance() - a);
  dist_ema = _DIST_ALPHA * dist_cali + (1 - _DIST_ALPHA) * dist_ema;
  return dist_ema;
}
