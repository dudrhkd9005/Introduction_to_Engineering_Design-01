int duty = 0;
int period = 0;
int i = 0;
int repeat = 0;
int PIN_NUM = 7;
void setup() {
  pinMode(PIN_NUM, OUTPUT);
  Serial.begin(115200);
}

int set_period(int period) {
  return period;
}

int set_duty(int duty){
  return duty * 0.01 * period;
}

void loop() {
  duty = set_duty(100);
  period = set_period(100); //원하는 period 값을 매개변수 인자값에 넣어준다. 0.1ms -> 100, 1ms -> 1000, 10ms -> 10000
  for(i = 0; i <= 100; i++){
    while(repeat <= 2500/period){
      duty = set_duty(i);
      digitalWrite(PIN_NUM,HIGH);
      delayMicroseconds(duty);
      digitalWrite(PIN_NUM,LOW);
      delayMicroseconds(period - duty);
      repeat++;
    }
    repeat = 0;
  }
  for(i = 100; i >= 0; i--){
   while(repeat <= 2500/period){
     duty = set_duty(i);
     digitalWrite(PIN_NUM,HIGH);
     delayMicroseconds(duty);
     digitalWrite(PIN_NUM,LOW);
     delayMicroseconds(period - duty);
     repeat++;
   }
   repeat = 0;
  }
}
