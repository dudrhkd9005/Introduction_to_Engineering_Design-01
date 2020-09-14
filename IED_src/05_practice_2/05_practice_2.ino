#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  count = toggle = 0;
}

void loop() {
  Serial.println(++count);
  toggle_state(toggle);
}

void toggle_state(int toggle){
  toggle = 0;
  Serial.println(toggle);
  digitalWrite(PIN_LED, toggle);
  delay(1000);
  for(int i = 0; i < 5; i++){
    toggle = 1;
    digitalWrite(PIN_LED, toggle);
    delay(100);
    toggle = 0;
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  while(1){
    digitalWrite(PIN_LED, 1);
  }
}
