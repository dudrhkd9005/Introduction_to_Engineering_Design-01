#define PIN_LED 13
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);
  while(!Serial){
    ;
  }
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle);
}

void loop() {
  toggle = toggle_state(toggle);
  toggle = 0;
  digitalWrite(PIN_LED, toggle);
  Serial.println("LED OFF");
  delay(1000);
  Serial.println("LED ON");
  toggle = 1;
  digitalWrite(PIN_LED, toggle);
  delay(1000);
}

int toggle_state(int toggle){
  return toggle;
}
