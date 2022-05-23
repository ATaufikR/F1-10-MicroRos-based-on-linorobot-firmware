const byte ledPin = 13;
const byte interruptPin = 19;
int state = 0;
int last = 999;
float lastrpm = 999;
float rpm = 0;
float time_1 = 0;
float time_2 = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(interruptPin, blink, FALLING);
}

void loop() {
  get_rpm();
  if (lastrpm != rpm){
    lastrpm = rpm;
    Serial.println(rpm);
  }
}

void blink() {
  state++;
//  Serial.println(state);
}

void get_rpm() {
  if (state == 1){
    time_1 = millis();
  }
  else if (state >= 3){
    detachInterrupt(interruptPin);
    time_2 = millis();
    rpm = 60000/(time_2 - time_1);
    state = 0;
    attachInterrupt(interruptPin, blink, FALLING);
  }
}
