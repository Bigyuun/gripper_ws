HardwareTimer pwmtimer2(2);



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(PB9, PWM);
  pinMode(PA0, PWM);
  pinMode(PA1, PWM);
  pinMode(PA2, PWM);
  pinMode(PA3, PWM);
  pinMode(PA6, PWM);
  pinMode(PA7, PWM);

  
  pwmtimer2.pause();
  pwmtimer2.setCount(0);
  pwmtimer2.setPrescaleFactor(1); // 72/1 = 72
  pwmtimer2.setOverflow(3600-1); // 72mHz/3600 = 20 (kHz)
  pwmtimer2.refresh();
  pwmtimer2.resume();

  pwmWrite(PA0, 50);
  pwmWrite(PA1, 500);
  pwmWrite(PA2, 3600);
  pwmWrite(PA3, 1800);
  pwmWrite(PA6, 3400);
  pwmWrite(PA7, 1);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("STM");
  delay(500);
}
