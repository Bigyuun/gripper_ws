void setup() {
  // put your setup code here, to run once:  
//  HardwareTimer pwmtimer1(1);
  HardwareTimer pwmtimer2(2);

//  pinMode(PA11, PWM);
  pinMode(PA2, PWM);
  
//  pwmtimer1.pause();
//  pwmtimer1.setCount(0);
//  pwmtimer1.setPrescaleFactor(1); // 72/1 = 72
//  pwmtimer1.setOverflow(3600-1); // 72mHz/3600 = 20 (kHz)
//  pwmtimer1.refresh();
//  pwmtimer1.resume();

  pwmtimer2.pause();
  pwmtimer2.setCount(0);
  pwmtimer2.setPrescaleFactor(1); // 72/1 = 72
  pwmtimer2.setOverflow(3600-1); // 72mHz/3600 = 20 (kHz)
  pwmtimer2.refresh();
  pwmtimer2.resume();

//  pwmWrite(PA11, 0);
  pwmWrite(PA2, 1800);
}

void loop() {
  // put your main code here, to run repeatedly:
//  for(int i=0; i<3600; i++)
//  {
////    pwmWrite(PA11, i);
//    pwmWrite(PA2, i);
//    delay(1);
//  }
}
