#define Baudrate 115200

#define MOTOR_PWM_PIN 9 // recommend pin #10 just use for PWM timer
#define MOTOR_DIRECTION 4
#define MOTOR_STSP 5
#define MOTOR_FREQUENCY 799

static int motor_duty = 300; // 50%
static int enc_counter = 0;

void FastPWMRegisterSet();
void Initialize();


void setup() {
  // put your setup code here, to run once:
  Initialize();
  FastPWMRegisterSet();
}

void loop() {
  // put your main code here, to run repeatedly:
  //OCR1A = analogRead(POTENTIOMETER); // 0~1023

  Serial.print("Duty : "); Serial.println(OCR1A);
  //delay(10);/}
}

void Initialize()
{
  Serial.begin(Baudrate);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION, OUTPUT);
  pinMode(MOTOR_STSP, OUTPUT);

  digitalWrite(MOTOR_DIRECTION, HIGH);
  digitalWrite(MOTOR_STSP, LOW);
  
}

void FastPWMRegisterSet()
{
  TCCR1A = bit(COM1A1) | bit(COM1A0) | bit(COM1B1) | bit(COM1B0); //inverting mode => 0일때 HIGH, MAX일때 LOW
  TCCR1A |= bit(WGM11);
  TCCR1B = bit(WGM12) | bit(WGM13); // Fast PWM mode using ICR1 as TOP
  TCCR1B |= bit(CS10); // no prescaler
  ICR1 = MOTOR_FREQUENCY; //주파수 설정 => 16MHz / ICR1 = 주파수
  OCR1A = motor_duty; // 듀티 설정 => OCR1A/ICR1
  OCR1B = 0;
  TCNT1 = 0;
}
