#define Baudrate 115200

#define POTENTIOMETER A0

#define ENCODER_PHASE_A 2 // pin #2,3 are only things for using interrupt
#define ENCODER_PAHSE_B 3
#define MOTOR_PWM_PIN 9 // recommend pin #10 just use for PWM timer
#define MOTOR_DIRECTION 7
#define MOTOR_STSP 5
#define MOTOR_FREQUENCY 799

void FastPWMRegisterSet();
void Initialize();
void EncoderInit();
void isrA();
void isrB();

static long g_enc_pos = 0;
static int motor_duty = 0; // 50%
static int flag = 0;



void setup() {
  // put your setup code here, to run once:
  Initialize();
  EncoderInit();
  FastPWMRegisterSet();

  digitalWrite(MOTOR_STSP, HIGH);
  //analogWrite(9,100);

}

void loop() {
  // put your main code here, to run repeatedly:
  
  Serial.print("Encoder Counter : ");
  Serial.println(g_enc_pos);

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

void EncoderInit()
{
  pinMode(ENCODER_PHASE_A, INPUT);
  pinMode(ENCODER_PAHSE_B, INPUT);
  attachInterrupt(0, isrA, CHANGE);
  attachInterrupt(1, isrB, CHANGE);
}

void FastPWMRegisterSet()
{
  //TCCR1A = bit(COM1A1) | bit(COM1A0) | bit(COM1B1) | bit(COM1B0); //inverting mode => 0일때 HIGH, MAX일때 LOW
  TCCR1A = bit(COM1A1)| bit(COM1B1); //non-inverting mode => 0일때 LOW, MAX일때 HIGH
  TCCR1A |= bit(WGM11);
  TCCR1B = bit(WGM12) | bit(WGM13); // Fast PWM mode using ICR1 as TOP
  TCCR1B |= bit(CS10); // no prescaler
  ICR1 = MOTOR_FREQUENCY; //주파수 설정 => 16MHz / ICR1 = 주파수
  OCR1A = motor_duty; // 듀티 설정 => OCR1A/ICR1
  OCR1B = 0;
  TCNT1 = 0;
}


void isrA()
{
  if (digitalRead(ENCODER_PHASE_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(ENCODER_PAHSE_B) == LOW) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_PAHSE_B) == HIGH) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}

void isrB()
{
  // look for a low-to-high on channel B
  if (digitalRead(ENCODER_PAHSE_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(ENCODER_PHASE_A) == HIGH) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(ENCODER_PHASE_A) == LOW) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}
