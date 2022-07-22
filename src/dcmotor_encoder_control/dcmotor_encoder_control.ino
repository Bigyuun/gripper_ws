/******************************************************************************************
 * Project : gripper_ws
 * 
 * @file dcmotor_encoder_control.ino
 * @git  https://github.com/Bigyuun/gripper_ws
 * @brief
 * DC motor control using encoder CODE
 * - PWM 20kHz
 * - Encoder count (interrupt)
 * 
 * - using Timer Registers for make up to 20kHz of PWM frequency
 * - counter will be increase(or decrease) 4(int) per 1 revolution
 *   ex) Suppose that motor reducer is 100, encoder resolution is 7/rev
 *       If motor shaft rotates 1 revolution, the encoder will count 2800 (100x7x4)
 *       
 ******************************************************************************************/

// define 
#define Baudrate 115200

#define POTENTIOMETER A0

// Arduino has the its own interrupt pin num#
#define ENCODER_PHASE_A 2 // pin #2,3 are only things for using interrupt
#define ENCODER_PAHSE_B 3

// Motor Conrol pin 
// #Digital pin #9,10 is for Timer Register (fix)
#define MOTOR_PWM_PIN 9
#define MOTOR_DIRECTION 11
#define MOTOR_STSP 12
#define MOTOR_FREQUENCY 799

// manual functions
void FastPWMRegisterSet();
void Initialize();
void EncoderInit();
void isrA();
void isrB();

// golbal variables
static long g_enc_pos = 0;
static int motor_duty = 399; // 50%
static int flag = 0;


void setup() {
  // put your setup code here, to run once:
  Initialize();
  EncoderInit();
  FastPWMRegisterSet();
}

void loop() {
  // put your main code here, to run repeatedly:

  if(flag == 0){
    Serial.print("Encoder Counter : ");
    Serial.println(g_enc_pos);  
  }
  
  if(g_enc_pos>=106400)
  {
    OCR1A =799;
    flag = 1;
  }
}

/**
 * @ brief  motor setup
 */
void Initialize()
{
  
  Serial.begin(Baudrate);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIRECTION, OUTPUT);
  pinMode(MOTOR_STSP, OUTPUT);

  digitalWrite(MOTOR_DIRECTION, HIGH);
  digitalWrite(MOTOR_STSP, HIGH);
  
}

/**
 * @ brief  interrupt pin setup
 */
void EncoderInit()
{
  pinMode(ENCODER_PHASE_A, INPUT);
  pinMode(ENCODER_PAHSE_B, INPUT);
  attachInterrupt(0, isrA, CHANGE);
  attachInterrupt(1, isrB, CHANGE);
}

/**
 * @brief   Timer Register setting
 * @ref     https://m.blog.naver.com/PostView.naver?isHttpsRedirect=true&blogId=ysahn2k&logNo=221330281715 
 * 
 * @comment COMnA0(1), COMnB0(1) are the register for High Low set
 *          WGM11(12,13) are the register for PWM mode type
 *          CS10 are the register for Prescaler
 *          ICR1 is the register for setting frequency
 *          OCR1A(B) are the register for setting duty
 */
void FastPWMRegisterSet()
{
  TCCR1A = bit(COM1A1) | bit(COM1A0) | bit(COM1B1) | bit(COM1B0); //inverting mode => 0일때 HIGH, MAX일때 LOW
  TCCR1A |= bit(WGM11);
  TCCR1B = bit(WGM12) | bit(WGM13); // Fast PWM mode using ICR1 as TOP
  TCCR1B |= bit(CS10); // no prescaler
  ICR1 = MOTOR_FREQUENCY; //주파수 설정 => 16MHz / ICR1 = 주파수
  OCR1A = 200; // 듀티 설정 => OCR1A/ICR1
  OCR1B = 0;
  TCNT1 = 0;
}


/**
 * @ brief  interrupt function for Encoder phase A
 */
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

  if(g_enc_pos>=106400)
  {
    OCR1A =799;
    flag = 1;
  }
}

/**
 * @ brief  interrupt function for Encoder phase B
 */
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
  if(g_enc_pos>=106400)
  {
    OCR1A =799;
    flag = 1;
  }
  
}
