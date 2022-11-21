
/******************************************************************************************
 * Project : gripper_ws :: main
 * -Git test.. 2022.11.21. -- global set
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/gripper_ws
 * @version 0.1
 * @date 2022-08-03
 * @copyright Copyright (c) 2022
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
 * @note 
 * Motor Driver : DMC-11
 * MCU          : Arduino Uno
******************************************************************************************/

/***
 * @name : EEPROM.h
 * @brief :
  Iterate through each byte of the EEPROM storage.
  Larger AVR processors have larger EEPROM sizes, E.g:
  - Arduino Duemilanove: 512 B EEPROM storage.
  - Arduino Uno:         1 kB EEPROM storage.
  - Arduino Mega:        4 kB EEPROM storage.
  Rather than hard-coding the length, you should use the pre-provided length function.
  This will make your code portable to all AVR processors.
***/
#include <EEPROM.h>
#include <Arduino_FreeRTOS.h>
#include <HX711_ADC.h>

/**
 * @brief 
 * Header for modifying Timer Register
 */
// #include <Arduino.h>
// #include <io2333.h>
// #include <io4434.h>
// #include <iotnx61.h>
// #include <iotnx4.h>

// #define configMINIMAL_STACK_SIZE ( ( portSTACK_TYPE ) 192 )
#define Baudrate 115200
/*********************************
 * Pin match
 *********************************/
// Arduino has the its own interrupt pin num#
#define ENCODER_PHASE_A 2    // pin #2,3 are only things for using interrupt
#define ENCODER_PAHSE_B 3

// Motor Conrol pin 
#define MOTOR_PWM_PIN 9      // #Digital pin #9,10 is for Timer Register (fix)
#define MOTOR_CCW 4
#define MOTOR_CW 5
// #define MOTOR_DIRECTION 4
// #define MOTOR_STSP 5

/*********************************
 * Motor info
 *********************************/
#define MOTOR_FREQUENCY 799
#define GEAR_RATIO 290

/*********************************
 * Encoder info
 *********************************/
#define ENCODER_POS_EEPROM_ADDRESS 0
#define ENCODER_RESOLUTION 7

/*********************************
 * Setting info
 *********************************/
#define TARGET_RESOULTION 5
//#define TARGET_POS (TARGET_RESOULTION*GEAR_RATIO*ENCODER_RESOLUTION*4)
#define TARGET_POS 53200

/*********************************
 * Manual functions
 *********************************/
void FastPWMRegisterSet();
void Initialize();
void EncoderInit();
void isrA();
void isrB();
long EEPROMReadlong();
void EEPROMWritelong();

void RTOSInit();

void MotorOperation();
void EEPROMSave();
void LoadCellUpdate();
/*********************************
 * Global variables
 *********************************/
static long g_enc_pos = 0;
const int motor_enable = 300; // 50%
const int motor_disable = 0;  // 0$
static char motor_op_flag = true;


///////////////////////////////////////////////////////////////////////////////////////



//pins:
const char HX711_dout_1 = 6; //mcu > HX711 no 1 dout pin
const char HX711_sck_1 = 7; //mcu > HX711 no 1 sck pin
const char HX711_dout_2 = 10; //mcu > HX711 no 2 dout pin
const char HX711_sck_2 = 11; //mcu > HX711 no 2 sck pin
const char HX711_dout_3 = 12; //mcu > HX711 no 3 dout pin
const char HX711_sck_3 = 13; //mcu > HX711 no 3 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3

unsigned long t = 0;

//volatile float f1[3]; //[x1,y1,z1]
//volatile float f2[3]; //[x2,y2,z2]
//volatile float f3[3]; //[x3,y3,z3]
//
//volatile float fx;
//volatile float fy;
//volatile float fz;



///////////////////////////////////////////////////////////////////////////////////////



//==================================================================================================
/**
 * @note Initializing Functions
 *        - START               */
//==================================================================================================

/*********************************
* GPIO setting
**********************************/
void Initialize()
{
  
  Serial.begin(Baudrate);

  Serial.print("Initializing...");
  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);

  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
  Serial.println(" DONE");
  //delay(500);
}

/*********************************
* Encoder Interrupt & init pos
**********************************/
void EncoderInit()
{
  Serial.print("Encoder interrupter Initializing...");
  pinMode(ENCODER_PHASE_A, INPUT);
  pinMode(ENCODER_PAHSE_B, INPUT);
  attachInterrupt(0, isrA, CHANGE);
  attachInterrupt(1, isrB, CHANGE);
  Serial.println(" DONE");
  
  g_enc_pos = 0;
  //g_enc_pos = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);

  Serial.print("EEPROM_pos : "); Serial.println(g_enc_pos);
  Serial.print("target Pos : "); Serial.println(TARGET_POS);
  //delay(500);
}

/*********************************
* PWM setting (Timer Register)
**********************************/
void FastPWMRegisterSet()
{
  Serial.print("TimerRegister Initializing...");
  //TCCR1A = bit(COM1A1) | bit(COM1A0) | bit(COM1B1) | bit(COM1B0); //inverting mode => 0일때 HIGH, MAX일때 LOW
  TCCR1A = bit(COM1A1)| bit(COM1B1); //non-inverting mode => 0일때 LOW, MAX일때 HIGH
  TCCR1A |= bit(WGM11);
  TCCR1B = bit(WGM12) | bit(WGM13); // Fast PWM mode using ICR1 as TOP
  TCCR1B |= bit(CS10); // no prescaler
  ICR1 = MOTOR_FREQUENCY; //주파수 설정 => 16MHz / ICR1 = 주파수
  OCR1A = motor_disable; // 듀티 설정 => OCR1A/ICR1
  OCR1B = motor_disable;
  TCNT1 = 0;
  Serial.println(" DONE");
  //delay(500);
}

void LoadCellInit()
{
  Serial.println("Starting...");

  float calibrationValue_1 = 2072.0; // calibration value load cell 1
  float calibrationValue_2 = 2072.0; // calibration value load cell 1
  float calibrationValue_3 = 2072.0; // calibration value load cell 3

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();

  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true;                 // set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;

  while ((loadcell_1_rdy + loadcell_2_rdy) < 2)
  { // run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy)
      loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy)
      loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy)
      loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }

  if (LoadCell_1.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }

  if (LoadCell_2.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }

  if (LoadCell_3.getTareTimeoutFlag())
  {
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }

  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
  Serial.println("Startup is complete");
}

/*********************************
* RTOS Thread Initializing
**********************************/
void RTOSInit()
{
  Serial.println("RTOS Thread Creating...");
  // xTaskCreate(MotorOperation, "Task1", 128, NULL, 0, NULL);
  xTaskCreate(EEPROMSave, "Task2", 40, NULL, 0, NULL);

  //vTaskStartScheduler();
  Serial.println("RTOS Thread Created!");
}

//==================================================================================================
/**
 * @note Initializing Functions
 *        - END               */
//==================================================================================================


//==================================================================================================
/**
* @note Functions - Encoder count : (g_enc_pos)
*         - START             */
//==================================================================================================

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
//==================================================================================================
/**
* @note Functions - Encoder count : (g_enc_pos)
*         - END             */
//==================================================================================================


//==================================================================================================
/**
* @note EEPROM write(save) & read functions
*         - START             */
//==================================================================================================

long EEPROMReadlong(long address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}
//==================================================================================================
/**
* @note EEPROM write(save) & read functions
*         - END             */
//==================================================================================================


//==================================================================================================
/**
* @note Operation of RTOS Threads
*         - START             */
//==================================================================================================

/**
 * @brief EEPROMSave() is RTOS Thread function of Duration about "g_enc_pos"
 */
void EEPROMSave(void *pvParameters)
{
  while(true)
  {
    EEPROMWritelong(ENCODER_POS_EEPROM_ADDRESS, g_enc_pos);
    long k = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
    Serial.println("EEPROM Saved!");

    vTaskDelay( 100 / portTICK_PERIOD_MS ); ;
  }
}

/**
 * @brief Motor Operation Thread function
 * 
 * @param pvParameters 
 */
void MotorOperation()
{
  // static unsigned int count = 0;
  // while(true)
  {
    Serial.print("Encoder Counter : "); Serial.println(g_enc_pos);
    if(motor_op_flag == 1)
    {
       if(g_enc_pos >= TARGET_POS || g_enc_pos <= -TARGET_POS)
       {
          digitalWrite(MOTOR_CW, LOW);
          digitalWrite(MOTOR_CCW, HIGH);
          OCR1A = motor_disable;
          motor_op_flag = 0;
          // Serial.print("Current Enc POS"); Serial.println(g_enc_pos);
       }
       else
       {
          digitalWrite(MOTOR_CW, HIGH);
          digitalWrite(MOTOR_CCW, LOW);
          OCR1A = motor_enable;  
          motor_op_flag = 1;
       }
    }
    // Serial.print("count is : "); Serial.println(count);
    // count ++;
    // vTaskDelay( 10 / portTICK_PERIOD_MS ); ;

  }
}

void LoadCellUpdate()
{
  LoadCell_1.update();
  LoadCell_2.update();
  LoadCell_3.update();
  Serial.print("LoadCell_1 : "); Serial.print(LoadCell_1.getData());
  Serial.print(" / LoadCell_2 : "); Serial.print(LoadCell_2.getData());
  Serial.print(" / LoadCell_3 : "); Serial.print(LoadCell_3.getData());
  Serial.print("\n");
  
  // vTaskDelay( 10 / portTICK_PERIOD_MS );
}

//==================================================================================================
/**
* @note Operation of RTOS Threads
*         - END             */
//==================================================================================================




//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                 -START-

void setup() {
  // put your setup code here, to run once:
  Initialize();
  EncoderInit();
  FastPWMRegisterSet();
  LoadCellInit();
  RTOSInit();
}

void loop() {
  /**
   * @brief no operating source in loop()
   * @note  RTOS Thread was operating each part
   */
  // Serial.print("Encoder Counter : ");
  // Serial.println(g_enc_pos);
  MotorOperation();
  LoadCellUpdate();

  // Serial.print("loop() stack : "); Serial.println(uxTaskGetStackHighWaterMark(0));
  //delay(200);

}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                  -END-
