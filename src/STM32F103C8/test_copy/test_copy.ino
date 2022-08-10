/******************************************************************************************
 * Project : gripper_ws :: main
 * 
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/gripper_ws
 * @version 0.1
 * @date 2022-08-09
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
 * MCU          : STM32F103C8
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
#include <MapleFreeRTOS821.h>
#include <HX711_ADC.h>
#include <HardwareTimer.h>


/**
 * @brief 
 */


#define BAUDRATE 115200

/*********************************
 * MCU
 *********************************/
#define CPU_CLOCK_SPEED  72    // mHz

/*********************************
 * PWM Timer 
 *********************************/
#define PRESCALER        1
#define COUNTER_PERIOD   3600
#define DUTY_MAX         3600   // same as COUNTER_PERIOD
#define DUTY_MIN         0

/*********************************
 * Pin match
 *********************************/
// Encoder pin (interrupt)
#define ENCODER_PHASE_A  PA0    
#define ENCODER_PAHSE_B  PA1

// Motor Conrol pin   
#define MOTOR_PWM_PIN    PA2      
#define MOTOR_CCW        PA3
#define MOTOR_CW         PA4

// Load Cell pin  
#define HX711_DOUT_1     PB3   //mcu > HX711 no 1 dout pin
#define HX711_SCK_1      PB4   //mcu > HX711 no 1 sck pin
#define HX711_DOUT_2     PB5   //mcu > HX711 no 2 dout pin
#define HX711_SCK_2      PB6   //mcu > HX711 no 2 sck pin
#define HX711_DOUT_3     PB7   //mcu > HX711 no 3 dout pin
#define HX711_SCK_3      PB8   //mcu > HX711 no 3 sck pin

// Default  
#define BOARD_LED_PIN    PC13
#define LED_PIN          PB11

/*********************************
 * Motor info
 *********************************/
#define MOTOR_FREQUENCY  799
#define GEAR_RATIO       289

/*********************************
 * Encoder info
 *********************************/
#define ENCODER_POS_EEPROM_ADDRESS  0
#define ENCODER_RESOLUTION          7

/*********************************
 * Encoder info
 *********************************/
#define CALIBRATION_VALUE           2072.0 // calibration value load cell 1
#define CALIBRATION_VALUE_2         2072.0 // calibration value load cell 1
#define CALIBRATION_VALUE_3         2072.0 // calibration value load cell 1

/*********************************
 * Setting info
 *********************************/
#define TARGET_RESOULTION   5
#define TARGET_POS          53200

/*********************************
 * Debug & Test
 *********************************/
#define BOARD_LED_PIN       PC13
#define LED_PIN             PB11

/*********************************
 * Manual functions
 *********************************/
void EncoderInit();
void MotorInit();
void LoadCellInit();
void RTOSInit();

void isr_encoder_phase_A();
void isr_encoder_phase_B();
// long EEPROMReadlong();
// void EEPROMWritelong();

// void EEPROMSave();

void MotorOperation();
void LoadCellUpdate();
void EncoderMonitor();

///////////////////////////////////////////////////////////////////////////////////////

/*********************************
 * Global variables
 *********************************/
static volatile long g_enc_pos = 0;
static char motor_op_flag = true;

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_DOUT_1, HX711_SCK_1); //HX711 1
HX711_ADC LoadCell_2(HX711_DOUT_2, HX711_SCK_2); //HX711 2
HX711_ADC LoadCell_3(HX711_DOUT_3, HX711_SCK_3); //HX711 3

unsigned long t = 0;

volatile float f1[3]; //[x1,y1,z1]
volatile float f2[3]; //[x2,y2,z2]
volatile float f3[3]; //[x3,y3,z3]

volatile float fx;
volatile float fy;
volatile float fz;

void setup() {
  Serial.begin(115200); delay(10);
  Serial.println();
  Serial.println("Starting...");

  EncoderInit();






  float calibrationValue_1; // calibration value load cell 1
  float calibrationValue_2; // calibration value load cell 2
  float calibrationValue_3; // calibration value load cell 3

  calibrationValue_1 = 2072.0; // uncomment this if you want to set this value in the sketch
  calibrationValue_2 = 2072.0; // uncomment this if you want to set this value in the sketch
  calibrationValue_3 = 2072.0; // uncomment this if you want to set this value in the sketch
  
#if defined(ESP8266) || defined(ESP32)
  //EEPROM.begin(512); // uncomment this if you use ESP8266 and want to fetch the value from eeprom
#endif
  //EEPROM.get(calVal_eepromAdress_1, calibrationValue_1); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_2, calibrationValue_2); // uncomment this if you want to fetch the value from eeprom
  //EEPROM.get(calVal_eepromAdress_3, calibrationValue_3); // uncomment this if you want to fetch the value from eeprom

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  //LoadCell_1.setReverseOutput();
  //LoadCell_2.setReverseOutput();
  //LoadCell_3.setReverseOutput();
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
  }
  if (LoadCell_1.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }
  if (LoadCell_2.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }
  LoadCell_1.setCalFactor(calibrationValue_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calibrationValue_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calibrationValue_3); // user set calibration value (float)
  Serial.println("Startup is complete");

  xTaskCreate(LoadCellUpdate, "Task1", 600, NULL, 2, NULL);
  vTaskStartScheduler();
}

void loop() {

}


void LoadCellUpdate(void *pvParameters)
{
  while(true)
  {
    static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();
  LoadCell_3.update();

  //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
      float f_1 = LoadCell_1.getData();
      float f_2 = LoadCell_2.getData();
      float f_3 = LoadCell_3.getData();

      f1[0] = f_1 * sin(radians(70)) * cos(radians(210));
      f1[1] = f_1 * sin(radians(70)) * sin(radians(210));  
      f1[2] = f_1 * cos(radians(20));
    
      f2[0] = f_2 * sin(radians(70)) * cos(radians(90));
      f2[1] = f_2 * sin(radians(70)) * sin(radians(90));
      f2[2] = f_2 * cos(radians(20));
    
      f3[0] = f_3 * sin(radians(70)) * cos(radians(-30));
      f3[1] = f_3 * sin(radians(70)) * sin(radians(-30));
      f3[2] = f_3 * cos(radians(20));
    
      fx = f1[0] + f2[0] + f3[0];
      fy = f1[1] + f2[1] + f3[1];
      fz = f1[2] + f2[2] + f3[2];
//      
//        Serial.print(fx);
//        Serial.print(",");
//        Serial.print(fy);
//        Serial.print(","); 
//        Serial.println(fz);
  
      Serial.print(f_1);
      Serial.print(",");
      Serial.print(f_2);
      Serial.print(",");
      Serial.println(f_3);
      newDataReady = 0;
      t = millis();
    }
  }

  // receive command from serial terminal, send 't' to initiate tare operation:
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') {
      LoadCell_1.tareNoDelay();
      LoadCell_2.tareNoDelay();
      LoadCell_3.tareNoDelay();
    }
  }

  //check if last tare operation is complete
  if (LoadCell_1.getTareStatus() == true) {
    Serial.println("Tare load cell 1 complete");
  }
  if (LoadCell_2.getTareStatus() == true) {
    Serial.println("Tare load cell 2 complete");
  }
  if (LoadCell_3.getTareStatus() == true) {
    Serial.println("Tare load cell 3 complete");
  }
  }
  
}








void EncoderInit()
{
  Serial.println("Encoder interrupter Initializing...");
  pinMode(ENCODER_PHASE_A, INPUT);
  pinMode(ENCODER_PAHSE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

  g_enc_pos = 0;
  //g_enc_pos = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
  Serial.print("DONE");
  Serial.print(" / EEPROM_pos : "); Serial.print(g_enc_pos);
  Serial.print(" / target Pos : "); Serial.println(TARGET_POS);
}


void enc_pos_monitor(void *pvParameters)
{
  Serial.print("enc pos : "); Serial.println(g_enc_pos);
}
