
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


/*********************************
 * Process Setting
 *********************************/
#define RTOS_FREQUENCY                 10     // ms (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE  10
#define RTOS_FREQUENCY_MOTOROPERATION  10
#define RTOS_FREQUENCY_ENCODER_MONITOR 100

#define BAUDRATE         115200

///////////////////////////////////////////////////////////////////////////////////////

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

// Debug & Test pin
#define BOARD_LED_PIN    PC13
#define LED_PIN          PB9

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



///////////////////////////////////////////////////////////////////////////////////////



//==================================================================================================
/**
 * @note Initializing Functions
 *        - START               */
//==================================================================================================


/*********************************
* Encoder Interrupt & init pos
**********************************/
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

/*********************************
* PWM setting (Timer Register)
**********************************/
void MotorInit()
{
  Serial.print("Motor Pin & PWM Initializing...");

  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);

  digitalWrite(MOTOR_CW, HIGH);
  digitalWrite(MOTOR_CCW, LOW);
  // put your setup code here, to run once:
  

  pinMode(MOTOR_PWM_PIN, PWM);

  HardwareTimer pwmtimer1(1);  
  pwmtimer1.pause();
  pwmtimer1.setCount(0);
  pwmtimer1.setPrescaleFactor(1);  // 72/1 = 72
  pwmtimer1.setOverflow(3600 - 1); // 72mHz/3600 = 20 (kHz)
  pwmtimer1.refresh();
  pwmtimer1.resume();

  pwmWrite(MOTOR_PWM_PIN, DUTY_MIN);

  Serial.println("DONE");
}

void LoadCellInit()
{
  Serial.print("Load Cell Inititializing...");

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
    Serial.print("\n");
    Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }

  if (LoadCell_2.getTareTimeoutFlag())
  {
    Serial.print("\n");
    Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }

  if (LoadCell_3.getTareTimeoutFlag())
  {
    Serial.print("\n");
    Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }

  LoadCell_1.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
  LoadCell_2.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
  LoadCell_3.setCalFactor(CALIBRATION_VALUE); // user set calibration value (float)
  Serial.println("Done");
}

/*********************************
* RTOS Thread Initializing
**********************************/
void RTOSInit()
{
  Serial.println("RTOS Thread Creating...");

  // xTaskCreate(MotorOperation,"Task1",
  //             200,NULL,tskIDLE_PRIORITY + 2,NULL);
  // xTaskCreate(task2,"Task2",
  //             200,NULL,tskIDLE_PRIORITY + 2,NULL);
  // xTaskCreate(task3,"Task3",
  //             200,NULL,tskIDLE_PRIORITY + 2,NULL);
  // xTaskCreate(task4,"Task4",
  //             200,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(task3,"Task5",
              200,NULL,tskIDLE_PRIORITY, NULL);
  xTaskCreate(MotorOperation, "Task1", 256, NULL, 1, NULL);
  xTaskCreate(LoadCellUpdate, "Task2", 600, NULL, 2, NULL);
  xTaskCreate(EncoderMonitor, "Task3", 128, NULL, 0, NULL);

  //  BaseType_t xReturn;

  // xReturn = xTaskCreate(MotorOperation, "Task1", 128, NULL, 0, NULL);
  // if(xReturn != pdPass) Serial.println("'Motor Operation' RTOS Thread creating fail..!")

  // xReturn = xTaskCreate(LoadCellUpdate, "Task2", 256, NULL, 0, NULL);
  // if(xReturn != pdPass) Serial.println("'Load Cell Update' RTOS Thread creating fail..!")

  // xReturn = xTaskCreate(EncoderMonitor, "Task2", 64, NULL, 0, NULL);
  // if(xReturn != pdPass) Serial.println("'Encoder Monitor' RTOS Thread creating fail..!")
  
  //xReturn = xTaskCreate(LoadCellUpdate, "Task2", 40, NULL, 0, NULL);
  

//  pinMode(BOARD_LED_PIN, OUTPUT);
//  pinMode(LED_PIN, OUTPUT);
//  pinMode(PB9, OUTPUT);
//  pinMode(PC14, OUTPUT);
//
//  xTaskCreate(task1,"Task1",
//              200,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task2,"Task2",
//              200,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task3,"Task3",
//              200,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task4,"Task4",
//              200,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task5,"Task5",
//              200,NULL,tskIDLE_PRIORITY + 2,NULL);

  vTaskStartScheduler();
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

void isr_encoder_phase_A()
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

void isr_encoder_phase_B()
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

// long EEPROMReadlong(long address) {
//   long four = EEPROM.read(address);
//   long three = EEPROM.read(address + 1);
//   long two = EEPROM.read(address + 2);
//   long one = EEPROM.read(address + 3);
  
//   return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
// }

// void EEPROMWritelong(int address, long value) {
//   byte four = (value & 0xFF);
//   byte three = ((value >> 8) & 0xFF);
//   byte two = ((value >> 16) & 0xFF);
//   byte one = ((value >> 24) & 0xFF);
  
//   EEPROM.write(address, four);
//   EEPROM.write(address + 1, three);
//   EEPROM.write(address + 2, two);
//   EEPROM.write(address + 3, one);
// }
//==================================================================================================
/**
* @note EEPROM write(save) & read functions
*         - END             */
//==================================================================================================


//==================================================================================================
/**
* @note Operation Functions of RTOS Threads
*         - START             */
//==================================================================================================

/**
 * @brief EEPROMSave() is RTOS Thread function of Duration about "g_enc_pos"
 */
// void EEPROMSave(void *pvParameters)
// {
//   while(true)
//   {
//     EEPROMWritelong(ENCODER_POS_EEPROM_ADDRESS, g_enc_pos);
//     long k = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
//     Serial.println("EEPROM Saved!");

//     vTaskDelay( 100 / portTICK_PERIOD_MS ); ;
//   }
// }

/**
 * @brief Encoder count Monitoring
 * 
 * @param pvParameters 
 */
void EncoderMonitor(void *pvParameters)
{
  while(true)
  {
    Serial.println("============Encoder Monitor=================");
    Serial.print("Encoder POS : ");
    Serial.println(g_enc_pos);
    Serial.println("============================================");

    vTaskDelay( RTOS_FREQUENCY_ENCODER_MONITOR / portTICK_PERIOD_MS ); ;
  }
}


/**
 * @brief Motor Operation Thread function
 * 
 * @param pvParameters 
 */
void MotorOperation(void *pvParameters)
{
  while(true)
  {
    Serial.print("Encoder Counter : "); Serial.println(g_enc_pos);
    if(motor_op_flag == 1)
    {
       if(g_enc_pos >= TARGET_POS || g_enc_pos <= -TARGET_POS)
       {
          digitalWrite(MOTOR_CW, LOW);
          digitalWrite(MOTOR_CCW, HIGH);
          motor_op_flag = 0;
       }
       else
       {
          digitalWrite(MOTOR_CW, HIGH);
          digitalWrite(MOTOR_CCW, LOW);
          motor_op_flag = 1;
       }
    }
     vTaskDelay( RTOS_FREQUENCY_MOTOROPERATION / portTICK_PERIOD_MS );
  }
}

void LoadCellUpdate(void *pvParameters)
{
  while(true)
  {
    LoadCell_1.update();
    LoadCell_2.update();
    LoadCell_3.update();
    Serial.print("LoadCell_1 : "); Serial.print(LoadCell_1.getData());
    Serial.print(" / LoadCell_2 : "); Serial.print(LoadCell_2.getData());
    Serial.print(" / LoadCell_3 : "); Serial.print(LoadCell_3.getData());
    Serial.print("\n");
    vTaskDelay( RTOS_FREQUENCY_LOADCELLUPDATE / portTICK_PERIOD_MS );
  }
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
  Serial.begin(115200);
  Serial.print("Initializing...");
  pinMode(LED_PIN, OUTPUT);
  EncoderInit();
  MotorInit();
  LoadCellInit();
  RTOSInit();

  Serial.println("All Initializing DONE.");
}

void loop() {
  /**
   * @brief no operating source in loop()
   * @note  RTOS Thread was operating each part
   */
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                  -END-


















 
static void task1(void *pvParameters) {
  for (;;) {
      vTaskDelay(1000);
      digitalWrite(BOARD_LED_PIN, HIGH);
      vTaskDelay(1000);
      digitalWrite(BOARD_LED_PIN, LOW);
  }
}
 
static void task2(void *pvParameters) {
  for (;;) {
      vTaskDelay(200);
      digitalWrite(LED_PIN, HIGH);
      vTaskDelay(200);
      digitalWrite(LED_PIN, LOW);
  }
}

static void task3(void *pvParameters) {
  for (;;) {
      vTaskDelay(500);
      digitalWrite(PB9, HIGH);
      vTaskDelay(500);
      digitalWrite(PB9, LOW);
  }
}

static void task4(void *pvParameters) {
  for (;;) {
      vTaskDelay(1000);
      digitalWrite(PC14, HIGH);
      vTaskDelay(1000);
      digitalWrite(PC14, LOW);
  }
}

static void task5(void *pvParameters) {
  for (;;) {
    Serial.println("LOOP");
    delay(1);
  }
}
