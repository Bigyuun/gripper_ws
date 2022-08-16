

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
// Hardware setting
#define NUMBER_OF_LOADCELL_MODULE      3
#define NUMBER_OF_MOTOR_DRIVER         1
#define NUMBER_OF_RTOS_THREADS         4

// Utility On & OFF setting
#define ACTIVE_LOADCELL                1       // 1-ON, 0-OFF
#define ACTIVE_ENCODER                 1       // 1-ON, 0-OFF
#define ACTIVE_MOTOR                   1       // 1-ON, 0-OFF
#define ACTIVE_DATA_MONITORING         1       // 1-ON, 0-OFF
#define ACTIVE_RTOS_THREAD             1

// ROTS Threads Frequency Setting
#define RTOS_FREQUENCY                 100     // Hz (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE  500     // Hz (Default)
#define RTOS_FREQUENCY_MOTOR_OPERATION 500     // Hz (Default)
#define RTOS_FREQUENCY_EEPROM_SAVE     100     // Hz (Default)
#define RTOS_FREQUENCY_MONITORING      10      // Hz (Default)

///////////////////////////////////////////////////////////////////////////////////////

/*********************************
 * Serial communication Parameters
 *********************************/
#define BAUDRATE                       115200
#define BANDWIDTH                      10

/*********************************
 * MCU
 *********************************/
#define CPU_CLOCK_SPEED       72    // mHz

/*********************************
 * PWM Timer 
 *********************************/
#define PRESCALER             1
#define COUNTER_PERIOD        3600
#define DUTY_MAX              3600   // same as COUNTER_PERIOD
#define DUTY_MIN              0

/*********************************
 * Pin match
 *********************************/
// Encoder pin (interrupt)
#define PIN_ENCODER_PHASE_A  PA0    
#define PIN_ENCODER_PAHSE_B  PA1

// Motor Conrol pin   
#define PIN_MOTOR_PWM        PA2      
#define PIN_MOTOR_CCW        PA3
#define PIN_MOTOR_CW         PA4

// Load Cell pin  
#define HX711_DOUT_1         PB4   //mcu > HX711 no 1 dout pin
#define HX711_SCK_1          PB5   //mcu > HX711 no 1 sck pin
#define HX711_DOUT_2         PB6   //mcu > HX711 no 2 dout pin
#define HX711_SCK_2          PB7   //mcu > HX711 no 2 sck pin
#define HX711_DOUT_3         PB8   //mcu > HX711 no 3 dout pin
#define HX711_SCK_3          PB9   //mcu > HX711 no 3 sck pin

// Debug & Test pin
#define PIN_BOARD_LED        PC13
#define PIN_LED              PB9

/*********************************
 * Motor info
 *********************************/
#define GEAR_RATIO           298

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
#define TARGET_RESOULTION   3.814814814
#define TARGET_POS          (GEAR_RATIO * ENCODER_RESOLUTION * 4 * TARGET_RESOULTION)
// #define TARGET_POS          10


/*********************************
 * Manual functions
 *********************************/
uint8_t EncoderInit();
uint8_t MotorInit();
uint8_t LoadCellInit();
uint8_t RTOSInit();

void isr_encoder_phase_A();
void isr_encoder_phase_B();
long EEPROMReadlong();
void EEPROMWritelong();

// RTOS Thread Functions
void EEPROMSaveNode();
void MotorOperation();
void LoadCellUpdateNode();
void MonitorAllParametersNode();
void SerialCommunicationNode();
static void LEDIndicator();

///////////////////////////////////////////////////////////////////////////////////////

/*********************************
 * Global variables
 *********************************/
static volatile long g_enc_pos = 0;
static char motor_op_flag = true;

#if ACTIVE_LOADCELL
//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_DOUT_1, HX711_SCK_1); //HX711 1
HX711_ADC LoadCell_2(HX711_DOUT_2, HX711_SCK_2); //HX711 2
HX711_ADC LoadCell_3(HX711_DOUT_3, HX711_SCK_3); //HX711 3
#endif

static float g_loadcell[NUMBER_OF_LOADCELL_MODULE] = {0};

unsigned long t = 0;

volatile float f1[3]; //[x1,y1,z1]
volatile float f2[3]; //[x2,y2,z2]
volatile float f3[3]; //[x3,y3,z3]

volatile float fx;
volatile float fy;
volatile float fz;

// monitoring val
static float loop_time_checker[NUMBER_OF_RTOS_THREADS] = {-1};


///////////////////////////////////////////////////////////////////////////////////////



//==================================================================================================
/**
 * @note Encoder Initialize & operate Functions
 *        - START               */
//==================================================================================================

/*********************************
* Encoder Interrupt & init pos
**********************************/
#if ACTIVE_ENCODER
uint8_t EncoderInit()
{
  Serial.print("Encoder interrupter Initializing...");
  pinMode(PIN_ENCODER_PHASE_A, INPUT);
  pinMode(PIN_ENCODER_PAHSE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

  g_enc_pos = 0;
  //g_enc_pos = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
  delay(100);
  Serial.print("DONE");
  Serial.print(" --> EEPROM_pos : "); Serial.print(g_enc_pos);
  Serial.print(" / target Pos : "); Serial.println(TARGET_POS);
  return 1;
}

/*********************************
* Encoder Interrupt Phase A
**********************************/
void isr_encoder_phase_A()
{
  if (digitalRead(PIN_ENCODER_PHASE_A) == HIGH) { 
    // check channel B to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PAHSE_B) == LOW) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(PIN_ENCODER_PAHSE_B) == HIGH) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}
/*********************************
* Encoder Interrupt Phase B
**********************************/
void isr_encoder_phase_B()
{
  // look for a low-to-high on channel B
  if (digitalRead(PIN_ENCODER_PAHSE_B) == HIGH) {   
   // check channel A to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PHASE_A) == HIGH) {  
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
  // Look for a high-to-low on channel B
  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(PIN_ENCODER_PHASE_A) == LOW) {   
      g_enc_pos += 1;
    } 
    else {
      g_enc_pos -= 1;
    }
  }
}
#endif
//==================================================================================================
/**
 * @note Encoder Initialize & operate Functions
 *        - END               */
//==================================================================================================


//==================================================================================================
/**
 * @note Motor Initialize & operate Functions
 *        - START               */
//==================================================================================================

/*********************************
* PWM setting (Timer Register)
**********************************/
#if ACTIVE_MOTOR
uint8_t MotorInit()
{
  Serial.print("Motor Pin & PWM Initializing...");

  pinMode(PIN_MOTOR_CW, OUTPUT);
  pinMode(PIN_MOTOR_CCW, OUTPUT);
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, LOW);  

  pinMode(PIN_MOTOR_PWM, PWM);
  pwmWrite(PIN_MOTOR_PWM, DUTY_MIN);

  HardwareTimer pwmtimer2(2);  
  pwmtimer2.pause();
  pwmtimer2.setCount(0);
  pwmtimer2.setPrescaleFactor(1);  // 72/1 = 72
  pwmtimer2.setOverflow(3600 - 1); // 72mHz/3600 = 20 (kHz)
  pwmtimer2.refresh();
  pwmtimer2.resume();



  delay(100);
  Serial.println("DONE");
  return 1;
}
#endif

/*********************************
* Motor Operating Function
**********************************/
void MotorOperation(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    if (g_enc_pos >= TARGET_POS || g_enc_pos <= -TARGET_POS)
    {
      // digitalWrite(MOTOR_CW, LOW);
      // digitalWrite(MOTOR_CCW, HIGH);

      digitalWrite(PIN_MOTOR_CW, LOW);
      digitalWrite(PIN_MOTOR_CCW, LOW);

      pwmWrite(PIN_MOTOR_PWM, DUTY_MIN);
      motor_op_flag = 0;
    }
    else
    {
      // digitalWrite(PIN_MOTOR_CW, LOW);
      // digitalWrite(PIN_MOTOR_CCW, HIGH);

      digitalWrite(PIN_MOTOR_CW, HIGH);
      digitalWrite(PIN_MOTOR_CCW, LOW);
      pwmWrite(PIN_MOTOR_PWM, (int)(DUTY_MAX / 5));
      motor_op_flag = 1;
    }

    loop_time_checker[2] = temp_time;
    // vTaskDelay( (1000/RTOS_FREQUENCY_MOTOR_OPERATION - (int)temp_time) / (portTICK_PERIOD_MS)  );
    vTaskDelay( (1000/RTOS_FREQUENCY_MOTOR_OPERATION) / (portTICK_PERIOD_MS)  );
  }
}

/*********************************
* Motor Home position
**********************************/
uint8_t MotorHoming()
{
  if(g_enc_pos >= 100)
  {
    digitalWrite(PIN_MOTOR_CW, LOW);
    digitalWrite(PIN_MOTOR_CCW, HIGH);
    pwmWrite(PIN_MOTOR_PWM, (int)(DUTY_MAX / 10));
  }
  else if(g_enc_pos <= -100)
  {
    digitalWrite(PIN_MOTOR_CW, HIGH);
    digitalWrite(PIN_MOTOR_CCW, LOW);
    pwmWrite(PIN_MOTOR_PWM, (int)(DUTY_MAX / 10));
  }
  else
  {
    digitalWrite(PIN_MOTOR_CW, LOW);
    digitalWrite(PIN_MOTOR_CCW, LOW);
    pwmWrite(PIN_MOTOR_PWM, DUTY_MIN);
  }
}




//==================================================================================================
/**
 * @note Motor Initialize & operate Functions
 *        - END               */
//==================================================================================================


//==================================================================================================
/**
 * @note Load Cell Initialize & operate Functions
 *        - START               */
//==================================================================================================
#if ACTIVE_LOADCELL
uint8_t LoadCellInit()
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

  delay(100);
  Serial.println("Done");
  return 1;
}

/*********************************
* Load Cell data Update
**********************************/
void LoadCellUpdateNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    LoadCell_1.update();
    LoadCell_2.update();
    LoadCell_3.update();
    g_loadcell[0] = LoadCell_1.getData();
    g_loadcell[1] = LoadCell_2.getData();
    g_loadcell[2] = LoadCell_3.getData();

    loop_time_checker[0] = temp_time;
    // vTaskDelay( (1000/RTOS_FREQUENCY_LOADCELLUPDATE - (int)temp_time) / portTICK_PERIOD_MS );
    vTaskDelay( (1000/RTOS_FREQUENCY_LOADCELLUPDATE) / portTICK_PERIOD_MS );
  }
}
#endif
//==================================================================================================
/**
 * @note Load Cell Initialize & operate Functions
 *        - END               */
//==================================================================================================


//==================================================================================================
/**
* @note EEPROM write(save) & read functions
*         - START             */
//==================================================================================================

/*********************************
* EEPROM read
**********************************/
long EEPROMReadlong(int address) {
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);
  
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/*********************************
* EEPROM write
**********************************/
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

/*********************************
* EEPROM Update (g_enc_pos update)
**********************************/
void EEPROMSaveNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    EEPROMWritelong(ENCODER_POS_EEPROM_ADDRESS, g_enc_pos);
    // long k = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);

    loop_time_checker[1] = temp_time;
    // vTaskDelay( (1000/RTOS_FREQUENCY_EEPROM_SAVE - (int)temp_time) / portTICK_PERIOD_MS ); ;
    vTaskDelay( (1000/RTOS_FREQUENCY_EEPROM_SAVE) / portTICK_PERIOD_MS ); ;
  }
}
//==================================================================================================
/**
* @note EEPROM write(save) & read functions
*         - END             */
//==================================================================================================







/*********************************
* Monitoring Data & loop time of each Threads
**********************************/
#if ACTIVE_DATA_MONITORING
void MonitorAllParametersNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while(true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    Serial.println("=====================================================");
    Serial.println("-------------------- <Data> -------------------------");
    Serial.println("LOADCELL_1    LOADCELL_2    LOADCELL_3    ENC_POS");
    for(int i=0; i<NUMBER_OF_LOADCELL_MODULE; i++)
    {
    Serial.print  (g_loadcell[i], 4); Serial.print  ("        ");
    }
    Serial.print  ("   ");
    Serial.print  (g_enc_pos);
    Serial.print  ("\n");
    Serial.println("----------------- <Loop Time> -----------------------");
    Serial.println("Thread 1      Thread 2      Thread 3      Thread 4");
    for(int i=0; i<NUMBER_OF_RTOS_THREADS; i++)
    {
    Serial.print  (loop_time_checker[i], 0); Serial.print(" ms          ");
    }
    Serial.print  ("\n");

    vTaskDelay( (1000/RTOS_FREQUENCY_MONITORING) / portTICK_PERIOD_MS );
  }
}
#endif

/*********************************
* Serial Communication Update
**********************************/
void SerialCommunicationNode(void *pvParameters)
{

}

/*********************************
* Indicator for operating successfully
**********************************/
static void LEDIndicator(void *pvParameters) {
  for (;;) {
      vTaskDelay(500);
      digitalWrite(PIN_BOARD_LED, HIGH);
      vTaskDelay(500);
      digitalWrite(PIN_BOARD_LED, LOW);
  }
}

//==================================================================================================
/**
* @note Operation of RTOS Threads
*         - END             */
//==================================================================================================


//==================================================================================================
/**
 * @note RTOS Threads Initialize Functions
 *        - START               */
//==================================================================================================

/*********************************
* RTOS Thread Initializing
**********************************/
#if ACTIVE_RTOS_THREAD
uint8_t RTOSInit()
{
  Serial.print("RTOS Thread Creating...");

  #if ACTIVE_MOTOR
  xTaskCreate(MotorOperation,
              "MotorOperation",
              256,
              NULL,
              tskIDLE_PRIORITY+2,
              NULL);
  #endif

  #if ACTIVE_LOADCELL
  xTaskCreate(LoadCellUpdateNode,
              "LoadCellUpdate",
              768,
              NULL,
              tskIDLE_PRIORITY+2,
              NULL);
  #endif

  #if ACTIVE_DATA_MONITORING

  xTaskCreate(MonitorAllParametersNode,
              "MonitorAllParameters",
              128,
              NULL,
              tskIDLE_PRIORITY+1,
              NULL);
  #endif

  xTaskCreate(EEPROMSaveNode,
              "EEPROMSave",
              128,
              NULL,
              tskIDLE_PRIORITY+1,
              NULL);

  xTaskCreate(LEDIndicator,
              "LEDIndicator_Debug",
              64,
              NULL,
              tskIDLE_PRIORITY,
              NULL);
  Serial.println("Done");

  // if vTaskStartScheduler doen't work, under line will be operated.
  vTaskStartScheduler();
  Serial.println("vTaskStartScheduler() failed!!!");
  
  return 0;
}
#endif
//==================================================================================================
/**
 * @note RTOS Threads Initialize Functions
 *        - END               */
//==================================================================================================


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                 -START-

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(115200);
  Serial.println("Initializing...");
  //Serial3.begin(115200);
  //Serial3.print("asdfasdfasdfasdfasdfasdfasdf");
  pinMode(PIN_BOARD_LED, OUTPUT);
  for(int i=0; i<NUMBER_OF_RTOS_THREADS; i++)
  {
  loop_time_checker[i] = -1;
  }

  if(ACTIVE_ENCODER)     EncoderInit();
  if(ACTIVE_MOTOR)       MotorInit();
  if(ACTIVE_LOADCELL)    LoadCellInit();
  if(ACTIVE_RTOS_THREAD) RTOSInit();

  Serial.println("All Initializing DONE.");
  delay(2000);
}

void loop() {
  /**
   * @brief no operating source in loop()
   * @note  RTOS Thread was operating each part
   */
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                  -END-