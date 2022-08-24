

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

/*********************************
 * Process Setting
 *********************************/
// Hardware setting
#define NUMBER_OF_LOADCELL_MODULE       3
#define NUMBER_OF_MOTOR_DRIVER          1
#define NUMBER_OF_RTOS_THREADS          4

// Utility On & OFF setting
#define ACTIVE_LOADCELL                 1       // 1-ON, 0-OFF
#define ACTIVE_MOTOR                    1       // 1-ON, 0-OFF
#define ACTIVE_DATA_MONITORING          1       // 1-ON, 0-OFF
#define ACTIVE_RTOS_THREAD              1

// ROTS Threads Frequency Setting
#define RTOS_FREQUENCY                  100     // Hz (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE   500     // Hz (Default)
#define RTOS_FREQUENCY_MOTOR_OPERATION  500     // Hz (Default)
#define RTOS_FREQUENCY_EEPROM_SAVE      100     // Hz (Default)
#define RTOS_FREQUENCY_MONITORING       10      // Hz (Default)

///////////////////////////////////////////////////////////////////////////////////////

/*********************************
 * Serial communication Parameters
 *********************************/
#define BAUDRATE                115200
#define BANDWIDTH               10

/*********************************
 * MCU
 *********************************/
#define CPU_CLOCK_SPEED         72 // mHz

/*********************************
 * PWM Timer
 *********************************/
#define PRESCALER               1
#define COUNTER_PERIOD          3600
#define DUTY_MAX                COUNTER_PERIOD         // same as COUNTER_PERIOD
#define DUTY_MIN                0

/*********************************
 * Pin match
 *********************************/
// Encoder pin (interrupt)
#define PIN_ENCODER_PHASE_A     PA0
#define PIN_ENCODER_PAHSE_B     PA1

// Motor Conrol pin
#define PIN_MOTOR_PWM           PA2
#define PIN_MOTOR_CCW           PA3
#define PIN_MOTOR_CW            PA4

// Load Cell pin
#define HX711_DOUT_1            PB4   // mcu > HX711 no 1 dout pin
#define HX711_SCK_1             PB5   // mcu > HX711 no 1 sck pin
#define HX711_DOUT_2            PB6   // mcu > HX711 no 2 dout pin
#define HX711_SCK_2             PB7   // mcu > HX711 no 2 sck pin
#define HX711_DOUT_3            PB8   // mcu > HX711 no 3 dout pin
#define HX711_SCK_3             PB9   // mcu > HX711 no 3 sck pin

// Debug & Test pin
#define PIN_BOARD_LED           PC13
#define PIN_LED                 PB10

/*********************************
 * Motor info
 *********************************/
#define GEAR_RATIO              298
#define GRIPPER_GEAR_RATIO      3.814814814

/*********************************
 * Encoder info
 *********************************/
#define ENCODER_POS_EEPROM_ADDRESS 0
#define ENCODER_RESOLUTION         7
#define HOMING_THRESHOLD           200

/*********************************
 * Load Cell info
 *********************************/
// #define CALIBRATION_VALUE           2072.0 // calibration value load cell 1
// #define CALIBRATION_VALUE_2         2072.0 // calibration value load cell 2
// #define CALIBRATION_VALUE_3         2072.0 // calibration value load cell 3

#define CALIBRATION_VALUE_1       2935.8   // calibration value load cell 1
#define CALIBRATION_VALUE_2       2987.0   // calibration value load cell 2
#define CALIBRATION_VALUE_3       3072.0   // calibration value load cell 3
#define LOADCELL_HOMING_VALUE     150

/*********************************
 * Setting info
 *********************************/
#define TARGET_RESOLTION          1
#define TARGET_POS                (GEAR_RATIO * ENCODER_RESOLUTION * 4 * GRIPPER_GEAR_RATIO * TARGET_RESOLTION)

#define ONE_RESOLUTION            (GEAR_RATIO * ENCODER_RESOLUTION * 4 * GRIPPER_GEAR_RATIO)

/*********************************
 * Manual functions
 *********************************/

uint8_t LoadCellInit();
uint8_t RTOSInit();

long EEPROMReadlong();
void EEPROMWritelong();

void isr_encoder_phase_A();
void isr_encoder_phase_B();
void EncoderInit();

// RTOS Thread Functions
void EEPROMSaveNode();
void LoadCellUpdateNode();
void MonitorAllParametersNode();
void SerialCommunicationNode();
static void LEDIndicator();

///////////////////////////////////////////////////////////////////////////////////////

enum MotorDirection
{
  kClockwise,
  kCounterClockwise,
};

enum MotorState
{
  kEnable,
  kDisable,
};

enum MotorOperationMode
{
  kVelocityMode,
  kPositionMode,
};

enum MotorCommand
{
  kNone,
  kHoming = 1,
  kHomingLoadcell,
  kOneResolution,
};

#if ACTIVE_MOTOR
class DCMotor
{
private:
  
  static DCMotor* anchor;

public:

  MotorState motor_state = kDisable;
  MotorDirection direction = kClockwise;
  MotorOperationMode op_mode = kVelocityMode;
  MotorCommand op_command = kNone;

  long actual_position = 0;
  long absolute_position = 0;
  long difference_position = 0;
  double target_velocity = 0;
  long target_position = 0;

public:
  DCMotor();
  ~DCMotor();

  void Enable();
  void Disable();
  void SetDirCW();
  void SetDirCCW();
  void SetDirOFF();
  void UpdateVelocity(int duty);
  void UpdatePosition(long val);
  void PWMInit();
  uint8_t Homing(uint16_t threshold);
  uint8_t HomingLoadCell(float target_force);
  uint8_t MoveOneResolution();
};
#endif

/*********************************
 * Global variables
 *********************************/
#if ACTIVE_MOTOR
DCMotor GripperMotor;
#endif


#if ACTIVE_LOADCELL
// HX711_ADC LoadCell[NUMBER_OF_LOADCELL_MODULE];
// LoadCell[0] = new HX711_ADC(HX711_DOUT_1, HX711_SCK_1);
// LoadCell[1] = new HX711_ADC(HX711_DOUT_2, HX711_SCK_2);
// LoadCell[2] = new HX711_ADC(HX711_DOUT_3, HX711_SCK_3);

// HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_DOUT_1, HX711_SCK_1); // HX711 1
HX711_ADC LoadCell_2(HX711_DOUT_2, HX711_SCK_2); // HX711 2
HX711_ADC LoadCell_3(HX711_DOUT_3, HX711_SCK_3); // HX711 3
static float g_loadcell_val[NUMBER_OF_LOADCELL_MODULE] = {0};

unsigned long t = 0;
volatile float f1[3]; //[x1,y1,z1]
volatile float f2[3]; //[x2,y2,z2]
volatile float f3[3]; //[x3,y3,z3]
volatile float fx;
volatile float fy;
volatile float fz;

#endif

#if ACTIVE_DATA_MONITORING
// monitoring val
static float loop_time_checker[NUMBER_OF_RTOS_THREADS] = {0};
static bool allparameters_monitoring_flag = false;
#endif
///////////////////////////////////////////////////////////////////////////////////////

//==================================================================================================
/**
 * @note Motor Initialize & operate Functions
 *        - END               */
//==================================================================================================

/*********************************
 * Encoder Interrupt Phase A
 **********************************/
void isr_encoder_phase_A()
{
  if (digitalRead(PIN_ENCODER_PHASE_A) == HIGH)
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PAHSE_B) == LOW)
    {
      GripperMotor.absolute_position += 1;
      GripperMotor.actual_position += 1;
    }
    else
    {
      GripperMotor.absolute_position -= 1;
      GripperMotor.actual_position -= 1;
    }
  }
  else // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PAHSE_B) == HIGH)
    {
      GripperMotor.absolute_position += 1;
      GripperMotor.actual_position += 1;
    }
    else
    {
      GripperMotor.absolute_position -= 1;
      GripperMotor.actual_position -= 1;
    }
  }
}

/*********************************
 * Encoder Interrupt Phase B
 **********************************/
void isr_encoder_phase_B()
{
  // look for a low-to-high on channel B
  if (digitalRead(PIN_ENCODER_PAHSE_B) == HIGH)
  {
    // check channel A to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PHASE_A) == HIGH)
    {
      GripperMotor.absolute_position += 1;
      GripperMotor.actual_position += 1;
    }
    else
    {
      GripperMotor.absolute_position -= 1;
      GripperMotor.actual_position -= 1;
    }
  }
  // Look for a high-to-low on channel B
  else
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(PIN_ENCODER_PHASE_A) == LOW)
    {
      GripperMotor.absolute_position += 1;
      GripperMotor.actual_position += 1;
    }
    else
    {
      GripperMotor.absolute_position -= 1;
      GripperMotor.actual_position -= 1;
    }
  }
}

/*********************************
 * Encoder Interrupt & init pos
 **********************************/
void EncoderInit()
{
  Serial.print("Encoder interrupter Initializing...");
  pinMode(PIN_ENCODER_PHASE_A, INPUT);
  pinMode(PIN_ENCODER_PAHSE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

  GripperMotor.absolute_position = 0;
  GripperMotor.absolute_position = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
  // GripperMotor.actual_position = 0;
  // GripperMotor.actual_position = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);

  delay(100);
  Serial.print("DONE");
  Serial.print(" --> EEPROM_pos : ");
  Serial.print(GripperMotor.absolute_position);
  Serial.print(" / target Pos : ");
  Serial.println(TARGET_POS);
}


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

  LoadCell_1.setCalFactor(CALIBRATION_VALUE_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(CALIBRATION_VALUE_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(CALIBRATION_VALUE_3); // user set calibration value (float)

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

  while (true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    LoadCell_1.update();
    LoadCell_2.update();
    LoadCell_3.update();
    g_loadcell_val[0] = LoadCell_1.getData();
    g_loadcell_val[1] = LoadCell_2.getData();
    g_loadcell_val[2] = LoadCell_3.getData();

    loop_time_checker[0] = temp_time;
    // vTaskDelay( (1000/RTOS_FREQUENCY_LOADCELLUPDATE - (int)temp_time) / portTICK_PERIOD_MS );
    vTaskDelay((1000 / RTOS_FREQUENCY_LOADCELLUPDATE) / portTICK_PERIOD_MS);
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
long EEPROMReadlong(int address)
{
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/*********************************
 * EEPROM write
 **********************************/
void EEPROMWritelong(int address, long value)
{
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

  while (true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    EEPROMWritelong(ENCODER_POS_EEPROM_ADDRESS, GripperMotor.absolute_position);
    // long k = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);

    loop_time_checker[1] = temp_time;
    // vTaskDelay( (1000/RTOS_FREQUENCY_EEPROM_SAVE - (int)temp_time) / portTICK_PERIOD_MS ); ;
    vTaskDelay((1000 / RTOS_FREQUENCY_EEPROM_SAVE) / portTICK_PERIOD_MS);
    ;
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

  while (true)
  {
    // curt_time = millis();
    // temp_time = curt_time - prev_time;
    // prev_time = curt_time;
    if (allparameters_monitoring_flag)
    {
      Serial.println("=====================================================");
      Serial.println("-------------------- <Data> -------------------------");
      Serial.println("LOADCELL_1    LOADCELL_2    LOADCELL_3    ENC_POS(abs)    ENC_POS(acual)");
      for (int i = 0; i < NUMBER_OF_LOADCELL_MODULE; i++)
      {
        Serial.print(g_loadcell_val[i], 4);
        Serial.print("        ");
      }
      Serial.print("   ");
      Serial.print(GripperMotor.absolute_position);
      Serial.print("   ");
      Serial.print(GripperMotor.actual_position);
      Serial.print("\n");
      Serial.println("----------------- <Loop Time> -----------------------");
      Serial.println("Thread 1      Thread 2      Thread 3      Thread 4");
      for (int i = 0; i < NUMBER_OF_RTOS_THREADS; i++)
      {
        Serial.print(loop_time_checker[i], 0);
        Serial.print(" ms          ");
      }
      Serial.print("\n");
    }

    vTaskDelay((1000 / RTOS_FREQUENCY_MONITORING) / portTICK_PERIOD_MS);
  }
}
#endif

/*********************************
 * Serial Communication Update
 **********************************/
void SerialCommunicationReadingNode(void *pvParameters)
{
  Serial.println("================================ Command Info ======================================");
  Serial.println("c : clockwise   / d : counter clockwise / z : enc_pos = 0 / m : monitoring parameters");
  Serial.println("r : run 1 cycle / s : stop motor                          / n : Not monitoring parameters");
  Serial.println("====================================================================================");

  while (true)
  {
    if (Serial.available())
    {
      char str_command = Serial.read();
      // String str_command = Serial.readString();
      Serial.println(str_command);

      if (str_command == 'c')
      {
        GripperMotor.SetDirCW();
      }
      else if (str_command == 'd')
      {
        GripperMotor.SetDirCCW();
      }
      else if (str_command == 'z')  GripperMotor.actual_position = 0;
      else if (str_command == 'r')
      {
        GripperMotor.Enable();
        GripperMotor.UpdateVelocity(DUTY_MAX/5);
      }
      else if (str_command == 's')
      {
        GripperMotor.Disable();
      }
      else if (str_command == 'h')  GripperMotor.op_command = kHoming;
      else if (str_command == 'l')  GripperMotor.op_command = kHomingLoadcell;

      else if (str_command == 'm')  allparameters_monitoring_flag = true;
      else if (str_command == 'n')  allparameters_monitoring_flag = false;
    }

    // vTaskDelay/((10) / portTICK_PERIOD_MS);
  }
}

void SerialCommunicationWritingNode(void *pvParameters)
{
  // for echo system
}

/*********************************
 * Indicator for operating successfully
 **********************************/
static void LEDIndicator(void *pvParameters)
{
  for (;;)
  {
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

#if ACTIVE_LOADCELL
  xTaskCreate(LoadCellUpdateNode,
              "LoadCellUpdate",
              384,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
#endif

#if ACTIVE_DATA_MONITORING
  xTaskCreate(MonitorAllParametersNode,
              "MonitorAllParameters",
              128,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
#endif

  // xTaskCreate(SerialCommunicationWritingNode,
  //             "SerialCommunicationWritingNode",
  //             256,
  //             NULL,
  //             tskIDLE_PRIORITY + 1,
  //             NULL);

  xTaskCreate(SerialCommunicationReadingNode,
              "SerialCommunicationReadingNode",
              256,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  xTaskCreate(EEPROMSaveNode,
              "EEPROMSave",
              128,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);

  xTaskCreate(LEDIndicator,
              "LEDIndicator_Debug",
              64,
              NULL,
              tskIDLE_PRIORITY + 1,
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

void setup()
{
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(115200);
  Serial.println("Initializing...");
  // Serial3.begin(115200);
  // Serial3.print("asdfasdfasdfasdfasdfasdfasdf");
  pinMode(PIN_BOARD_LED, OUTPUT);
  for (int i = 0; i < NUMBER_OF_RTOS_THREADS; i++)
  {
    loop_time_checker[i] = -1;
  }

  if (ACTIVE_MOTOR)
    GripperMotor.PWMInit();
    EncoderInit();
  if (ACTIVE_LOADCELL)
    LoadCellInit();
  if (ACTIVE_RTOS_THREAD)
    RTOSInit();

  Serial.println("All Initializing DONE.");
  delay(2000);
}

void loop()
{
  /**
   * @brief no operating source in loop()
   * @note  RTOS Thread was operating each part
   */
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                  -END-



































/////////////////////

DCMotor::DCMotor()
{
  anchor = this;
}

void DCMotor::Enable()
{
  this->motor_state = kEnable;
}
void DCMotor::Disable()
{
  this->motor_state = kDisable;
  this->op_command = kNone;
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, LOW);
  pwmWrite(PIN_MOTOR_PWM, DUTY_MIN);
}
void DCMotor::SetDirCW()
{
  this->direction = kClockwise;
  digitalWrite(PIN_MOTOR_CW, HIGH);
  digitalWrite(PIN_MOTOR_CCW, LOW);
}
void DCMotor::SetDirCCW()
{
  this->direction = kCounterClockwise;
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, HIGH);
}
void DCMotor::SetDirOFF()
{
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, LOW);
}
void DCMotor::UpdateVelocity(int duty)
{
  if (duty > DUTY_MAX)
  {
    this->target_velocity = DUTY_MIN;
    Serial.print("duty setup error -> over : ");
    Serial.println(DUTY_MAX);
  }
  else if (duty < 0)
  {
    this->target_velocity = DUTY_MIN;
    Serial.print("duty setup error -> under : ");
    Serial.println(DUTY_MIN);
  }
  else
  {
    this->target_velocity = duty;
    pwmWrite(PIN_MOTOR_PWM, this->target_velocity);
  }
}
void DCMotor::UpdatePosition(long val)
{
  this->target_position = val;
}





/*********************************
 * PWM setting (Timer Register)
 **********************************/
void DCMotor::PWMInit()
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
  pwmtimer2.setPrescaleFactor(PRESCALER);    // 72/1 = 72
  pwmtimer2.setOverflow(COUNTER_PERIOD - 1); // 72mHz/3600 = 20 (kHz)
  pwmtimer2.refresh();
  pwmtimer2.resume();

  delay(100);
  Serial.println("DONE");
}

/*********************************
 * Motor Home position
 **********************************/
uint8_t DCMotor::Homing(uint16_t threshold)
{
  Serial.print("Homing ...");
  if (this->absolute_position <= threshold && this->absolute_position >= -threshold)
  {
    this->Disable();
    Serial.println("Finished");
    return 1;
  }

  if (this->absolute_position > threshold)
  {
    this->SetDirCCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }
  else if (this->absolute_position < -threshold)
  {
    this->SetDirCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }

  while (true)
  {
    if (this->actual_position <= threshold && this->actual_position >= -threshold)
    {
      this->Disable();
      Serial.println("Finished");
      break;
    }
  }
  return 1;
}

/*********************************
 * Set zero based on Load Cell
 **********************************/
uint8_t DCMotor::HomingLoadCell(float target_force)
{
  Serial.print("Homing Load Cell...");

  while (true)
  {
    float avg_loadcell_val = 0;
    for (int i = 0; i < NUMBER_OF_LOADCELL_MODULE; i++)
    {
      avg_loadcell_val += g_loadcell_val[i];
    }
    avg_loadcell_val = avg_loadcell_val / NUMBER_OF_LOADCELL_MODULE;

    if (avg_loadcell_val >= target_force)
    {
      this->actual_position = 0;
      this->difference_position = this->difference_position + (this->absolute_position - this->actual_position);

      this->Disable();
      Serial.print("Finished -> avg_val : ");
      Serial.println(avg_loadcell_val);
      break;
    }
    else
    {
      this->SetDirCW();
      this->UpdateVelocity(DUTY_MAX / 10);
      this->Enable();
    }
  }
  return 1;
}

uint8_t DCMotor::MoveOneResolution()
{
  if (this->direction = kClockwise)
  {
    this->UpdatePosition(this->actual_position + ONE_RESOLUTION);
    this->SetDirCW();
  }
  else if (this->direction = kCounterClockwise)
  {
    this->UpdatePosition(this->actual_position - ONE_RESOLUTION);
    this->SetDirCCW();
  }

  while (true)
  {
    if (this->direction == kClockwise)
    {
      if (this->actual_position <= this->target_position)
      {
        this->UpdateVelocity(DUTY_MAX / 10);
        this->Enable();
      }
      else
      {
        this->Disable();
        break;
      }
    }

    else if (this->direction == kCounterClockwise)
    {
      if (this->actual_position >= this->target_position)
      {
        this->UpdateVelocity(DUTY_MAX / 10);
        this->Enable();
      }
      else
      {
        this->Disable();
        break;
      }
    }
  }
}
