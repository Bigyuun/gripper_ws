
/******************************************************************************************
 * Project : gripper_ws :: main
 *
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/gripper_ws
 * @version 0.1
 * @date 2022-08-30
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

//#define USE_SEMAPHORE_DMA1
#include <EEPROM.h>
#include <MapleFreeRTOS821.h>         // For Using RTOS Thread
#include <HX711_ADC.h>                // For Initializing Load Cell
#include <HardwareTimer.h>            // For Setup PWM Timer

/*********************************
 * Process Setting
 *********************************/
// Hardware setting
#define NUMBER_OF_LOADCELL_MODULE        3
#define NUMBER_OF_MOTOR_DRIVER           1

// Utility On & OFF setting
#define ACTIVE_LOADCELL                  1  // 1-ON, 0-OFF
#define ACTIVE_MOTOR                     1  // 1-ON, 0-OFF
#define ACTIVE_DATA_MONITORING           1  // 1-ON, 0-OFF
#define ACTIVE_SERIAL_READING            1  // 1-ON, 0-OFF
#define ACTIVE_SERIAL_WRITING            1  // 1-ON, 0-OFF
#define ACTIVE_RTOS_THREAD               1  // 1-ON, 0-OFF

// ROTS Threads Frequency Setting
#define RTOS_FREQUENCY                   100   // Hz (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE    500   // Hz (Default)
#define RTOS_FREQUENCY_MOTOR_OPERATION   500   // Hz (Default)
#define RTOS_FREQUENCY_EEPROM_SAVE       250   // H-z (Default)
#define RTOS_FREQUENCY_MONITORING        25    // Hz (Default)
#define RTOS_FREQUENCY_SERIALREADING     500   // Hz (Default)
#define RTOS_FREQUENCY_SERIALWRITING     100   // Hz (Default)
#define TOTAL_SYSTEM_DELAY               1     // ms

/*********************************
 * Serial communication Parameters
 *********************************/
#define BAUDRATE                         115200
#define BANDWIDTH                        10

/*********************************
 * MCU
 *********************************/
#define CPU_CLOCK_SPEED                  72 // mHz

/*********************************
 * PWM Timer
 *********************************/
#define PRESCALER                        1
#define COUNTER_PERIOD                   3600
#define DUTY_MAX                         COUNTER_PERIOD // same as COUNTER_PERIOD
#define DUTY_MIN                         0

/*********************************
 * Pin match
 *********************************/
// Encoder pin (interrupt)
#define PIN_ENCODER_PHASE_A              PA0
#define PIN_ENCODER_PAHSE_B              PA1

// Motor Conrol pin            
#define PIN_MOTOR_PWM                    PA2
#define PIN_MOTOR_CCW                    PA3
#define PIN_MOTOR_CW                     PA4

// Load Cell pin             
#define HX711_DOUT_1                     PB4 // mcu > HX711 no 1 dout pin
#define HX711_SCK_1                      PB5  // mcu > HX711 no 1 sck pin
#define HX711_DOUT_2                     PB6 // mcu > HX711 no 2 dout pin
#define HX711_SCK_2                      PB7  // mcu > HX711 no 2 sck pin
#define HX711_DOUT_3                     PB8 // mcu > HX711 no 3 dout pin
#define HX711_SCK_3                      PB9  // mcu > HX711 no 3 sck pin

// Debug & Test pin            
#define PIN_BOARD_LED                    PC13
#define PIN_LED                          PB10

/*********************************
 * Motor info
 *********************************/
#define GEAR_RATIO                       298
#define GRIPPER_GEAR_RATIO               3.814814814

/*********************************
 * Encoder info
 *********************************/
#define ENCODER_POS_EEPROM_ADDRESS       0
#define ENCODER_RESOLUTION               7
#define HOMING_THRESHOLD                 150
#define DEFUALT_THRESHOLD                150

/*********************************
 * Load Cell info
 *********************************/
#define CALIBRATION_VALUE_1              2942.0 // calibration value load cell 1
#define CALIBRATION_VALUE_2              3069.0 // calibration value load cell 2
#define CALIBRATION_VALUE_3              2991.0 // calibration value load cell 3
#define LOADCELL_HOMING_VALUE            150

#define XY_PLANE_THETA_1                 90
#define XY_PLANE_THETA_2                 210
#define XY_PLANE_THETA_3                 -30
#define XZ_PLANE_THETA_1                 20
#define XZ_PLANE_THETA_2                 70

#define K_RUBBER                         0.05

/*********************************
 * Setting info
 *********************************/
#define TARGET_REVOLTION                 1
#define TARGET_POS                      (GEAR_RATIO * ENCODER_RESOLUTION * 4 * GRIPPER_GEAR_RATIO * TARGET_REVOLTION)
// #define TARGET_POS                      (GEAR_RATIO * ENCODER_RESOLUTION * 4 * TARGET_REVOLTION)
#define ONE_REVOLUTION                  (GEAR_RATIO * ENCODER_RESOLUTION * 4 * GRIPPER_GEAR_RATIO)
// #define ONE_REVOLUTION                  (GEAR_RATIO * ENCODER_RESOLUTION * 4)

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

float parsing_pos(String str);

// RTOS Thread Functions
void EEPROMSaveNode();
void LoadCellUpdateNode();
void MonitorAllParametersNode();
void SerialCommunicationNode();

// Mutex Function (Lock, Unlock);
void PrintOnMutex(String msg);

static void LEDIndicator();

struct LoopTimeChecker
{
  float loop_time_checker_LoadCellUpdate = -1;
  float loop_time_checker_MotorOperation = -1;
  float loop_time_checker_EEPROMUpdate   = -1;
  float loop_time_checker_SerialWriting  = -1;
  float loop_time_checker_SerialReading  = -1;
};

enum MotorDirection
{
  kNoneDir,
  kClockwise,
  kCounterClockwise,
}; 
enum MotorState
{
  kDisable,
  kEnable,
};
enum MotorOperationMode
{
  kVelocityMode = 1,
  kPositionMode,
};
enum MotorCommand
{
  kNone,
  kHoming = 1,
  kHomingLoadcell,
  kRevolution,
};
class DCMotor
{
private:
  static DCMotor *anchor;

public:
  MotorState motor_state = kDisable;
  MotorDirection direction = kNoneDir;
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
  void UpdateTargetPosition(long val);
  void SetActualPosZero();
  void SetAbsolutePosZero();
  void PWMInit();
  uint8_t Homing(uint16_t threshold);
  uint8_t HomingLoadCell(float target_force);
  uint8_t MoveRevolution(long target_rev);


  uint8_t MoveTargetPos(float target_angle);
};

/************************************************************************************************************
 * ///////////////////////////////////////////////////////////////////////////////////////////////////////////
 *                                          Global variables
 * \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
 ************************************************************************************************************/
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
static volatile float g_loadcell_val[NUMBER_OF_LOADCELL_MODULE] = {0};
static volatile float force_vector[NUMBER_OF_LOADCELL_MODULE][3] = {0};
static volatile float fx = 0;
static volatile float fy = 0;
static volatile float fz = 0;
static volatile float fz_total = 0;

unsigned long t = 0;


#endif

#if ACTIVE_DATA_MONITORING
// monitoring val
static bool allparameters_monitoring_flag = false;
static bool data_echo_flag_ = false;
LoopTimeChecker TimeChecker;
#endif

// Serial communication buffers.
String str_recv_buffer;
String str_send_buffer;

// Task synchronization (mutex & semaphore)
xSemaphoreHandle xMutex;

//==================================================================================================
/**
 * @note [CLASS] Motor Initialize & operate Functions
 *        - START              */
//==================================================================================================
DCMotor::DCMotor()
{
  
}
DCMotor::~DCMotor()
{
  
}

void DCMotor::Enable()
{
  this->motor_state = kEnable;
}
void DCMotor::Disable()
{
  // PrintOnMutex("/STOP 1;");
  this->motor_state = kDisable;
  this->op_command = kNone;
  this->direction = kNoneDir;
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, LOW);
  pwmWrite(PIN_MOTOR_PWM, DUTY_MIN);
  // PrintOnMutex("/STOP 0;");
}
void DCMotor::SetDirCW()
{
  // PrintOnMutex("/CW 1;");
  this->direction = kClockwise;
  digitalWrite(PIN_MOTOR_CW, HIGH);
  digitalWrite(PIN_MOTOR_CCW, LOW);
  // PrintOnMutex("/CW 0;");
}
void DCMotor::SetDirCCW()
{
  // PrintOnMutex("/CCW 1");
  this->direction = kCounterClockwise;
  digitalWrite(PIN_MOTOR_CW, LOW);
  digitalWrite(PIN_MOTOR_CCW, HIGH);
  // PrintOnMutex("/CCW 0");
}
void DCMotor::SetDirOFF()
{
  this->direction = kNoneDir;
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
void DCMotor::UpdateTargetPosition(long val)  {this->target_position = val;}
void DCMotor::SetActualPosZero()              {this->actual_position = 0;}
void DCMotor::SetAbsolutePosZero()            {this->absolute_position = 0;}

/*********************************
 * PWM setting (Timer Register)
 **********************************/
void DCMotor::PWMInit()
{
  // Serial.print("Motor Pin & PWM Initializing...");

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

  // Serial.println("DONE");
}

/*********************************
 * Motor Home position
 **********************************/
uint8_t DCMotor::Homing(uint16_t threshold)
{
  // Serial.print("Homing ...");
  if (this->absolute_position <= threshold && this->absolute_position >= -threshold)
  {
    this->Disable();
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
    if (this->motor_state == kEnable)
    {
      if ((this->absolute_position <= threshold) && (this->absolute_position >= -threshold))
      {
        this->Disable();
        return 1;
      }
    }
    else
    {
      this->Disable();
      return 0;
    }

    // Never delete -> if you delete this delay, then the operation will be make problem
    vTaskDelay((1) / portTICK_PERIOD_MS);
  }
  return 1;
}

/*********************************
 * Set zero based on Load Cell
 **********************************/
uint8_t DCMotor::HomingLoadCell(float target_force)
{
  // Serial.print("Homing Load Cell...");
  this->Enable();

  while (true)
  {
    if (this->motor_state == kEnable)
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
        this->Disable();
        return 1;
      }
      else
      {
        this->SetDirCW();
        this->UpdateVelocity(DUTY_MAX / 10);
        this->Enable();
      }
    }
    else
    {
      this->Disable();
      return 0;
    }

    // Never delete -> if you delete this delay, then the operation will be make problem
    vTaskDelay((1) / portTICK_PERIOD_MS);
  }
}

/*********************************
 * Move Revolution you want
 **********************************/
uint8_t DCMotor::MoveRevolution(long target_rev)
{
  // Serial.print("One revolution...");
  if (this->direction == kClockwise)
  {
    this->UpdateTargetPosition(this->absolute_position + target_rev * ONE_REVOLUTION);
    this->SetDirCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }
  else if (this->direction == kCounterClockwise)
  {
    this->UpdateTargetPosition(this->absolute_position - target_rev * ONE_REVOLUTION);
    this->SetDirCCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }

  while (true)
  {
    if (this->motor_state == kEnable)
    {
      if (this->direction == kClockwise)
      {
        if (this->absolute_position >= this->target_position)
        {
          this->Disable();
          return 1;
        }
      }
      else if (this->direction == kCounterClockwise)
      {
        if (this->absolute_position <= this->target_position)
        {
          this->Disable();
          return 1;
        }
      }
    }
    else
    {
      this->Disable();
      return 0;
    }

    // Never delete -> if you delete this delay, then the operation will be make problem
    vTaskDelay((1) / portTICK_PERIOD_MS);
  }
  return 0;
}

/***
 * @date 2023-04-27
 * @brief Moving gear to input angle. (type : degree)
*/
uint8_t DCMotor::MoveTargetPos(float target_angle){

  float target_pos = ONE_REVOLUTION * target_angle/360;
  this->UpdateTargetPosition((long)target_pos);

  if ( this->absolute_position <= target_pos + DEFUALT_THRESHOLD && this->absolute_position >= target_pos - DEFUALT_THRESHOLD ) {
    this->Disable();
    return 1;
  }

  if ( this->absolute_position > target_pos ) {
    this->SetDirCCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }
  else if ( this->absolute_position < target_pos) {
    this->SetDirCW();
    this->UpdateVelocity(DUTY_MAX / 10);
    this->Enable();
  }

  while(true) {
    if (this->motor_state == kEnable) {
      if (this->direction == kClockwise) {
        if (this->absolute_position >= this->target_position-DEFUALT_THRESHOLD) {
          this->Disable();
          return 1;
        }
      }
      else if (this->direction == kCounterClockwise) {
        if (this->absolute_position <= this->target_position+DEFUALT_THRESHOLD) {
          this->Disable();
          return 1;
        }
      }
    } else {
      this->Disable();
      return 0;
    }
    // Never delete -> if you delete this delay, then the operation will be make problem
    vTaskDelay((1) / portTICK_PERIOD_MS);
  }
  return 0;
}

//==================================================================================================
/**
 * @note [CLASS] Motor Initialize & operate Functions
 *        - END              */
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
  // look for a high-to-low on channel B
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
 * Encoder initialization
 **********************************/
void EncoderInit()
{
  // Serial.print("Encoder interrupter Initializing...");
  pinMode(PIN_ENCODER_PHASE_A, INPUT);
  pinMode(PIN_ENCODER_PAHSE_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PHASE_A), isr_encoder_phase_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_PAHSE_B), isr_encoder_phase_B, CHANGE);

  GripperMotor.absolute_position = 0;
  GripperMotor.absolute_position = EEPROMReadlong(ENCODER_POS_EEPROM_ADDRESS);
  Serial.print("abs pos : ");Serial.println(GripperMotor.absolute_position);
  delay(100);
  // Serial.print("DONE");
  // Serial.print(" -> EEPROM_pos : ");
  // Serial.print(GripperMotor.absolute_position);
  // Serial.print(" / target Pos : ");
  // Serial.println(TARGET_POS);
}

#if ACTIVE_LOADCELL
/*********************************
 * Load Cell Initialization
 **********************************/
uint8_t LoadCellInit()
{
  // Serial.print("Load Cell Inititializing...");

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
    // Serial.print("\n");
    // Serial.println("Timeout, check MCU>HX711 no.1 wiring and pin designations");
  }

  if (LoadCell_2.getTareTimeoutFlag())
  {
    // Serial.print("\n");
    // Serial.println("Timeout, check MCU>HX711 no.2 wiring and pin designations");
  }

  if (LoadCell_3.getTareTimeoutFlag())
  {
    // Serial.print("\n");
    // Serial.println("Timeout, check MCU>HX711 no.3 wiring and pin designations");
  }

  LoadCell_1.setCalFactor(CALIBRATION_VALUE_1); // user set calibration value (float)
  LoadCell_2.setCalFactor(CALIBRATION_VALUE_2); // user set calibration value (float)
  LoadCell_3.setCalFactor(CALIBRATION_VALUE_3); // user set calibration value (float)

  // Serial.println("Done");
  return 1;
}
#endif

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
 * Monitoring Data & loop time of each Threads
 **********************************/
#if ACTIVE_DATA_MONITORING
void MonitorAllParametersNode(void *pvParameters)
{
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
      Serial.print("           ");
      Serial.print(GripperMotor.actual_position);
      Serial.print("\n");
      Serial.println("----------------- <Loop Time> -----------------------");
      Serial.println("EEPROMUpdate  LoadCellUpdate  MotorControl  SerialReading  SerialWriting");

      Serial.print(TimeChecker.loop_time_checker_EEPROMUpdate, 0);
      Serial.print(" ms          ");
      Serial.print(TimeChecker.loop_time_checker_LoadCellUpdate, 0);
      Serial.print(" ms          ");
      Serial.print(TimeChecker.loop_time_checker_MotorOperation, 0);
      Serial.print(" ms          ");
      Serial.print(TimeChecker.loop_time_checker_SerialReading, 0);
      Serial.print(" ms          ");
      Serial.print(TimeChecker.loop_time_checker_SerialWriting, 0);
      Serial.print(" ms");
      Serial.print("\n");
    }

    vTaskDelay((1000 / RTOS_FREQUENCY_MONITORING) / portTICK_PERIOD_MS);
  }
}
#endif

/*********************************
 * Serial Command Information
 **********************************/
void SerialCommandINFO()
{
  Serial.println("================================ Command Info ======================================");
  Serial.println("[Protocol] : '/' + 'command_message' + ';' ( Example -> /CW; )");
  Serial.println("CW                  : Clockwise");
  Serial.println("CCW                 : Counter clockwise");
  Serial.println("SETACTZERO     (Z)  : Set Encoder Actual Pos 0 (actual pos only, not absolute pos)");
  Serial.println("SETABSZERO     (ZB) : Set Encoder Absolute Pos 0 (absolute pos only, not actual pos)");
  Serial.println("RUN            (R)  : Move motor depend on user setup direction & speed");
  Serial.println("STOP           (S)  : Stop motor");
  Serial.println("REVOLUTION     (O)  : Move motor 1 rev");
  Serial.println("MOVE TARGETPOS (P)  : Move to target pose (degree)");
  Serial.println("HOMING         (H)  : Motor Homing operation (Move until the absolute pos will be 0)");
  Serial.println("LOADCELLHOMING (LH) : Load cell Homing operation (Move until the average of all loadcell value will be user set (default : 150)");
  Serial.println("MONITOR        (M)  : Monitoring values & loop time of each threads");
  Serial.println("NONMONITOR     (N)  : Stop Monitoring");
  Serial.println("ECHOOFF             : Stop Echo");
  Serial.println("ECHOON              : Start Echo");
  Serial.println("I                   : Show Command Info");
  Serial.println("====================================================================================");
}

/*********************************
 * Serial Data Parsing
 **********************************/
float parsing_pos(String str){
  // str[0] is must 'P'
  float pos = str.substring(1, str.length()).toFloat();
  return pos;
}

/*********************************
 * RTOS Thread Initializing
 **********************************/
#if ACTIVE_RTOS_THREAD
uint8_t RTOSInit()
{
  // Serial.print("RTOS Thread Creating...");

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

#if ACTIVE_SERIAL_WRITING
  xTaskCreate(EchoNode,
              "EchoNode",
              384,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
#endif

#if ACTIVE_SERIAL_READING
  xTaskCreate(SerialReadingNode,
              "SerialReadingNode",
              256,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
#endif

#if ACTIVE_MOTOR
  xTaskCreate(MotorOperatingNode,
              "MotorOperationMode",
              256,
              NULL,
              tskIDLE_PRIORITY + 1,
              NULL);
#endif

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

  // Serial.println("Done");

  // if vTaskStartScheduler doen't work, under line will be operated.
  vTaskStartScheduler();
  Serial.println("vTaskStartScheduler() failed!!!");

  return 0;
}
#endif

//==================================================================================================
/**
 * @note THREADS Nodes
 *        - START               */
//==================================================================================================

/*********************************
 * Motor Sequences
 **********************************/
void MotorOperatingNode(void *pvParameters)
{ 
  GripperMotor.Homing(HOMING_THRESHOLD);
  
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while (true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    if      (GripperMotor.op_command == kHoming)          {PrintOnMutex("/HOMING 1;");
                                                           GripperMotor.Homing(HOMING_THRESHOLD); 
                                                           PrintOnMutex("/HOMING 0;");
                                                           }
    else if (GripperMotor.op_command == kHomingLoadcell)  {PrintOnMutex("/HOMINGLOADCELL 1;");
                                                           GripperMotor.HomingLoadCell(LOADCELL_HOMING_VALUE);
                                                           PrintOnMutex("/HOMINGLOADCELL 0;");
                                                           }
    else if (GripperMotor.op_command == kRevolution)      {PrintOnMutex("/REVOLUTION 1;");
                                                           GripperMotor.MoveRevolution(TARGET_REVOLTION);
                                                           PrintOnMutex("/REVOLUTION 0;");
                                                           }

    TimeChecker.loop_time_checker_MotorOperation = temp_time;
    vTaskDelay(((1000 / RTOS_FREQUENCY_MOTOR_OPERATION) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}

/*********************************
 * Serial Communication Update
 **********************************/
#if ACTIVE_SERIAL_READING
void SerialReadingNode(void *pvParameters)
{
  // SerialCommandINFO();
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;

  while (true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    if (Serial.available() <= 0)
      continue;

    // received messages update
    str_recv_buffer += Serial.readString();
    // find '/'
    int ipos0 = str_recv_buffer.indexOf('/');
    if (ipos0 < 0)
      continue;
    // find ';'
    int ipos1 = str_recv_buffer.indexOf(';');
    if (ipos1 < 0)
      continue;
    // get valid message & recv_buffer clear
    String valid_msg = str_recv_buffer.substring(ipos0 + 1, ipos1);
    str_recv_buffer = "";

    if (valid_msg == "CW")                  {PrintOnMutex("/CW 1;"); GripperMotor.SetDirCW(); PrintOnMutex("/CW 0;");}
    else if (valid_msg == "CCW")            {PrintOnMutex("/CCW 1;"); GripperMotor.SetDirCCW(); PrintOnMutex("/CCW 0;");}
    else if (valid_msg == "SETACTZERO")     {PrintOnMutex("/SETACTZERO 1;"); GripperMotor.SetActualPosZero(); PrintOnMutex("/SETACTZERO 0;");}
    else if (valid_msg == "Z")              {PrintOnMutex("/SETACTZERO 1;"); GripperMotor.SetActualPosZero(); PrintOnMutex("/SETACTZERO 0;");}
    else if (valid_msg == "SETABSZERO")     {PrintOnMutex("/SETABSZERO 1;"); GripperMotor.SetAbsolutePosZero(); PrintOnMutex("/SETABSZERO 0;");}
    else if (valid_msg == "ZB")             {PrintOnMutex("/SETABSZERO 1;"); GripperMotor.SetAbsolutePosZero(); PrintOnMutex("/SETABSZERO 0;");}
    else if (valid_msg == "RUN")           {PrintOnMutex("/RUN 1;");
                                            GripperMotor.motor_state = kEnable;
                                            GripperMotor.UpdateVelocity(DUTY_MAX / 10);
                                            PrintOnMutex("/RUN 0;");
                                            }
    else if (valid_msg == "R")             {PrintOnMutex("/RUN 1;");
                                            GripperMotor.motor_state = kEnable;
                                            GripperMotor.UpdateVelocity(DUTY_MAX / 10);
                                            PrintOnMutex("/RUN 0;");
                                            }
    else if (valid_msg == "STOP")           {PrintOnMutex("/STOP 1;"); GripperMotor.Disable(); PrintOnMutex("/STOP 0;");}
    else if (valid_msg == "S")              {PrintOnMutex("/STOP 1;"); GripperMotor.Disable(); PrintOnMutex("/STOP 0;");}
    else if (valid_msg == "HOMING")         GripperMotor.op_command = kHoming;
    else if (valid_msg == "H")              GripperMotor.op_command = kHoming;
    else if (valid_msg == "HOMINGLOADCELL") GripperMotor.op_command = kHomingLoadcell;
    else if (valid_msg == "HL")             GripperMotor.op_command = kHomingLoadcell;
    else if (valid_msg == "REVOLUTION")     GripperMotor.op_command = kRevolution;
    else if (valid_msg == "O")              GripperMotor.op_command = kRevolution;
    else if (valid_msg[0] == 'P')           {PrintOnMutex("/P 1;");GripperMotor.MoveTargetPos(parsing_pos(valid_msg));PrintOnMutex("/P 0;")}
    else if (valid_msg == "MONITOR")        {PrintOnMutex("/MONITOR 1;"); allparameters_monitoring_flag = true; PrintOnMutex("/MONITOR 0");}
    else if (valid_msg == "M")              {PrintOnMutex("/MONITOR 1;"); allparameters_monitoring_flag = true; PrintOnMutex("/MONITOR 0");}
    else if (valid_msg == "NONMONITOR")     {PrintOnMutex("/NONMONITOR 1;"); allparameters_monitoring_flag = false; PrintOnMutex("/NONMONITOR 0;");}
    else if (valid_msg == "N")              {PrintOnMutex("/NONMONITOR 1;"); allparameters_monitoring_flag = false; PrintOnMutex("/NONMONITOR 0;");}
    else if (valid_msg == "I")              SerialCommandINFO();
    else if (valid_msg == "ECHOON")         {PrintOnMutex("/ECHOON 1;"); data_echo_flag_ = true; PrintOnMutex("/ECHOON 0;");}
    else if (valid_msg == "ECHOOFF")        {PrintOnMutex("/ECHOOFF 1;"); data_echo_flag_ = false; PrintOnMutex("/ECHOOFF 0;");}

    // Serial.print("Command Message : ");
    // Serial.println(valid_msg);
    TimeChecker.loop_time_checker_SerialReading = temp_time;
    vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALREADING)) / portTICK_PERIOD_MS);
  }
}
#endif

#if ACTIVE_SERIAL_WRITING
void EchoNode(void *pvParameters)
{
  static unsigned long curt_time = millis();
  static unsigned long prev_time = millis();
  static unsigned long temp_time = 0;
  static unsigned long count_ = 0;
  const char C_STX = 2;
  const char C_ETX = 3;
  while (true)
  {
    curt_time = millis();
    temp_time = curt_time - prev_time;
    prev_time = curt_time;

    if (data_echo_flag_ == true)
    {
      // str_send_buffer += String(C_STX);
      str_send_buffer += String(GripperMotor.motor_state) + ",";
      str_send_buffer += String(GripperMotor.op_command) + ",";
      for (int i = 0; i < NUMBER_OF_LOADCELL_MODULE; i++)
      {
        str_send_buffer += String(g_loadcell_val[i]) + ",";
      }
      str_send_buffer += String(fx) + ",";
      str_send_buffer += String(fy) + ",";
      str_send_buffer += String(fz) + ",";
      str_send_buffer += String(fz_total) + ",";
      str_send_buffer += String(GripperMotor.absolute_position) + ",";
      // str_send_buffer += String(count_++);
//      str_send_buffer += String(C_ETX);

      if (xSemaphoreTake(xMutex, (portTickType)10) == true)
      {
        Serial.println(str_send_buffer);
        xSemaphoreGive(xMutex);
      }
    }
    str_send_buffer = "";
    TimeChecker.loop_time_checker_SerialWriting = temp_time;
//    vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALWRITING) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
    vTaskDelay(((1000 / RTOS_FREQUENCY_SERIALWRITING) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}
#endif

/*********************************
 * EEPROM Update (Global encoder pos update)
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
    
    TimeChecker.loop_time_checker_EEPROMUpdate = temp_time;
    vTaskDelay(((1000 / RTOS_FREQUENCY_EEPROM_SAVE) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}

/*********************************
 * Load Cell data Update
 **********************************/
#if ACTIVE_LOADCELL
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
    g_loadcell_val[0] = LoadCell_1.getData()/K_RUBBER;
    g_loadcell_val[1] = LoadCell_2.getData()/K_RUBBER;
    g_loadcell_val[2] = LoadCell_3.getData()/K_RUBBER;

    force_vector[0][0] = g_loadcell_val[0] * sin(radians(XZ_PLANE_THETA_2)) * cos(radians(XY_PLANE_THETA_2));
    force_vector[0][1] = g_loadcell_val[0] * sin(radians(XZ_PLANE_THETA_2)) * sin(radians(XY_PLANE_THETA_2));
    force_vector[0][2] = g_loadcell_val[0] * cos(radians(XZ_PLANE_THETA_1));
    
    force_vector[1][0] = g_loadcell_val[1] * sin(radians(XZ_PLANE_THETA_2)) * cos(radians(XY_PLANE_THETA_1));
    force_vector[1][1] = g_loadcell_val[1] * sin(radians(XZ_PLANE_THETA_2)) * sin(radians(XY_PLANE_THETA_1));
    force_vector[1][2] = g_loadcell_val[1] * cos(radians(XZ_PLANE_THETA_1));

    force_vector[2][0] = g_loadcell_val[2] * sin(radians(XZ_PLANE_THETA_2)) * cos(radians(XY_PLANE_THETA_3));
    force_vector[2][1] = g_loadcell_val[2] * sin(radians(XZ_PLANE_THETA_2)) * sin(radians(XY_PLANE_THETA_3));
    force_vector[2][2] = g_loadcell_val[2] * cos(radians(XZ_PLANE_THETA_1));

    fx = (force_vector[0][0] + force_vector[1][0] + force_vector[2][0])/3;
    fy = (force_vector[0][1] + force_vector[1][1] + force_vector[2][1])/3;
    fz = (force_vector[0][2] + force_vector[1][2] + force_vector[2][2])/3;

    TimeChecker.loop_time_checker_LoadCellUpdate = temp_time;
    vTaskDelay(((1000 / RTOS_FREQUENCY_LOADCELLUPDATE) - TOTAL_SYSTEM_DELAY) / portTICK_PERIOD_MS);
  }
}
#endif

/*********************************
 * Serial Print using Mutex
 **********************************/
void PrintOnMutex(String msg)
{
  if(xSemaphoreTake(xMutex, (portTickType)10 ) == true)
  {
    Serial.println(msg);
    xSemaphoreGive(xMutex);
  }
}



/*********************************
 * Indicator for operating successfully
 **********************************/
static void LEDIndicator(void *pvParameters)
{
  while (true)
  {
    vTaskDelay(500);
    digitalWrite(PIN_BOARD_LED, HIGH);
    vTaskDelay(500);
    digitalWrite(PIN_BOARD_LED, LOW);
  }
}

//==================================================================================================
/**
 * @note THREADS Nodes
 *        - END               */
//==================================================================================================

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Setup & Loop >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//                                 -START-

void setup()
{
  // put your setup code here, to run once:
  delay(3000);
  Serial.begin(BAUDRATE);
  Serial.setTimeout(1);
  // Serial.println("Initializing...");
  pinMode(PIN_BOARD_LED, OUTPUT);

  if (ACTIVE_MOTOR)
  {
    GripperMotor.PWMInit();
    EncoderInit();
  }
  if (ACTIVE_LOADCELL)
  {
    LoadCellInit();
  }

  /**
   * @brief RTOS Threads must be created at last
   */
  if (ACTIVE_RTOS_THREAD)
  {
    vSemaphoreCreateBinary(xMutex); // Semaphore must be created before xTaskCreate()
    RTOSInit();
  }
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
