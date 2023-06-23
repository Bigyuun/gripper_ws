
/******************************************************************************************
 * Project : gripper_ws :: main
 * 
 * @file main.ino
 * @author Dae-Yun Jang (bigyun9375@gmail.com)
 * @git  https://github.com/Bigyuun/gripper_ws
 * @version 0.1
 * @date 2022-08-11
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
#define RTOS_FREQUENCY_LOADCELLUPDATE  2
#define RTOS_FREQUENCY_MOTOROPERATION  2
#define RTOS_FREQUENCY_ENCODER_MONITOR 10
#define BAUDRATE                       115200

#define NUMBER_OF_LOADCELL_MODULE      3

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
#define TARGET_POS          (GEAR_RATIO * ENCODER_RESOLUTION * 4 * TARGET_RESOULTION)


namespace GripperManagerNode
{

class GripperManager
{

public:
    GripperManager(/* args */);
    ~GripperManager();

private:
    /* data */

    /*********************************
     * Global variables
     *********************************/
    static long g_enc_pos = 0;
    static char motor_op_flag = true;
    
    //HX711 constructor (dout pin, sck pin)
    HX711_ADC LoadCell_1(HX711_DOUT_1, HX711_SCK_1); //HX711 1
    HX711_ADC LoadCell_2(HX711_DOUT_2, HX711_SCK_2); //HX711 2
    HX711_ADC LoadCell_3(HX711_DOUT_3, HX711_SCK_3); //HX711 3

    // Load Cell data
    unsigned long t = 0;
    volatile float f1[3]; //[x1,y1,z1]
    volatile float f2[3]; //[x2,y2,z2]
    volatile float f3[3]; //[x3,y3,z3]
    volatile float fx;
    volatile float fy;
    volatile float fz;


public:

    uint8_t EncoderInit();
    uint8_t MotorInit();
    uint8_t LoadCellInit();
    uint8_t RTOSInit();

    void isr_encoder_phase_A();
    void isr_encoder_phase_B();
    long EEPROMReadlong();
    void EEPROMWritelong();

    void EEPROMSave();
    void LoadCellUpdate();
};
   
}










