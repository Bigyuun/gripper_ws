/*********************************
 * Process Setting
 *********************************/
#define RTOS_FREQUENCY                 10     // ms (Default)
#define RTOS_FREQUENCY_LOADCELLUPDATE  2
#define RTOS_FREQUENCY_MOTOROPERATION  2
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