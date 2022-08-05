#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
#include <Arduino.h>

//pins:
const int HX711_dout_1 = 6; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 7; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 10; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 11; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 12; //mcu > HX711 no 3 dout pin
const int HX711_sck_3 = 13; //mcu > HX711 no 3 sck pin

//HX711 constructor (dout pin, sck pin)
HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3

const int calVal_eepromAdress_1 = 0; // eeprom adress for calibration value load cell 1 (4 bytes)
const int calVal_eepromAdress_2 = 4; // eeprom adress for calibration value load cell 2 (4 bytes)
const int calVal_eepromAdress_3 = 8; // eeprom adress for calibration value load cell 3 (4 bytes)
unsigned long t = 0;

volatile float f1[3]; //[x1,y1,z1]
volatile float f2[3]; //[x2,y2,z2]
volatile float f3[3]; //[x3,y3,z3]

volatile float fx;
volatile float fy;
volatile float fz;


long encoder_CWvalue;
long encoder_CCWvalue;
long Home_value;

void setup() 
{
  Serial.begin(115200);
  delay(10);
  Serial.println("Starting...");

  float calibrationValue_1 = 2072.0; // calibration value load cell 1
  float calibrationValue_2 = 2072.0; // calibration value load cell 1
  float calibrationValue_3 = 2072.0; // calibration value load cell 3

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  
  unsigned long stabilizingtime = 2000; // tare preciscion can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; //set this to false if you don't want tare to be performed in the next step
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  
  while ((loadcell_1_rdy + loadcell_2_rdy) < 2) 
  { //run startup, stabilization and tare, both modules simultaniously
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
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
  
  pinMode(5, OUTPUT); //PWM
  pinMode(6, OUTPUT); //CW(LOW-) & CCW(HIGH+)
  pinMode(7, OUTPUT); //STOP(LOW) & GO(HIGH)
  digitalWrite(6, HIGH);
  digitalWrite(7, LOW);

//----------------------------------------homing
//CW
while(LoadCell_3.getData() < 100)
{
  if (LoadCell_3.getData() > 100)
  {
    break;
  }
  
  analogWrite(5,200);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  LoadCell_3.update();
  Serial.print(LoadCell_3.getData());
  Serial.print("\t");
}

  digitalWrite(7, LOW);
  delay(500);

//CCW
while(LoadCell_3.getData() < 150)
{
  if (LoadCell_3.getData() > 150)
  {
    break;
  }
  
  analogWrite(5,200);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  LoadCell_3.update();
  Serial.print(LoadCell_3.getData());
  Serial.print("\t");
}


  digitalWrite(7, LOW);
  delay(500);
      
  Serial.println(LoadCell_1.getData());
  Serial.println(LoadCell_2.getData());
  Serial.println(LoadCell_3.getData());
  Home_value = (encoder_CWvalue + encoder_CCWvalue) / 2;
  Serial.print("Home_value : ");
  Serial.println(Home_value);
  delay(5000);

//GO HOME
if(Home_value > 0)
{analogWrite(5,100);
  digitalWrite(6, LOW);
  digitalWrite(7, HIGH);
  }

else
{analogWrite(5,100);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  }


  LoadCell_3.update();
  Serial.print(LoadCell_3.getData());
  Serial.print("\t");
}

void loop()
{}
/*
void loop()
{
  encoder.service();
  int encoder_change = encoder.get_change();
  if (encoder_change) {
    Serial.println(encoder.get_count());
  } 
  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();
  LoadCell_3.update();

  //get smoothed value from data set
  if ((newDataReady)) 
  {
    if (millis() > t + serialPrintInterval) 
    {
      float f_1 = LoadCell_1.getData();
      float f_2 = LoadCell_2.getData();
      int f_3 = LoadCell_3.getData();
      
      do
        {
        analogWrite(5,50);
        digitalWrite(6, HIGH);
        digitalWrite(7, HIGH);
        Serial.print("Load : ");
        Serial.print(f_3);
        encoder_CWvalue = encoder.get_count();
        Serial.print("     encoder_CWvalue : ");
        Serial.println(encoder_CWvalue);
        }
        while (f_3 == 100);
        
        analogWrite(5,50);
        digitalWrite(6, LOW);
        digitalWrite(7, HIGH);


      
      newDataReady = 0;
      t = millis();
    }
  }
}
*/


void HongJunHoming()
{
  
}