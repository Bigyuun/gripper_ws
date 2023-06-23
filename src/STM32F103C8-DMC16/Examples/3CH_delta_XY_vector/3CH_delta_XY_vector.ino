#include "HX711.h"

HX711 scale1;
HX711 scale2;
HX711 scale3;

// HX711 circuit wiring
uint8_t dataPin_1 = PA6;
uint8_t clockPin_1 = PA7;
uint8_t dataPin_2 = PB5;
uint8_t clockPin_2 = PB6;
uint8_t dataPin_3 = PB7;
uint8_t clockPin_3 = PB8;

uint32_t start, stop;
volatile float f_1;
volatile float f_2;
volatile float f_3;

volatile float f1[3]; //[x1,y1,z1]
volatile float f2[3]; //[x2,y2,z2]
volatile float f3[3]; //[x3,y3,z3]

volatile float fx;
volatile float fy;
volatile float fz;

volatile float rubber_k = 0.05;

volatile float f_ztotal;



void setup()
{

  
  Serial.begin(115200);
  delay(10);
  //Serial.println(scale.read());      // print a raw reading from the ADC
  //Serial.println(scale.read_average(20));    // print the average of 20 readings from the ADC
  //Serial.println(scale.get_value(5));    // print the average of 5 readings from the ADC minus the tare weight (not set yet)
  //Serial.println(scale.get_units(5), 1);  // print the average of 5 readings from the ADC minus tare weight (not set) divided
  // by the SCALE parameter (not set yet)
  //scale.set_scale(2280.f);    // this value is obtained by calibrating the scale with known weights; see the README for details
  //scale.tare();                // reset the scale to 0

  scale1.begin(dataPin_1, clockPin_1);
  scale2.begin(dataPin_2, clockPin_2);
  scale3.begin(dataPin_3, clockPin_3);

  // TODO find a nice solution for this calibration..
  // load cell factor 500G
  scale1.set_scale(2072.54);
  scale2.set_scale(2072.54);
  scale3.set_scale(2072.54);
  
  // load cell factor 500G
  // scale.set_scale(2072.54);
  // reset the scale to zero = 0
  scale1.tare();
  scale2.tare();
  scale3.tare();

  Serial.println("\nPERFORMANCE");
  start = micros();
  f_1 = 0;
  f_2 = 0;
  f_3 = 0;
  for (int i = 0; i < 100; i++)
  {
    f_1 = scale1.read_medavg(7);
    delayMicroseconds(500);
    f_2 = scale2.read_medavg(7);
    delayMicroseconds(500);
    f_3 = scale3.read_medavg(7);
  }
  stop = micros();
  Serial.print("100x read_medavg(7) = ");
  Serial.println(stop - start);
  Serial.print("  VAL: ");
  Serial.println(f_1, 2);
  Serial.println(f_2, 2);
  Serial.println(f_3, 2);
}


void loop()
{
  f_1 = scale1.get_value(3); //print the average of 5 readings from the ADC minus tare weight (not set) divided
  delayMicroseconds(500);
  f_2 = scale2.get_value(3);
  delayMicroseconds(500);
  f_3 = scale3.get_value(3);
  delayMicroseconds(500);
  
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
  
  f_ztotal = (f1[2] + f2[2] + f3[2]) / rubber_k / 3;

  Serial.print(f_1);
  Serial.print(",");
  Serial.print(f_2);
  Serial.print(",");
  Serial.println(f_3);
//  Serial.print(",");

//  Serial.print(fx);
//  Serial.print(",");
//  Serial.print(fy);
//  Serial.print(",");
//  Serial.println(fz);

  delayMicroseconds(500);
}
