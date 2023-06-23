#include <MapleFreeRTOS821.h>

#define BOARD_LED_PIN PC13
#define LED_PIN PB11
 
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

 
void setup() 
{
  Serial.begin(115200);
  
  pinMode(BOARD_LED_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(PB9, OUTPUT);
  pinMode(PC14, OUTPUT);
//  xTaskCreate(task1,"Task1",
//              configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task2,"Task2",
//              configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task3,"Task3",
//              configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 2,NULL);
//  xTaskCreate(task4,"Task4",
//              configMINIMAL_STACK_SIZE,NULL,tskIDLE_PRIORITY + 2,NULL);


  xTaskCreate(task1,"Task1",
              200,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(task2,"Task2",
              200,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(task3,"Task3",
              200,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(task4,"Task4",
              200,NULL,tskIDLE_PRIORITY + 2,NULL);
  xTaskCreate(task5,"Task5",
              200,NULL,tskIDLE_PRIORITY + 2,NULL);
  vTaskStartScheduler();
}
 
void loop() 
{

}
