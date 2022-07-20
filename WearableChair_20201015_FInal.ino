/*
   자이로센서 : BMI160 (wit-motion.Co)
   압력센서 : FSR401 (기준저항 10 k옴)
   포토인터럽터 : ee-sx952 series
   (모듈이 없는 것이므로 회로 꼭 확인 ! --> 참고 : https://www.youtube.com/watch?v=i7APHZlFd-I)
   모터 : 스텝모터 (변속기 장착)
   (핀 잘 확인하고, EN 핀이 좀... 이상해!!! ㅜㅜ)
*/

/* 기본 제어 순서
************* 의자 펼치는 과정 **********************
     (after 배열은 그냥 초기화)
         [START]   <----------------------------------┐
     --> 좌 우 각도 수신 후 보정(0~360도)                │
     --> 도출된 값으로 각도와 각속도를 계산                │ prev = after 넣기
     --> 알고리즘 적용(기울기,각속도,그 외 센서)--- (NO)  -┘
     --> 의자 펼침 (모터 동작)


************* 의자 접는 과정 **********************
     [ 위와 동일 ] 알고리즘만 다름.
*/


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< 프로그램 시작 >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#define Baudrate 115200

//============================= 사용자 정의함수 INDEX ===============================
void initialize();
int ReadFSR_Left();
int ReadFSR_Right();
bool CheckPhotoINT_Left();
bool CheckPhotoINT_Right();
void motor_operation();
void Final_Data();
void get_data_Left();
void get_data_Right();
float get_angle_Left();
float get_angle_Right();

//============================= 전역변수 설정 ===============================
/* //LCD 출력 선언 */
//LiquidCrystal_I2C LCD(0x27, 16, 2);
int LCD_HZ = 0;

/* 자이로센서 데이터 수신용 변수 */
unsigned char Re_buf[11], counter = 0;  // 자이로센서 데이터 수신용 변수
unsigned char sign_Left = 0, sign_Right = 0;    // 데이터 처리용 트리거
unsigned char buff_trigger = 0;   // 자이로센서 좌우 수신 결정 트리거
float Re_angle = 0;   // get_data 함수의 리턴 변수 (자이로센서 데이터 분석 결과각도값)

/* FSR 압력센서 */
const int FSR_Left = A0;
const int FSR_Right = A1;
const int FSR_Threshold = 100; // 손가락 최대압 약 750, 착석시 약 350 (아날로그 Read 값)

/* 포토인터럽터 변수 */
const int PhotoINT_Left = 7;
const int PhotoINT_Right = 8;

/* 본함수에서 알고리즘에 적용할 변수*/
double stime = 0, etime = 0;   // dt를 구하기 위해 시간을 재야함 (각속도 계산 활용)
float prev_angle_Left, after_angle_Left;  // 왼쪽의 이전 이후 버퍼 (각도값)
float prev_angle_Right, after_angle_Right;  // 오른쪽의 이전 이후 버퍼 (각도값)
float w_Left, w_Right;  // 각속도
double dt = 0.01;
unsigned char alg_trigger = 0; // 알고리즘 시작용 트리거 (각속도 오류없을 때 true로 바꾸어주고 알고리즘 함수 실행)

/* 의자 다리 상태를 위한 변수 */
unsigned char state = 0; // 0 : 일어서있는 상태(의자 다리가 접힌 상태), 1 : 앉은 상태(의자 다리가 펴진 상태)

/* 자동모드 수동모드 변환을 위한 input용 핀 변수 */
const int user_mode = 4;  // input으로 사용할 것임
const int fold_motor = 5;
const int unfold_motor = 6;

/* 모터 제어를 위한 변수 */
// 설정
const char Microstep = 8; // 마이크로스텝 (8 --> 1600 step / 1 rev)
const double time_req = 500000; // setup_micro
const double theta = 45; // setup

const int TimeToUnfolding = 900; // 900 ms 동안 모터 동작시켜서 다리 펼침 (각도제어가 없기에 trial and error 로 테스트해야함)
double step_req;
double freq_req;

// 모터 드라이버 핀 설정
const int Motor_enable = 799;
const int Motor_disable = 0;
const int DIR_L = 9;
const int DIR_R = 10;
const int PWM_L = 11; // Motor Right Pin
const int PWM_R = 12;  // Motor Left Pin

// 시각용 LED Pin 설정
const int LED_Power = 31;
const int LED_Motor = 32;
const int LED_Emerg = 33;

/* [샘플] 수신 데이터 저장용 변수 (a:가속도, w:각속도, angle:절대각도(roll,pitch,yaw), T:센서내부온도) */
// 회사 제공함수 변수 - 냅두셈 (MPU6050 스케치 참조)
//float a_Left[3] w_Left[3], angle_Left[3], T_Left;
//float a_Right[3], w_Right[3], angle_Right[3], T_Right;


//============================= 실행 함수 ===============================
void setup() {
  initialize();
}

void loop() {
  int mode = digitalRead(user_mode);

  if (mode == 1) {
    Final_Data();
    // 노이즈 제거된 깔끔한 데이터 받고 알고리즘 실행
    if (alg_trigger == 1)
    {
      ////       필요시 쓰세요 데이터 값 확인용 시리얼모니터
      //            Serial.print("   A_L : ");
      //            Serial.print(after_angle_Left);
      //            Serial.print("   W_L : ");
      //            Serial.print(w_Left);
      //            Serial.print("   A_R : ");
      //            Serial.print(after_angle_Right);
      //            Serial.print("   W_R : ");
      //            Serial.print(w_Right);
      //            Serial.print("   dt : ");
      //            Serial.println(dt, 8.8);

      switch (state) {
        case 0 :  // 일어서 있을 때
          if ( (after_angle_Left > 40) && (after_angle_Left < 90) && (after_angle_Right > 40) && (after_angle_Right < 90) && (w_Left > 0) && (w_Left < 40) && (w_Right > 0) && (w_Right < 40) ) {
            motor_operation(0);
            state = 1;
            delay(500);
          }
          break;
        case 1 :  // 앉아 있을 때
          if ((after_angle_Left < 30) && (after_angle_Left > -10) && (after_angle_Right < 30) && (after_angle_Right > -10)) {
            if ( (ReadFSR_Left() < FSR_Threshold) && (ReadFSR_Right() < FSR_Threshold))
            {
              motor_operation(1);
              state = 0;
              delay(500);
            }
          }
          break;
      }
    }
    // 다음 각도 비교를 위해 이전값으로 저장
    prev_angle_Left = after_angle_Left;
    prev_angle_Right = after_angle_Right;

    // 트리거 초기화
    alg_trigger = 0;
    buff_trigger = 0;
    //LCD_HZ++;
  }

  else {
    int operate_switch_1 = digitalRead(fold_motor);
    int operate_switch_2 = digitalRead(unfold_motor);
    
    if ( (operate_switch_1 == 1) && (operate_switch_2 == 0) ){
      Serial.println("MOTOR FOLD");
      digitalWrite(DIR_L, LOW); // CCW
      digitalWrite(DIR_R, HIGH);  // CW --> 두 다리가 대칭구조이므로 모터도 대칭 회전방향을 가져야함
      OCR1A = Motor_enable;  // enable
      OCR1B = Motor_enable;  // enable
    }
    else if( (operate_switch_1 == 0) && (operate_switch_2 == 1) ){
      Serial.println("MOTOR UNFOLD");
      digitalWrite(DIR_L, HIGH);  // CW
      digitalWrite(DIR_R, LOW);  // CCW --> 두 다리가 대칭구조이므로 모터도 대칭 회전방향을 가져야함
      OCR1A = Motor_enable;  // enable
      OCR1B = Motor_enable;  // enable
    }
    else{
      OCR1A = Motor_disable;  // disable
      OCR1B = Motor_disable;  // disable
    }
  }
}

//================== 사용자 정의 함수 (인터럽트, 설정, 센서데이터 수신 및 처리 등...) =====================

/* 초기 설정 함수 */
// Baudrate, //LCD, 핀설정 등
void initialize() {
  Serial.begin(Baudrate); // 시리얼 모니터용으로 사용함
  Serial1.begin(Baudrate);
  Serial2.begin(Baudrate);
  // //LCD 초기화
  //LCD.init();
  //LCD.backlight();
  //LCD.setCursor(0, 0);
  //LCD.print("Booting...");
  delay(300);

  //  step_req = 200 * 14 * Microstep * theta / 360; // 14는 감속비
  //  freq_req = step_req / (time_req / 1000000);
  //Serial.println(step_req); Serial.println(freq_req);

  //LCD.setCursor(0, 0);
  //LCD.print("pinMode setup..");
  // 출력용 핀 설정 (모터, LED, 포토인터럽터)
  pinMode(user_mode, INPUT);
  pinMode(fold_motor, INPUT);
  pinMode(unfold_motor, INPUT);
  pinMode(PWM_L, OUTPUT);
  pinMode(DIR_L, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  digitalWrite(PWM_L, HIGH);
  digitalWrite(PWM_R, HIGH);
  pinMode(LED_Power, OUTPUT);
  pinMode(LED_Motor, OUTPUT);
  pinMode(LED_Emerg, OUTPUT);
  pinMode(PhotoINT_Left, INPUT);
  pinMode(PhotoINT_Right, INPUT);

  //  attachInterrupt(0, BUTTON, RISING);
  digitalWrite(LED_Power, HIGH);
  delay(300);

  // PWM 타이머 레지스터 설정
  TCCR1A = bit(COM1A1) | bit(COM1A0) | bit(COM1B1) | bit(COM1B0); //inverting mode => 0일때 HIGH, MAX일때 LOW
  TCCR1A |= bit(WGM11);
  TCCR1B = bit(WGM12) | bit(WGM13); // Fast PWM mode using ICR1 as TOP
  TCCR1B |= bit(CS10); // no prescaler
  ICR1 = 799; //주파수 설정 => 16MHz / ICR1 = 주파수
  OCR1A = Motor_disable; // 듀티 설정 => OCR1A/ICR1
  OCR1B = Motor_disable;
  TCNT1 = 0;

  //LCD.setCursor(0, 0);
  //LCD.print("position init..");
  // 모터 위치 초기화 (접힌상태)..****
  digitalWrite(DIR_L, LOW); // CCW
  digitalWrite(DIR_R, HIGH);  // CW --> 두 다리가 대칭구조이므로 모터도 대칭 회전방향을 가져야함
  OCR1A = Motor_enable;
  OCR1B = Motor_enable;
  //포토인터럽트 체크하면서 원위치 (원위치 아니면 모터 동작시킴)
  int flag_L = 0;
  int flag_R = 0;
  while ( (flag_L == 0) || (flag_R == 0) )
  {
    if (CheckPhotoINT_Left() == true) {
      OCR1A = Motor_disable; // disable
      flag_L = 1;  // 포토인터럽트에 들어옴
    }

    if (CheckPhotoINT_Right() == true) {
      OCR1B = Motor_disable;  // disable
      flag_R = 1;   // 포토인터럽트에 들어옴
    }
  }

  //LCD.setCursor(0, 1);
  //LCD.print("OK !!");
  OCR1A = Motor_disable;
  OCR1B = Motor_disable;
  Serial.println("???");
  delay(1000);
  //LCD.clear();
}

/* FSR 센서값 읽는 함수 (아날로그 핀 사용) */
int ReadFSR_Left()
{
  return analogRead(FSR_Left);
}
int ReadFSR_Right()
{
  return analogRead(FSR_Right);
}

/* 포토인터럽터 확인함수
   디지털핀 input 전압 체크
   0V : false, 5V : true
   물체 X : 0V 신호, 물체 O : 5V 신호 들어오는 포토인터럽트임
*/
bool CheckPhotoINT_Left () {
  return digitalRead(PhotoINT_Left);
}

bool CheckPhotoINT_Right () {
  return digitalRead(PhotoINT_Right);
}

/* 모터 동작 함수
  1. case --> 0:다리 펴짐 / 1:다리 접힘 / 2:긴급정지 / 3:초기 위치로 초기화(접힌상태로)
  2. LED 부착하여 동작 확인
  3. 가감속 프로파일 약 10% 가속 -> 약 80% 유지 -> 약 10% 감속
*/
void motor_operation(int num) {
  int flag_L = 0;
  int flag_R = 0;

  switch (num) {
    case 0: // 의자 펼침
      //delay(10);
      digitalWrite(LED_Motor, HIGH);
      Serial.println("motor unfolding");
      delay(100);
      //LCD.clear();
      //LCD.setCursor(0, 0);
      //LCD.print("Unfolding...");
      //      //LCD.setCursor(0, 1);
      //      //LCD.print("                ");

      digitalWrite(DIR_L, HIGH);  // CW
      digitalWrite(DIR_R, LOW);  // CCW --> 두 다리가 대칭구조이므로 모터도 대칭 회전방향을 가져야함

      OCR1A = Motor_enable;  // enable
      OCR1B = Motor_enable;  // enable

      delay(TimeToUnfolding); // 900 ms (=0.9 sec) 동작...
      OCR1A = Motor_disable;  // disable
      OCR1B = Motor_disable;  // disable

      //LCD.clear();
      break;

    case 1: // 의자 접힘
      digitalWrite(LED_Motor, HIGH);
      Serial.println("motor folding");
      delay(100);
      //LCD.clear();
      //LCD.setCursor(0, 0);
      //LCD.print("Folding...");
      //      //LCD.setCursor(0, 1);
      //      //LCD.print("                ");
      delay(100);
      digitalWrite(DIR_L, LOW); // CCW
      digitalWrite(DIR_R, HIGH);  // CW --> 두 다리가 대칭구조이므로 모터도 대칭 회전방향을 가져야함
      OCR1A = Motor_enable;  // enable
      OCR1B = Motor_enable;  // enable

      //포토인터럽트 체크하면서 원위치
      while ( (flag_L == 0) || (flag_R == 0) )
      {
        if (CheckPhotoINT_Left() == true) {
          OCR1A = Motor_disable; // disable
          flag_L = 1;  // 포토인터럽트에 들어옴
        }
        else {
          OCR1A = Motor_enable;
          flag_L = 0;
        }

        if (CheckPhotoINT_Right() == true) {
          OCR1B = Motor_disable;  // disable
          flag_R = 1;   // 포토인터럽트에 들어옴
        }
        else {
          OCR1B = Motor_enable;
          flag_R = 0;
        }
      }
      Serial.println("Done");
      digitalWrite(LED_Motor, LOW);
      //LCD.clear();
      break;
    case 2: // emergency stop --> 이후 Reset 과 연계
      digitalWrite(LED_Emerg, HIGH);
      Serial.println("Emergency Stop");
      OCR1A = 799;  // disable
      OCR1B = 799;  // disable
      break;
    case 3: // reset (접힌상태로) --> Emergency Stop 과 연계되어 사용됨
      // 탈조풀어주고 포토인터럽트 확인 + 모터동작으로 원위치 시키기
      Serial.println("Reset Position");

      //혹시나 덜 왔을 경우 -> 포토인터럽트 한스텝씩 체크하면서 원위치

      while ( (flag_L == 0) || (flag_R == 0) )
      {
        if (CheckPhotoINT_Left() == false) {
          digitalWrite(PWM_L, HIGH);
          delayMicroseconds(500);
          digitalWrite(PWM_L, LOW);
          delayMicroseconds(500);
        }
        else flag_L = 1;

        if (CheckPhotoINT_Right() == false) {
          digitalWrite(PWM_R, HIGH);
          delayMicroseconds(500);
          digitalWrite(PWM_R, LOW);
          delayMicroseconds(500);
        }
        else flag_R = 1;
      }
      digitalWrite(LED_Emerg, LOW);
      break;
  }
}

/* 최종 데이터 처리 함수
  1. 자이로 시리얼 데이터 수신
  2. 수신 데이터 처리 -> 숫자로 나타내기
  3. 각도 보정
  4. 각도를 통해 각속도 도출
  5. 각속도를 활용하여 노이즈 제거
  6. final data를 출력 -> 이 값은 알고리즘으로
*/
void Final_Data() {
  // buff trigger를 0 : 왼쪽, 1 : 오른쪽 확인용으로 쓰는 것임
  // 무한루프 내에 각각 다리의 각도 하나 받을때까지 갇힘
  while (buff_trigger == 0 ) {
    get_data_Left();
    after_angle_Left = get_angle_Left();
  }
  while (buff_trigger == 1) {
    get_data_Right();
    after_angle_Right = get_angle_Right();
  }

  // 기존의 Roll 범위 (-180 ~ +180)를 (+0 ~ +360) 로 보정
  after_angle_Left = after_angle_Left + 90 ;
  after_angle_Right = after_angle_Right + 90;

  //etime = micros();

  // 각속도 계산
  // double dt = (etime - stime) / 1000000; //microsec
  w_Left = ( after_angle_Left - prev_angle_Left ) / dt;
  w_Right = ( after_angle_Right - prev_angle_Right ) / dt;

  //stime = micros();

  if ( (abs(w_Left) < 400) && (abs(w_Right) < 400) ) {
    alg_trigger = 1;  // 제대로된 데이터를 수신해야 알고리즘에 집어 넣음
  }

  //  if (LCD_HZ >= 30)
  //  {
  //    // 다른 값들도 출력.... 확인용!!
  //    Serial.print("FSR_L : "); Serial.print(ReadFSR_Left()); Serial.print("     FSR_R : "); Serial.println(ReadFSR_Right());
  //    if (CheckPhotoINT_Left()) Serial.print("PhotoInt_Left : Exist  /  ");
  //    else Serial.print("PhotoInt_Left : None  /  ");
  //    if (CheckPhotoINT_Right()) Serial.println("Photoint_Right : Exist");
  //    else Serial.println("PhotoInt_Right : None");

  //    //  출력화면 :   윗줄 > Ang : XX.X  XX.X
  //    //            아래줄 > Pres : XXX  XXX
  //    int P_L = analogRead(FSR_Left), P_R = analogRead(FSR_Right);
  //
  //    //LCD.setCursor(0, 0);
  //    //LCD.print("Ang:");
  //    //LCD.setCursor(4, 0);
  //    //LCD.print(after_angle_Left, 1);
  //    //LCD.setCursor(11, 0);
  //    //LCD.print(after_angle_Right, 1);
  //    //LCD.setCursor(0, 1);
  //    //LCD.print("Press:");
  //    //LCD.setCursor(6, 1);
  //    if (P_L < 1000) //LCD.print(" ");
  //    if (P_L < 100) //LCD.print(" ");
  //    if (P_L < 10) //LCD.print(" ");
  //    //LCD.print(P_L);
  //    //LCD.setCursor(12, 1);
  //    if (P_R < 1000) //LCD.print(" ");
  //    if (P_R < 100) //LCD.print(" ");
  //    if (P_R < 10) //LCD.print(" ");
  //    //LCD.print(P_R);
  //    //    //LCD.setCursor(13, 0); //LCD.print("   "); // 갱신되면서 소수점 남아있는거 지워줄라고... ㅎㅎ
  //    //    //LCD.setCursor(13, 1); //LCD.print("   ");
  //    //LCD_HZ = 0; // 수신량 초기화
  //  }
}

/* ====== 자이로센서 데이터 처리하는 함수들 =======*/

/* 자이로센서 왼쪽 데이터 수신 */
// return 의 의미를 잘 이해해야함
void get_data_Left() {
  while (Serial1.available()) {
    Re_buf[counter] = (unsigned char)Serial1.read();
    if (counter == 0 && Re_buf[0] != 0x55) return; //第0号数据不是帧头

    // 0x53만 받아야되는데, 제품회사 소프트웨어의 컨텐츠를 각도만 받게끔 설정해야 빨리받음
    // 0x51, 0x52 받아버리면 속도 느려짐
    // Protocol : " 0x55 | 0x53 | .............. "
    if (counter == 1 && Re_buf[1] != 0x53)
    {
      Re_buf[0] = 0;
      Re_buf[1] = 0;
      counter = 0;
      //Serial.println("counter_L = 1");
      return;
    }
    counter++;

    if (counter == 11 )         //接收到11个数据
    {
      counter = 0;             //重新赋值，准备下一帧数据的接收
      sign_Left = 1;
      buff_trigger++; // 이거 loop로 옮기면 느려진다.!
      //Serial.println("L_All data save");
      return;
    }
  }
}

/* 자이로센서 오른쪽 데이터 수신 */
// return 의 의미를 잘 이해해야함
void get_data_Right() {
  while (Serial2.available()) {
    Re_buf[counter] = (unsigned char)Serial2.read();
    if (counter == 0 && Re_buf[0] != 0x55) return; //第0号数据不是帧头

    // 0x53만 받아야되는데, 제품회사 소프트웨어의 컨텐츠를 각도만 받게끔 설정해야 빨리받음
    // 0x51, 0x52 받아버리면 속도 느려짐
    // Protocol : " 0x55 | 0x53 | .............. "
    if (counter == 1 && Re_buf[1] != 0x53)
    {
      Re_buf[0] = 0;
      Re_buf[1] = 0;
      counter = 0;
      //Serial.println("counter_R = 1");
      return;
    }
    counter++;

    if (counter == 11 )         //接收到11个数据
    {
      counter = 0;             //重新赋值，准备下一帧数据的接收
      sign_Right = 1;
      buff_trigger++; // 이거 loop로 옮기면 느려진다.!
      //Serial.println("R_All data save");
      return;
    }

  }
}

/* 왼쪽 자이로센서 데이터로 각도값을 구함 */
float get_angle_Left() {
  if (sign_Left)
  {
    sign_Left = 0;
    if ( (Re_buf[0] == 0x55) && (Re_buf[1] == 0x53) )
    {
      Re_angle = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;  //检查帧头
      //Serial.print("angle_Left : "); Serial.println(Re_angle);    // 출력용
    }
    return Re_angle;
  }
}

/* 오른쪽 자이로센서 데이터로 각도값을 구함 */
float get_angle_Right() {
  if (sign_Right)
  {
    sign_Right = 0;
    if ( (Re_buf[0] == 0x55) && (Re_buf[1] == 0x53) )
    {
      Re_angle = (short(Re_buf [3] << 8 | Re_buf [2])) / 32768.0 * 180;  //检查帧头
      //Serial.print("angle_Right : "); Serial.println(Re_angle);   // 출력용
    }
    return Re_angle;
  }
}
