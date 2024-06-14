// global data
int nowphase = 0;  //(0='check',1='ready',2='flight',3='open')
int accelcount = 0, altitudecount = 0;
bool judge_func_CR = false;
bool judge_func_RF = false;
bool judge_func_FO = false;
bool judge_accel = false;
bool judge_altitude = false;
unsigned long time_100Hz = 0;
int count_10Hz = 0;
String uplink = "";

double acceldata = 0;
double altitudedata = 0;
double voltagedata=0;
//閾値設定
double accel_RF_threshold = 0;     //READYからFLIGHTへの加速度の遷移条件
double accel_FO_threshold = 0;     //FLIGHTからREADYへの加速度の遷移条件
double altitude_RF_threshold = 0;  //READYからFLIGHTへの高度の遷移条件
double altitude_FO_threshold = 0;  //FLIGHTからREADYへの高度の遷移条件
double voltage_threshold = 0;      //keyスイッチによる電圧検知
//LED
#define PWM_LED_RED 20   //GPIO20を使用する
#define PWM_LED_BLUE 21  //GPIO21を使用する
// digitalWrite(PWM_LED_RED, HIGH);
// digitalWrite(PWM_LED, LOW);

//servo
#define PWM_SERVO_1 5  //GPIO5を使用する
#define PWM_SERVO_2 6  //GPIO6を使用する

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT 2
#define CAN0_CS 3
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);
#define get_data_xiao1 /*アドレス*/ ;
#define get_data_xiao2 /*アドレス*/ ;
#define CCP_opener_control /*アドレス*/ ;



// setup()ではdelay()使用可
void setup() {
  //LED
  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);
  //servo
  pinMode(PWM_SERVO_1, OUTPUT);
  pinMode(PWM_SERVO_2, OUTPUT);
  //CAN
  CCP.begin();
  //LEDデバック
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  // デバッグ出力
  Serial.begin(115200);
  // Wire.setSDA(BNO_SDA);
  // Wire.setSCL(BNO_SCL);
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;
    count_10Hz++;  //100Hz処理されたときに+1
    if (count_10Hz > 10) {
      count_10Hz = 0;
      // 10Hzで実行する処理
    }
    //100Hzで実行する処理
  }
  // 常に実行する処理
  //LED消灯
  digitalWrite(PWM_LED_BLUE, Low);
  digitalWrite(PWM_LED_RED, Low);
  //getDataここでデータ格納

  //CEACK_to_READY
  if (voltagedata >= voltage_threshold) {
    judge_func_CR = true;
  }

  //AccelJudge
  //READY_to_FLIGHT
  if (nowphase == 1) {
    if (accel_RF_threshold <= acceldata) {  //閾値以上
      accelcount++;
      if (accelcount >= 5) {
        digitalWrite(PIN_LED_RED, HIGH);
        judge_accel = true;
        accelcount = 0;
      }
    } else {
      accelcount = 0;
    }
  } else if (nowphase == 2) {
    //FLIGHT_to_OPEN
    if (accel_RF_threshold >= acceldata) {  //閾値以下
      accelcount++;
      if (accelcount >= 5) {
        digitalWrite(PIN_LED_RED, HIGH);
        judge_accel = true;
        accelcount = 0;
      }
    } else {
      accelcount = 0;
    }
  }

  //AltitudeJudge
  //READY_to_FLIGHT
  if (nowphase == 1) {
    if (altitude_RF_threshold <= altitudedata) {  //閾値以上
      altitudecount++;
      if (altitudecount >= 5) {
        digitalWrite(PWM_LED_RED, HIGH);
        judge_altitude = true;
        altitudecount = 0;
      }
    } else {
      altitudecount = 0;
    }
  } else if (nowphase == 2) {
    //FLIGHT_to_OPEN
    if (altitude_RF_threshold <= altitudedata) {  //閾値以下
      altitudecount++;
      if (altitudecount >= 5) {
        digitalWrite(PWM_LED_RED, HIGH);
        judge_altitude = true;
        altitudecount = 0;
      }
    } else {
      altitudecount = 0;
    }
  }

  if ((judge_accel) && (judge_altitude)) {  //READYからFLIGHTへの遷移を許可
    if (nowphase == 1) {
      judge_func_RF = true;
      judge_accel = false;
      judge_altitude = false;
    } else if (nowphase == 2) {  //FLIGHTからOPENへの遷移を許可
      judge_func_FO = true;
      judge_accel = false;
      judge_altitude = false;
    }
  }
  //phasejudge
  if ((nowphase == 0) && (judge_func_CR)) {
    digitalWrite(PWM_LED_BLUE, HIGH);
    nowphase = 1;  //READY
  } else if ((nowphase == 1) && (judge_func_RF)) {
    digitalWrite(PWM_LED_BLUE, HIGH);
    nowphase = 2;  //FLIGHT
  } else if ((nowphase == 2) && (judge_func_FO)) {
    digitalWrite(PWM_LED_BLUE, HIGH);
    //servo
    digitalWrite(PWM_SERVO_1, HIGH);
    digitalWrite(PWM_SERVO_2, HIGH);
    nowphase = 3;  //OPENED
  }

  //CCP.read_device();
  //uplink
  // int sensorValue = analogRead(voltagePin);  // アナログ入力から値を読み取る
  // float voltage = sensorValue * (5.0 / 1023.0);  // センサ値を電圧に変換（5V基準）
  //voltagedata,acceldata,altitudedataの格納
  switch (CCP.id) {
    case CCP_opener_control:
      if (CCP.str_match("CHECK", 5)) goCHECK();
      if (CCP.str_match("READY", 5)) goREADY();
      break;
    case CCP_open_time_command_s:
      open_threshold_time_ms = CCP.data_float() * 1000;
      delay(200);
      CCP.float_to_device(CCP_open_time_response_s, (float)open_threshold_time_ms / 1000.0);
      break;
    default:
      break;
  }
}
void goCHECK() {
  nowphase = 0;
  accelcount = 0;
  altitudecount = 0;
  judge_func_CR = false;
  judge_func_RF = false;
  judge_func_FO = false;
}
void goREADY() {
  nowphase = 1;
  judge_func_CR = true;
  judge_func_RF = false;
  judge_func_FO = false;
}