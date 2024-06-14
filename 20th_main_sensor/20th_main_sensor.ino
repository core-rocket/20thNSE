#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//key
const int voltagePin = A0;           // アナログ入力ピンを指定
const float thresholdVoltage = 2.5;  // しきい値電圧 (例: 2.5V)
//LED
#define PWM_LED_RED 20   //GPIO20を使用する
#define PWM_LED_BLUE 21  //GPIO21を使用する
// digitalWrite(PIN_LED, HIGH);
// digitalWrite(PIN_LED, LOW);

//CAN
#include <CCP_MCP2515.h>
#define CAN0_INT 2
#define CAN0_CS 3
CCP_MCP2515 CCP(CAN0_CS, CAN0_INT);
#define get_data_xiao1 /*アドレス*/ ;
#define get_data_xiao2 /*アドレス*/ ;
#define fasejudge_xiao /*アドレス*/ ;

Adafruit_BME280 bme;  // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

//BME280
float get_bme_pressure[10];
float get_bme_temperature[10];
float get_bme_humidity[10];
float pressure_median;
float temperature_median;
float humidity_median;

//BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
float accel_data[5][3];  //x,y,z軸方向のデータを10回格納するために[10][3]
float orientation_data[5][3];
float velocity_data[5][3];
float linearaccel_data[5][3];
float magnetometer_data[5][3];
float gravity_data[5][3];

float bno_accel_median[3];  //x,y,z軸方向の中央値データを格納
float bno_orient_median[3];
float bno_velocity_median[3];
float bno_linearaccel_median[3];
float bno_magnet_median[3];
float bno_gravity_median[3];

//
unsigned long time_100Hz = 0;
int count_10Hz = 0;

// setup()ではdelay()使用可
void setup() {
  Serial.begin(115200);
  //LED
  pinMode(PWM_LED_RED, OUTPUT);
  pinMode(PWM_LED_BLUE, OUTPUT);
  //CAN
  CCP.begin();

  //BME280
  Serial.println(F("BME280 Sensor event test"));

  if (!bme.begin()) {
    while (1)
      Serial.print(F("error"));
    Serial.println(000);
    delay(10);
  }

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
  //BNO055
  Serial.println(F("Hello by BNO055"));

  /* Initialise the sensor */
  if (!bno.begin())  //BNOから応答がない場合
  {
    while (1)
      Serial.print(F("error"));
    Serial.println(001);
    delay(10);  //無限ループでエラーが起きたコードの実行を阻止
  }
}

// loop()と，ここから呼び出される関数ではdelay()使用禁止
void loop() {
  if (millis() - time_100Hz >= 10) {
    time_100Hz += 10;
    count_10Hz++;  //100Hz処理されたときに+1
    if (count_10Hz > 10) {
      count_10Hz = 0;
      // 10Hzで実行する処理
      //BME280]
      sensors_event_t temp_event, pressure_event, humidity_event;
      bme_temp->getEvent(&temp_event);
      bme_pressure->getEvent(&pressure_event);
      bme_humidity->getEvent(&humidity_event);
      get_bme_pressure[count_10Hz] = pressure_event.pressure;  //bmeの生データを配列に格納
      get_bme_temperature[count_10Hz] = temp_event.temperature;
      get_bme_humidity[count_10Hz] = humidity_event.relative_humidity;
      //BNO055
      sensors_event_t orientationData, angVelocityData, linearAccelData, magnetometerData, accelerometerData, gravityData;
      bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
      bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
      bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
      bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
      bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
      bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

      printEvent(&orientationData, orientation_data, count_10Hz);
      printEvent(&angVelocityData, velocity_data, count_10Hz);
      printEvent(&linearAccelData, linearaccel_data, count_10Hz);
      printEvent(&magnetometerData, magnetometer_data, count_10Hz);
      printEvent(&accelerometerData, accel_data, count_10Hz);
      printEvent(&gravityData, gravity_data, count_10Hz);

      //キャリブレーション(センサーや計測機器が正確に測定できるように調整するプロセス)の実行
      uint8_t system, gyro, accel, mag = 0;
      bno.getCalibration(&system, &gyro, &accel, &mag);
      //50HZで実行する処理
      if (count_10Hz %= 50) {

        //BNOの中央値計算50Hz処理
        medianfilter_50Hz(accel_data, bno_accel_median);
        medianfilter_50Hz(orientation_data, bno_orient_median);
        medianfilter_50Hz(velocity_data, bno_velocity_median);
        medianfilter_50Hz(linearaccel_data, bno_linearaccel_median);
        medianfilter_50Hz(magnetometer_data, bno_magnet_median);
        medianfilter_50Hz(gravity_data, bno_gravity_median);

        transfer_to_can(bno_accel_median, fasejudge_xiao);  //BNO055(状態遷移xiaoへCAN送信)
        transfer_to_can(bno_orient_median, logger_xiao);    //BNO(記録部へCAN送信)
        transfer_to_can(bno_velocity_median, logger_xiao);  //BNO(記録部へCAN送信)
        transfer_to_can(bno_magnet_median, logger_xiao);    //BNO(記録部へCAN送信)
        transfer_to_can(bno_gravity_median, logger_xiao);   //BNO(記録部へCAN送信)
        transfer_to_can(bno_accel_median, logger_xiao);     //BNO(記録部へCAN送信)
      }
    }
    //100Hzで実行する処理
    //LED
    digitalWrite(PWM_LED_RED, HIGH);
    pressure_median = medianfilter_100Hz(get_bme_pressure);
    temperature_median = medianfilter_100Hz(get_bme_temperature);
    humidity_median = medianfilter_100Hz(get_bme_humidity);
    ccp.float_to_device(fasejudge_xiao, pressure_median);  //BME280(状態遷移xiaoへCAN送信)
    ccp.float_to_device(logger_xiao, temperature_median);  //BNO(記録部へCAN送信)
    ccp.float_to_device(logger_xiao, humidity_median);     //BNO(記録部へCAN送信)
    digitalWrite(PWM_LED_RED, LOW);
  }
  // 常に実行する処理
  int sensorValue = analogRead(voltagePin);      // アナログ入力から値を読み取る
  float voltage = sensorValue * (5.0 / 1023.0);  // センサ値を電圧に変換（5V基準）
  if (voltage >= thresholdVoltage) {             //参考に
    Serial.println(1);                           // しきい値を超えたら1を出力
  } else {
    Serial.println(2);  // しきい値を超えなければ2を出力
  }
}

//50Hz中央値計算
void medianfilter_50Hz(float dataholder[5][3], float datamedian[3]) {
  float temp[5];

  for (int i = 0; i < 3; i++) {  // i=0でx軸,i=1でy軸,i=2でz軸に分類
    for (int j = 0; j < 5; j++) {
      temp[j] = dataholder[j][i];
    }
    datamedian[i] = findMedian(temp, 5);
  }
  return 0;
}
//100Hz中央値計算
float medianfilter_100Hz(float get_data[]) {
  return findMedian(get_data, 10);
}
//中央値を求めるためにソート
float findMedian(float arr[], int n) {
  // 一時的な配列をソート
  sortArray(arr, n);

  // 中央値を返す
  if (n % 2 == 0) {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  } else {
    return arr[n / 2];
  }
}

void sortArray(float arr[], int n) {
  for (int i = 0; i < n - 1; i++) {
    for (int j = 0; j < n - i - 1; j++) {
      if (arr[j] > arr[j + 1]) {
        // Swap arr[j] and arr[j + 1]
        float temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
  return 0;
}
//CANへのデータ送信(配列)
void transfer_to_can(float mediandata, int canid_name) {
  for (int i = 0; i < 3; i++) {
    ccp.float_to_device(canid_name, mediandata[i]);
  }
  return 0;
}
//get_bno055data
void printEvent(sensors_event_t *event, float dataholder[][3], int count) {
  double x = -1000000, y = -1000000, z = -1000000;

  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_ORIENTATION) {
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  } else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  } else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  } else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else if (event->type == SENSOR_TYPE_GRAVITY) {
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  } else {
    Serial.print("Unk:");
  }
  dataholder[count][0] = x;
  dataholder[count][1] = y;
  dataholder[count][2] = z;
}

/*
error_number
000:bme280 no detected
001:bno055 no detected
*/
