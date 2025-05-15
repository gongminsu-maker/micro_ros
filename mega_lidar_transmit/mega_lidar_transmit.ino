#include <RPLidar.h>
#define RPLIDAR_MOTOR 9 // The PWM pin for control the speed of RPLIDAR's motor (MOTOCTRL).

RPLidar lidar;

float prevAngle = 0.0;
float minDistance = 100000;
float angleAtMinDist = 0;
                      
void setup() {
  Serial.begin(115200);
  Serial1.begin(115200); // For RPLidar
  Serial2.begin(115200); // For esp32
  lidar.begin(Serial1);
  pinMode(RPLIDAR_MOTOR, OUTPUT);  // set pin modes
  analogWrite(RPLIDAR_MOTOR, 250); // motor on
  delay(1000); // 1s
  lidar.startScan(); // start scanning
  Serial.print("lidarstart");

}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    //perform data processing here... 
    float distance = lidar.getCurrentPoint().distance;
    float angle = lidar.getCurrentPoint().angle;  // 0-360 deg
    int quality = lidar.getCurrentPoint().quality;
    int  startBit = lidar.getCurrentPoint().startBit;

    // 유효한 데이터만 전송 (distance > 0 , quality 10 ~ 15이상 추천한다네)
    if (distance > 0 && quality > 10) {
      Serial2.print(distance, 2); // mm, 소수점 2자리
      Serial2.print(",");
      Serial2.print(angle, 2); // degree, 소수점 2자리
      Serial2.print(",");
      Serial2.print(quality);
      Serial2.print(",");
      Serial2.println(startBit);
      
    }
  }

  //else {
  //  analogWrite(RPLIDAR_MOTOR, 0); // LIDAR 동작 안 하면 모터 OFF
  //  rplidar_response_device_info_t info;
  //  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
  //    lidar.startScan();
  //    analogWrite(RPLIDAR_MOTOR, 255);
  //    delay(100);
  //  }
  //}
}
