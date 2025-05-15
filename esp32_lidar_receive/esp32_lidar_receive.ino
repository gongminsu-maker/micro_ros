#include <HardwareSerial.h>

HardwareSerial mySerial(2);  // UART2 사용
float prevAngle = 0.0;

void setup() {
  Serial.begin(115200);                               // PC 디버깅용
  mySerial.begin(115200, SERIAL_8N1, 15, 2);          // RX=15, TX=2 (순서 확인!)
  Serial.println("ESP32 Serial2 begin");
}

void loop() {
  if (mySerial.available()) {
    String receivedLine = mySerial.readStringUntil('\n'); // mega가 보낸 한 줄 전체 읽기

    // ',' 기준으로 파싱
    int firstComma = receivedLine.indexOf(','); //indexof() : 특정 문자의 위치 찾기
    int secondComma = receivedLine.indexOf(',', firstComma + 1);
    int thirdComma = receivedLine.indexOf(',', secondComma + 1);


    if (firstComma > 0 && secondComma > firstComma && thirdComma > secondComma) {
      //substring(start, end) : start ~ end 잘라오기.
      float distance = receivedLine.substring(0, firstComma).toFloat(); //string으로 받고 타입에 맞게 바꿔주기
      float angle = receivedLine.substring(firstComma + 1, secondComma).toFloat();
      int quality = receivedLine.substring(secondComma + 1,thirdComma).toInt();
      int startbit = receivedLine.substring(thirdComma + 1).toInt();
    // 한 바퀴 돌았는지 판단(prevAngle -> currentAngle 넘어갈 때)

      Serial.print(distance);
      Serial.print(", ");
      Serial.print(angle);
      Serial.print(", ");
      Serial.print(quality);
      Serial.print(",");
      Serial.println(startbit);
    }
  }
}

