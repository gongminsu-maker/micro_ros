#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define LEFT_MOTOR_PWM  19   // 왼쪽 모터 PWM 핀
#define LEFT_MOTOR_DIR1 18  // 왼쪽 모터 방향 핀 1
#define LEFT_MOTOR_DIR2 5  // 왼쪽 모터 방향 핀 2

#define RIGHT_MOTOR_PWM 4  // 오른쪽 모터 PWM 핀
#define RIGHT_MOTOR_DIR1 16 // 오른쪽 모터 방향 핀 1
#define RIGHT_MOTOR_DIR2 17 // 오른쪽 모터 방향 핀 2

#define WHEEL_SEPARATION 0.30  // 바퀴 간 거리 (30cm)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// 오류 발생 시 루프
void error_loop(){
  while(1){
    Serial.println("No twist yet");
    delay(100);
  }
}

// **모터 속도 및 방향 설정 함수**
void setMotor(int pwm_pin, int dir1_pin, int dir2_pin, float speed) {
    int pwm_value = constrain(abs(speed * 255), 0, 255);

    // 시리얼 출력 (디버깅용)
    Serial.print("Motor PWM Pin: "); Serial.print(pwm_pin);
    Serial.print(" | Speed: "); Serial.print(speed);
    Serial.print(" | PWM Value: "); Serial.println(pwm_value);

    if (speed > 0) { // 전진
        digitalWrite(dir1_pin, HIGH);
        digitalWrite(dir2_pin, LOW);
    } else if (speed < 0) { // 후진
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, HIGH);
    } else { // 정지
        digitalWrite(dir1_pin, LOW);
        digitalWrite(dir2_pin, LOW);
    }

    analogWrite(pwm_pin, pwm_value);
}

// **ROS 2에서 `/cmd_vel` 메시지 수신 시 실행되는 콜백 함수**
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    // ROS2 Twist 메시지 값 가져오기
    float linear_x = msg->linear.x;  // 전진/후진 속도
    float angular_z = msg->angular.z; // 회전 속도

    // 시리얼 출력 (디버깅용)
    Serial.print("Received /cmd_vel | Linear X: ");
    Serial.print(linear_x);
    Serial.print(" | Angular Z: ");
    Serial.println(angular_z);

    // **차동 구동 속도 변환 공식 적용**
    float left_speed = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
    float right_speed = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);

    // 디버깅 출력
    Serial.print("Left Speed: "); Serial.print(left_speed);
    Serial.print(" | Right Speed: "); Serial.println(right_speed);

    // 모터 제어 함수 호출
    setMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, left_speed);
    setMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2, right_speed);
}

// **ESP32 초기 설정**
void setup() {
  set_microros_wifi_transports("HY-DORM5-658", "residence658", "192.168.0.8", 8888);
  Serial.begin(115200);
  delay(2000);

  // **모터 핀을 출력 모드로 설정**
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

  allocator = rcl_get_default_allocator();

  // ROS 2 노드 및 구독자 설정
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"));

  // 실행기 설정
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

  Serial.println("micro-ROS node started.");
}

// **ROS 2에서 오는 메시지를 계속 체크**
void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
