#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>
#include <my_custom_message/msg/motor.h>

rcl_subscription_t subscriber;
nav_msgs__msg__Odometry msg;
my_custom_message__msg__Motor motor;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t publisher;
rcl_node_t node;
rcl_timer_t timer;

#define LEFT_MOTOR_PWM  19   // 왼쪽 모터 PWM 핀
#define LEFT_MOTOR_DIR1 18  // 왼쪽 모터 방향 핀 1
#define LEFT_MOTOR_DIR2 5  // 왼쪽 모터 방향 핀 2

#define RIGHT_MOTOR_PWM 4  // 오른쪽 모터 PWM 핀
#define RIGHT_MOTOR_DIR1 16 // 오른쪽 모터 방향 핀 1
#define RIGHT_MOTOR_DIR2 17 // 오른쪽 모터 방향 핀 2

#define LEFT_ENCODER_A 39  // 왼쪽 엔코더 A 채널
#define LEFT_ENCODER_B 36  // 왼쪽 엔코더 B 채널
#define RIGHT_ENCODER_A 35 // 오른쪽 엔코더 A 채널
#define RIGHT_ENCODER_B 34 // 오른쪽 엔코더 B 채널

#define WHEEL_SEPARATION 0.30  // 바퀴 간 거리 (30cm)
#define PPR 11                  // 엔코더 펄스
#define GEAR_RATIO 90           // 감속비
#define ENCODER_FACTOR (2.0 * M_PI / (PPR * GEAR_RATIO * 2)) // 각속도 변환 계수
#define SPEED_CORRECTION_KP 0.1 //오른쪽 모터 속도 보정 값(PID 제어에서 P만)

volatile int left_pulse_count = 0;
volatile int right_pulse_count = 0;
unsigned long last_time = 0;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

// 오류 발생 시 루프
void error_loop(){
  while(1){
    Serial.println("not prepare yet");
    delay(100);
  }
}

// **엔코더 인터럽트 핸들러 (A, B 채널)**
void IRAM_ATTR left_encoder_ISR() {
  if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
    left_pulse_count++;
  } else {
    left_pulse_count--;
  }
}

void IRAM_ATTR right_encoder_ISR() {
  if (digitalRead(RIGHT_ENCODER_A) == digitalRead(RIGHT_ENCODER_B)) {
    right_pulse_count++;
  } else {
    right_pulse_count--;
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
    const nav_msgs__msg__Odometry * msg = (const nav_msgs__msg__Odometry *)msgin;

    float linear_x = msg->twist.twist.linear.x;  
    float angular_z = msg->twist.twist.angular.z;
    //오도메트리의 너무 작은 값 방지.
    if (abs(linear_x) < 0.01) linear_x = 0.0;
    if (abs(angular_z) < 0.01) angular_z = 0.0;

    // 시리얼 출력 (디버깅용)
    Serial.print("Received /cmd_vel | Linear X: ");
    Serial.print(linear_x);
    Serial.print(" | Angular Z: ");
    Serial.println(angular_z);

    float left_speed = linear_x - (angular_z * WHEEL_SEPARATION / 2.0);
    float right_speed = linear_x + (angular_z * WHEEL_SEPARATION / 2.0);
    // 디버깅 출력
    Serial.print("Left Speed: "); Serial.print(left_speed);
    Serial.print(" | Right Speed: "); Serial.println(right_speed);

    setMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, left_speed);
    setMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2, right_speed);
}

// **엔코더 값을 각속도로 변환하는 함수**
void update_motor_velocity() {
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time) / 1000.0; // 초 단위 변환

  float left_angular_velocity = (left_pulse_count * ENCODER_FACTOR) / delta_time;
  float right_angular_velocity = (right_pulse_count * ENCODER_FACTOR) / delta_time;

  // ROS 2 메시지로 엔코더 값 발행
  motor.left_linear_velocity = left_angular_velocity;
  motor.right_linear_velocity = right_angular_velocity;

  left_pulse_count = 0;
  right_pulse_count = 0;
  last_time = current_time;
}

// **모터 속도 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    RCSOFTCHECK(rcl_publish(&publisher,&motor,NULL));
  }
}

// **ESP32 초기 설정**
void setup() {
  set_microros_wifi_transports("HY-DORM5-658", "residence658", "192.168.0.8", 8888);
  Serial.begin(115200);
  delay(2000);
  // 모터 핀을 출력 모드로 설정
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);
  //엔코더 핀모드
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  //2체배 방식 채널 A의 rising, falling모두 인식
  attachInterrupt(LEFT_ENCODER_A, left_encoder_ISR, CHANGE);
  attachInterrupt(RIGHT_ENCODER_A, right_encoder_ISR, CHANGE);

  allocator = rcl_get_default_allocator();

  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs,msg,Odometry),"/odom"));
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));

  RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  //RCCHECK(rclc_executor_set_trigger(&executor,rclc_executor_trigger_one, &subscriber));
  Serial.println("micro-ROS node started.");
}

void loop() {
  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
