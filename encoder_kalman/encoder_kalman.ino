#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <my_custom_message/msg/motor.h>

// ROS2 객체 선언
geometry_msgs__msg__Twist msg;
geometry_msgs__msg__Twist corrected_msg;
my_custom_message__msg__Motor motor;

rcl_subscription_t subscriber_cmd_vel;      // 키보드 명령용 cmd_vel 구독
rcl_subscription_t subscriber_corrected;    // 보정된 cmd_vel_corrected 구독
rcl_publisher_t encoder_publisher;          // 엔코더 데이터 퍼블리셔
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;

// 모터 핀 설정
#define LEFT_MOTOR_PWM  19
#define LEFT_MOTOR_DIR1 18
#define LEFT_MOTOR_DIR2 5
#define RIGHT_MOTOR_PWM 4
#define RIGHT_MOTOR_DIR1 16
#define RIGHT_MOTOR_DIR2 17

#define LEFT_ENCODER_A 39  // 왼쪽 엔코더 A 채널
#define LEFT_ENCODER_B 36  // 왼쪽 엔코더 B 채널
#define RIGHT_ENCODER_A 35 // 오른쪽 엔코더 A 채널
#define RIGHT_ENCODER_B 34 // 오른쪽 엔코더 B 채널

// 바퀴 간 거리 및 엔코더 설정
#define WHEEL_SEPARATION 0.30
#define PPR 11
#define GEAR_RATIO 90
#define ENCODER_FACTOR (2.0 * M_PI / (PPR * GEAR_RATIO * 2))

// 엔코더 관련 변수
volatile int left_pulse_count = 0;
volatile int right_pulse_count = 0;
unsigned long last_time = 0;

// **속도 저장 변수**
float last_linear_x = 0.0;
float last_angular_z = 0.0;
float corrected_linear_x = 0.0;
float corrected_angular_z = 0.0;

// **보정 속도 반영 비율**
#define ALPHA 0.2  // 보정된 속도를 점진적으로 반영 (20%)
// 에러 매크로
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop() { while(1){ Serial.println("Error!"); delay(100); }}

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

// **모터 속도 및 방향 설정**
void setMotor(int pwm_pin, int dir1_pin, int dir2_pin, float speed) {
    int pwm_value = constrain(abs(speed * 255), 0, 255);
    digitalWrite(dir1_pin, speed > 0 ? HIGH : LOW);
    digitalWrite(dir2_pin, speed < 0 ? HIGH : LOW);
    analogWrite(pwm_pin, pwm_value);
    Serial.print(pwm_pin);
    Serial.println(pwm_value);
}

// **키보드 입력(cmd_vel) 수신**
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    last_linear_x = msg->linear.x;
    last_angular_z = msg->angular.z;

    Serial.print("Received /cmd_vel | Linear X: ");
    Serial.print(last_linear_x);
    Serial.print(" | Angular Z: ");
    Serial.println(last_angular_z);

    float left_speed = last_linear_x - (last_angular_z * WHEEL_SEPARATION / 2.0);
    float right_speed = last_linear_x + (last_angular_z * WHEEL_SEPARATION / 2.0);

    setMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, left_speed);
    setMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2, right_speed);
}

// **보정된 cmd_vel_corrected 수신**
void corrected_cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    corrected_linear_x = msg->linear.x;
    corrected_angular_z = msg->angular.z;

    // 최종 명령 생성: 기존 명령과 보정된 명령을 혼합
    float final_linear_x = (1 - ALPHA) * last_linear_x + ALPHA * corrected_linear_x;
    float final_angular_z = (1 - ALPHA) * last_angular_z + ALPHA * corrected_angular_z;

    Serial.print("Corrected /cmd_vel_corrected | Linear X: ");
    Serial.print(corrected_linear_x);
    Serial.print(" | Angular Z: ");
    Serial.println(corrected_angular_z);

    float left_speed = final_linear_x - (final_angular_z * WHEEL_SEPARATION / 2.0);
    float right_speed = final_linear_x + (final_angular_z * WHEEL_SEPARATION / 2.0);

    setMotor(LEFT_MOTOR_PWM, LEFT_MOTOR_DIR1, LEFT_MOTOR_DIR2, left_speed);
    setMotor(RIGHT_MOTOR_PWM, RIGHT_MOTOR_DIR1, RIGHT_MOTOR_DIR2, right_speed);
}

// **엔코더 값을 각속도로 변환하는 함수**
void update_motor_velocity() {
  unsigned long current_time = millis();
  float delta_time = (current_time - last_time) / 1000.0; // 초 단위 변환

  float left_angular_v = (left_pulse_count * ENCODER_FACTOR) / delta_time;
  float right_angular_v = (right_pulse_count * ENCODER_FACTOR) / delta_time;

  // ROS 2 메시지로 엔코더 값 발행
  motor.left_w= left_angular_v;
  motor.right_w = right_angular_v;

  left_pulse_count = 0;
  right_pulse_count = 0;
  last_time = current_time;
}

// **모터 속도 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
  }
}

// **ROS2 설정**
void setup() {
  
  Serial.begin(115200);
  set_microros_wifi_transports("HY-DORM5-658", "residence658", "192.168.0.6", 8888);
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
    //micro_ros 설정 부분
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  RCCHECK(rclc_subscription_init_default(&subscriber_cmd_vel, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&subscriber_corrected, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel_corrected"));
  RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback));
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_corrected, &corrected_msg, &corrected_cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  Serial.println("micro-ROS node started.");
}

void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
