#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <my_custom_message/msg/motor.h>

geometry_msgs__msg__Twist msg;
my_custom_message__msg__Motor motor;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t encoder_publisher;
rcl_node_t node;
rcl_timer_t timer;

// 모터 및 엔코더 핀 정의
#define LEFT_MOTOR_PWM  19
#define LEFT_MOTOR_DIR1 18
#define LEFT_MOTOR_DIR2 5
#define LEFT_ENCODER_A  39
#define LEFT_ENCODER_B  36

// 모터 및 엔코더 상수
#define PPR 11
#define GEAR_RATIO 90
#define ENCODER_FACTOR (2.0 * M_PI / (PPR * GEAR_RATIO * 2)) // 각속도 변환 계수
#define D 0.0875 // 휠의 지름 (m)
#define R (D / 2) // 휠의 반지름 (m)

// 제어 변수
float target_w = 0.0;  // 목표 각속도 (rad/s)
float measured_w = 0.0; // 실제 측정된 각속도
float error = 0.0;      // 오차 값
float pwm_output = 0.0; // PWM 값 (컨트롤러 출력)
float last_pwm_output = 0.0; // **수렴 시 유지할 PWM 값 저장 변수**

// **P + I 제어기 변수**
float Kp = 100.0;  // 비례 게인
float Ki = 10.0;   // 적분 게인

float integral = 0.0;   // 적분항

// PWM 제한
#define PWM_MIN 60
#define PWM_MAX 240
#define ERROR_THRESHOLD 0.05 // **수렴 기준 0.05 rad/s 이하**

volatile long left_pulse_count = 0;
unsigned long last_time = 0;
unsigned long last_control_time = 0;

// 에러 매크로
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}


// 오류 발생 시 루프
void error_loop() {
  while(1) {
    Serial.println("Error occurred!");
    delay(100);
  }
}

// **엔코더 인터럽트 핸들러**
void IRAM_ATTR left_encoder_ISR() {
  if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
    left_pulse_count++;
  } else {
    left_pulse_count--;
  }
}

// **모터 속도 설정 함수**
void setMotor(float speed) {
    int pwm_value = abs(speed);
    pwm_value = constrain(pwm_value, PWM_MIN, PWM_MAX);  

    Serial.print("Target w: "); Serial.print(target_w);
    Serial.print(" | Measured w: "); Serial.print(measured_w);
    Serial.print(" | Error: "); Serial.print(error);
    Serial.print(" | PWM: "); Serial.println(pwm_value);

    if (speed > 0) { // 전진
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    } else if (speed < 0) { // 후진
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    } else { // 정지
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }

    analogWrite(LEFT_MOTOR_PWM, pwm_value);
}

// **ROS 2에서 `/cmd_vel` 메시지 수신 시 실행되는 콜백 함수**
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear_x = msg->linear.x;
    if (abs(linear_x) < 0.01) linear_x = 0.0;

    target_w = linear_x / R;  // **목표 각속도 계산**
    
    Serial.print("Received linear_x: ");
    Serial.print(linear_x);
    Serial.print(" | Calculated target_w: ");
    Serial.println(target_w);
}

// **엔코더 값을 각속도로 변환하는 함수**
void update_motor_velocity() {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_time) / 1000.0; // 초 단위 변환

    measured_w = (left_pulse_count * ENCODER_FACTOR) / delta_time; // 실제 각속도 계산
    left_pulse_count = 0;
    last_time = current_time;

    motor.left_w = measured_w; // 엔코더 값 퍼블리싱용 저장
    Serial.print("Measured w: ");
    closed_loop_control();
}

// **P + I 기반 Closed Loop Control (수렴 시 PWM 유지)**
void closed_loop_control() {
    unsigned long current_time = millis();
    float delta_time = (current_time - last_control_time) / 1000.0; // 초 단위 변환
    last_control_time = current_time;

    error = target_w - measured_w;     // 오차 계산

    // **P + I 제어기 계산**
    integral += error * delta_time; // 적분항
    pwm_output = (Kp * error) + (Ki * integral); // P + I 적용

    // **수렴 조건: error < 0.05 rad/s이면 PWM 유지**
    if (abs(error) < ERROR_THRESHOLD) {
        pwm_output = last_pwm_output; // 마지막 PWM 값 유지
    } else {
        last_pwm_output = pwm_output; // PWM 업데이트
    }

    setMotor(pwm_output);  // 모터 제어
}
// **모터 속도, imu 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
  }
}


// **setup() 함수**
void setup() {
    set_microros_wifi_transports("HOTPOT", "18123030", "192.168.98.124", 8888);
    Serial.begin(115200);
    delay(2000);

    // 모터 및 엔코더 핀 설정
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR2, OUTPUT);
    pinMode(LEFT_ENCODER_A, INPUT);
    pinMode(LEFT_ENCODER_B, INPUT);
    attachInterrupt(LEFT_ENCODER_A, left_encoder_ISR, CHANGE);
     //micro_ros 설정 부분
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    RCCHECK(rclc_subscription_init_default(&subscriber,&node,ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),"/cmd_vel"));
    //엔코더를 통한 속도값 퍼블리셔
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    //RCCHECK(rclc_executor_set_trigger(&executor,rclc_executor_trigger_one, &subscriber));
    Serial.println("micro-ROS node started.");
   
}

// **loop() 함수**
void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
    delay(50);                // 주기 조절
}
