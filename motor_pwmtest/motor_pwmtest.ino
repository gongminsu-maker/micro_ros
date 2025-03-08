#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//motor 관련부

#include <my_custom_message/msg/motor.h>


my_custom_message__msg__Motor motor;
rcl_subscription_t subscriber;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_publisher_t encoder_publisher;
rcl_node_t node;
rcl_timer_t timer;

#define LEFT_MOTOR_PWM  19
#define LEFT_MOTOR_DIR1 18
#define LEFT_MOTOR_DIR2 5
#define LEFT_ENCODER_A  39
#define LEFT_ENCODER_B  36
// right
#define RIGHT_MOTOR_PWM 4 
#define RIGHT_MOTOR_DIR1 16 
#define RIGHT_MOTOR_DIR2 17
#define RIGHT_ENCODER_A 35 
#define RIGHT_ENCODER_B 34

// **모터 및 엔코더 상수**
#define PPR 11
#define GEAR_RATIO 90
#define ENCODER_FACTOR (2.0 * M_PI / (PPR * GEAR_RATIO * 2)) // 각속도 변환 계수
#define D 0.0875 // 휠의 지름 (m)
#define R 0.04375 // 휠의 반지름 (m)
#define WHEEL_SEPARATION 0.261 

// **엔코더 측정 변수**
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;
int pwm = 100;
float measured_w_left = 0.0;
float measured_w_right = 0.0;
unsigned long last_time = 0;
// 에러 매크로
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// offset

// 오류 발생 시 루프
void error_loop(){
  while(1){
    Serial.println("not prepare yet");
    delay(100);
  }
}

// 인터럽트 서비스 루틴(ISR)
// left
void IRAM_ATTR left_encoder_ISR() {
    if (digitalRead(LEFT_ENCODER_A) != digitalRead(LEFT_ENCODER_B)) {
        left_encoder_count++; 
    } else {
        left_encoder_count--; 
    }
}
// right
void IRAM_ATTR right_encoder_ISR() {
    if (digitalRead(RIGHT_ENCODER_A) != digitalRead(RIGHT_ENCODER_B)) {
        right_encoder_count--; 
    } else {
        right_encoder_count++; 
    }
}
void update_motor_velocity() {
  // ROS 2 메시지로 엔코더 값 발행
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0; // 초 단위 변환
    // 현재 속도 측정 (rad/s)
  measured_w_left = ((left_encoder_count) * ENCODER_FACTOR) / dt;
  measured_w_right = ((right_encoder_count) * ENCODER_FACTOR) / dt;
  right_encoder_count = 0; // 카운트 리셋
  left_encoder_count = 0; // 카운트 리셋
  last_time = current_time;

  motor.left_w = measured_w_left;
  motor.right_w = measured_w_right;
}
// **모터 속도, imu 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
  }
}
void setpwm(){
  digitalWrite(LEFT_MOTOR_DIR1, HIGH);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  Serial.println(abs(pwm));
  analogWrite(LEFT_MOTOR_PWM, abs(pwm));
  analogWrite(RIGHT_MOTOR_PWM, abs(pwm));
}

void setup() {
    //set_microros_wifi_transports("HOTPOT","18123030", "192.168.98.124", 8888);
    set_microros_wifi_transports("HY-DORM5-658","residence658", "192.168.0.8", 8888);
    //set_microros_wifi_transports("KT_GiGA_5D35","ahb66kz314", "172.30.1.58", 8888);
    //set_microros_wifi_transports("I_phone","dlgksrufdlek", "192.168.166.124", 8888);
    Serial.begin(115200);
    delay(2000);

    // 모터 핀 설정
    pinMode(LEFT_MOTOR_PWM, OUTPUT);
    pinMode(LEFT_MOTOR_DIR1, OUTPUT);
    pinMode(LEFT_MOTOR_DIR2, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
    pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

    // 엔코더 핀 설정 및 인터럽트 활성화
    pinMode(LEFT_ENCODER_A, INPUT);
    pinMode(LEFT_ENCODER_B, INPUT);
    pinMode(RIGHT_ENCODER_A, INPUT);
    pinMode(RIGHT_ENCODER_B, INPUT);
    attachInterrupt(LEFT_ENCODER_A, left_encoder_ISR, CHANGE);
    attachInterrupt(RIGHT_ENCODER_A, right_encoder_ISR, CHANGE);
     //micro_ros 설정 부분
    allocator = rcl_get_default_allocator();

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));
    RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(my_custom_message, msg, Motor), "motor_topic"));
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback));
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    //RCCHECK(rclc_executor_set_trigger(&executor,rclc_executor_trigger_one, &subscriber));
    Serial.println("micro-ROS node started.");
}


void loop() {
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
  setpwm();
  delay(50);
}

