#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//motor 관련부
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
// left
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
#define MAX_MOTOR_RPM 110  // 모터 최대 RPM
#define MAX_PWM 255 // 9v = 255, 12v = 240
#define MIN_PWM 0  // 모터가 동작하는 최소 PWM 값

// **PID 제어 변수**
//12V
//float Kp_L = 3.1;  // 기존보다 높임
//float Ki_L = 4.5;   // 적분항 증가
//float Kd_L = 0.05;  // 미세 조정

//float Kp_R = 3.7;  // 기존보다 높임
//float Ki_R = 4.5;   // 적분항 증가
//float Kd_R = 0.05;  // 미세 조정

//9V
float Kp_L = 3.5;  // 기존보다 높임
float Ki_L = 4.5;   // 적분항 증가
float Kd_L = 0.05;  // 미세 조정

float Kp_R = 3.7;  // 기존보다 높임
float Ki_R = 4.5;   // 적분항 증가
float Kd_R = 0.05;  // 미세 조정

float desired_left_speed = 0.0;
float desired_right_speed = 0.0;

float target_omega_left = 0.0;  
float target_omega_right = 0.0;     // 목표 각속도 (rad/s)
float measured_w_left = 0.0;    
float measured_w_right = 0.0;      // 현재 측정된 바퀴 각속도 (rad/s)
float error_left = 0.0, last_error_left = 0.0, left_integral = 0.0;
float error_right = 0.0, last_error_right = 0.0, right_integral = 0.0;
float control_output_left = 0.0;
float control_output_right = 0.0;
int motor_pwm = 0;

// **엔코더 측정 변수**
volatile int left_encoder_count = 0;
volatile int right_encoder_count = 0;
unsigned long last_time_left = 0;
unsigned long last_time_right = 0;
//엔코더 offset변수
float left_offset_count = 0.0;
float right_offset_count = 0.0;



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

void offset() {
    Serial.println("🚀 엔코더 오프셋 보정 시작...");
    delay(1000); // 안정화 대기

    int sum_left = 0, sum_right = 0;
    int sample_count = 20;  // 20개 샘플 측정
    int interval = 50; // 50ms 간격


    for (int i = 0; i < sample_count; i++) {
        sum_left += left_encoder_count;
        sum_right += right_encoder_count;
        delay(interval);
    }

    left_offset_count = sum_left / sample_count;
    right_offset_count = sum_right / sample_count;

    Serial.print("Left Encoder Offset: ");
    Serial.println(left_offset_count);
    Serial.print(" Right Encoder Offset: ");
    Serial.println(right_offset_count);
}

// **ROS 2에서 `/cmd_vel` 메시지 수신 시 실행되는 콜백 함수**
void subscription_callback(const void *msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

    float desired_speed_mps = msg->linear.x;  
    float order_w = msg->angular.z;
    //오도메트리의 너무 작은 값 방지.
    if (abs(desired_speed_mps) < 0.01) desired_speed_mps = 0.0;
    if (abs(order_w) < 0.01) order_w = 0.0;
    // 목표 각속도 계산 (rad/s)
    desired_left_speed = desired_speed_mps + (order_w*WHEEL_SEPARATION)/2;
    desired_right_speed = desired_speed_mps - (order_w*WHEEL_SEPARATION)/2;

    target_omega_left = desired_left_speed / R;
    target_omega_right = desired_right_speed / R;
    
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
// **PWM을 RPM 값으로 변환하는 함수 (새로운 공식 적용)**
// left
int calculatePWM_LEFT(float rpm) {
    float pwm = (abs(rpm) / 110.0) * (255);  // 9v = 255, 12v = 240
    return constrain((int)pwm, MIN_PWM, MAX_PWM);
}
// right
int calculatePWM_RIGHT(float rpm) {
    float pwm = (abs(rpm) / 110.0) * (255);  // 9v = 255, 12v = 240
    return constrain((int)pwm, MIN_PWM, MAX_PWM);
}

// **모터 속도 제어 함수**
// LEFT
void leftsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (target_omega_left > 0) {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    } else if (target_omega_left < 0) {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }
    Serial.println(abs(pwm));
    analogWrite(LEFT_MOTOR_PWM, abs(pwm));
}
// RIGHT
void rightsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (target_omega_right > 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    } else if (target_omega_right < 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    }
    Serial.println(abs(pwm));
    analogWrite(RIGHT_MOTOR_PWM, abs(pwm));
}

// 속도 측정 및 PID 제어 실행
// left
void updateleftControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_left) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // 디버깅: 엔코더 카운트 확인
    Serial.print("Encoder Count: ");
    Serial.println(left_encoder_count);

    // 현재 속도 측정 (rad/s)
    measured_w_left = ((left_encoder_count-left_offset_count) * ENCODER_FACTOR) / dt;
    left_encoder_count = 0; // 카운트 리셋
    //  오차 계산
    error_left = target_omega_left - measured_w_left;
    Serial.print("target_left");
    Serial.println(target_omega_left);
    Serial.print("error_left");
    Serial.println(error_left);
    Serial.print("measured_left");
    Serial.println(measured_w_left);

    
    // PID 계산
    left_integral += error_left * dt;
    float derivative = (error_left - last_error_left) / dt;
    control_output_left = (Kp_L * error_left) + (Ki_L * left_integral) + (Kd_L * derivative);
    
    last_error_left = error_left;
    last_time_left = current_time;

    // **디버깅: PID 출력을 확인**
    //Serial.print("Control Output_left: ");
    //Serial.println(control_output_left);

    // PWM 변환 (새로운 공식 사용)
    float target_rpm = (control_output_left / (2 * M_PI)) * 60;  // rad/s → RPM 변환
    motor_pwm = calculatePWM_LEFT(target_rpm);

    // 모터에 PWM 적용
    leftsetMotorPWM(motor_pwm);
}
// right
void updaterightControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_right) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // **디버깅: 엔코더 카운트 확인**
    //Serial.print("Encoder Count: ");
    //Serial.println(right_encoder_count);

    // 1. **현재 속도 측정 (rad/s)**
    measured_w_right = ((right_encoder_count-right_offset_count) * ENCODER_FACTOR) / dt;
    right_encoder_count = 0; // 카운트 리셋

    // 2. **오차 계산**
    error_right = target_omega_right - measured_w_right;
    Serial.print("target_right");
    Serial.println(target_omega_right);
    Serial.print("error_right");
    Serial.println(error_right);
    Serial.print("measured_right");
    Serial.println(measured_w_right);
    
    // 3. **PID 계산**
    right_integral += error_right * dt;
    float derivative = (error_right - last_error_right) / dt;
    control_output_right = (Kp_R * error_right) + (Ki_R * right_integral) + (Kd_R * derivative);
    
    last_error_right = error_right;
    last_time_right = current_time;

    // 4. **PWM 변환 (새로운 공식 사용)**
    float target_rpm = (control_output_right / (2 * M_PI)) * 60;  // rad/s → RPM 변환
    motor_pwm = calculatePWM_RIGHT(target_rpm);

    // 5. **모터에 PWM 적용**
    rightsetMotorPWM(motor_pwm);
}

void update_motor_velocity() {
  // ROS 2 메시지로 엔코더 값 발행
  motor.left_w = measured_w_left;
  motor.right_w = measured_w_right;
  motor.left_target_w = target_omega_left;
  motor.right_target_w = target_omega_right;
}
// **모터 속도, imu 정보 발행 콜백함수**
void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
  RCLC_UNUSED(last_call_time);
  if (timer != NULL){
    update_motor_velocity();
    RCSOFTCHECK(rcl_publish(&encoder_publisher,&motor,NULL));
  }
}

void setup() {
    set_microros_wifi_transports("HOTPOT","18123030", "192.168.98.124", 8888);
    //set_microros_wifi_transports("HY-DORM5-658","residence658", "192.168.0.8", 8888);
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
    //offset();
    Serial.println("offset done");
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


void loop() {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
    updateleftControlLoop();
    updaterightControlLoop();
    delay(50);
}
