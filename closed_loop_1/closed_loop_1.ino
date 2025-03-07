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
#define MAX_PWM 240
#define MIN_PWM 65  // 모터가 동작하는 최소 PWM 값

// **PID 제어 변수**
float Kp_L = 4.0;  // 기존보다 높임
float Ki_L = 5.0;   // 적분항 증가
float Kd_L = 0.05;  // 미세 조정

float Kp_R = 4.0;  // 기존보다 높임
float Ki_R = 5.0;   // 적분항 증가
float Kd_R = 0.05;  // 미세 조정

float desired_speed_mps = 0.3; // 목표 속도 (m/s)
float order_w = 0.0;
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
    float pwm = (rpm / 110.0) * (240 - 65) + 65;  // 240 -> 255로 수정
    return constrain((int)pwm, MIN_PWM, MAX_PWM);
}
// right
int calculatePWM_RIGHT(float rpm) {
    float pwm = (rpm / 110.0) * (240 - 65) + 65;  // 240 -> 255로 수정
    return constrain((int)pwm, MIN_PWM, MAX_PWM);
}

// **모터 속도 제어 함수**
// LEFT
void leftsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (pwm > 0) {
        digitalWrite(LEFT_MOTOR_DIR1, HIGH);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    } else if (pwm < 0) {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(LEFT_MOTOR_DIR1, LOW);
        digitalWrite(LEFT_MOTOR_DIR2, LOW);
    }

    analogWrite(LEFT_MOTOR_PWM, abs(pwm));
}
// RIGHT
void rightsetMotorPWM(int pwm) {
    pwm = constrain(pwm, MIN_PWM, MAX_PWM);

    if (pwm > 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    } else if (pwm < 0) {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, HIGH);
    } else {
        digitalWrite(RIGHT_MOTOR_DIR1, LOW);
        digitalWrite(RIGHT_MOTOR_DIR2, LOW);
    }

    analogWrite(RIGHT_MOTOR_PWM, abs(pwm));
}

// **속도 측정 및 PID 제어 실행**
// left
void updateleftControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_left) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // 디버깅: 엔코더 카운트 확인
    Serial.print("Encoder Count: ");
    Serial.println(left_encoder_count);

    // 1. 현재 속도 측정 (rad/s)
    measured_w_left = (left_encoder_count * ENCODER_FACTOR) / dt;
    left_encoder_count = 0; // 카운트 리셋

    // 2. 오차 계산
    error_left = target_omega_left - measured_w_left;
    
    // 3. PID 계산
    left_integral += error_left * dt;
    float derivative = (error_left - last_error_left) / dt;
    control_output_left = (Kp_L * error_left) + (Ki_L * left_integral) + (Kd_L * derivative);
    
    last_error_left = error_left;
    last_time_left = current_time;

    // **디버깅: PID 출력을 확인**
    Serial.print("Control Output_left: ");
    Serial.println(control_output_left);

    // 4. **PWM 변환 (새로운 공식 사용)**
    float target_rpm = (control_output_left / (2 * M_PI)) * 60;  // rad/s → RPM 변환
    motor_pwm = calculatePWM_LEFT(target_rpm);

    // 5. **모터에 PWM 적용**
    leftsetMotorPWM(motor_pwm);
}
void updaterightControlLoop() {
    unsigned long current_time = millis();
    float dt = (current_time - last_time_right) / 1000.0; // 초 단위 변환
    if (dt < 0.01) return; // 너무 짧은 시간 간격 방지

    // **디버깅: 엔코더 카운트 확인**
    Serial.print("Encoder Count: ");
    Serial.println(right_encoder_count);

    // 1. **현재 속도 측정 (rad/s)**
    measured_w_right = (right_encoder_count * ENCODER_FACTOR) / dt;
    right_encoder_count = 0; // 카운트 리셋

    // 2. **오차 계산**
    error_right = target_omega_right - measured_w_right;
    
    // 3. **PID 계산**
    right_integral += error_right * dt;
    float derivative = (error_right - last_error_right) / dt;
    control_output_right = (Kp_R * error_right) + (Ki_R * right_integral) + (Kd_R * derivative);
    
    last_error_right = error_right;
    last_time_right = current_time;

    // **디버깅: PID 출력을 확인**
    Serial.print("Control Output RIGHT: ");
    Serial.println(control_output_right);
    Serial.print(measured_w_right);

    // 4. **PWM 변환 (새로운 공식 사용)**
    float target_rpm = (control_output_right / (2 * M_PI)) * 60;  // rad/s → RPM 변환
    motor_pwm = calculatePWM_RIGHT(target_rpm);

    // 5. **모터에 PWM 적용**
    rightsetMotorPWM(motor_pwm);
}
void setup() {
    Serial.begin(115200);

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

    // 목표 각속도 계산 (rad/s)
    desired_left_speed = desired_speed_mps + (order_w*WHEEL_SEPARATION)/2;
    desired_right_speed = desired_speed_mps - (order_w*WHEEL_SEPARATION)/2;

    target_omega_left = desired_left_speed / R;
    target_omega_right = desired_right_speed / R;
}

void loop() {
    updateleftControlLoop();
    updaterightControlLoop();
    //Serial.print("order_w: ");
    //Serial.println(order_w);
    //Serial.print("desired_left_speed: ");
    //Serial.println(target_omega_left);
    //Serial.print("desired_right_speed: ");
    //Serial.println(target_omega_right);
    
         // 탭 문자 (값 구분)
    Serial.print(target_omega_left);
    Serial.print("\t");          // 탭 문자
    Serial.print(measured_w_left);
    Serial.print("\t"); 
    Serial.print(target_omega_right);  // Y축 값 1: 목표 각속도
    Serial.print("\t");  
    Serial.println(measured_w_right);

    delay(50);
}
