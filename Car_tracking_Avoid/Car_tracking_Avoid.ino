#include <Wire.h>                     // 包含Wire(I2C)通讯库 Include Wire library
#include <Adafruit_PWMServoDriver.h>  // 包含Adafruit PWMServoDriver库 Include Adafruit PWMServoDriver library

// 定义电机控制引脚 Define motor control pins
#define Motor_L1_F_PIN 11  // 控制小车左前方电机前进 Control the motor on the left front of the car
#define Motor_L1_B_PIN 10  // 控制小车左前方电机后退 Control the motor back on the left front of the car
#define Motor_L2_F_PIN 8   // 控制小车左后方电机前进 Control car left rear motor forward
#define Motor_L2_B_PIN 9   // 控制小车左后方电机后退 Control the car left rear motor back
#define Motor_R1_F_PIN 13  // 控制小车右前方电机前进 Control the right front motor of the car to move forward
#define Motor_R1_B_PIN 12  // 控制小车右前方电机后退 Control the motor back on the right front of the car
#define Motor_R2_F_PIN 14  // 控制小车右后方电机前进 Control car right rear motor forward
#define Motor_R2_B_PIN 15  // 控制小车右后方电机后退 Control car right rear motor back

// 定义底层驱动芯片参数 Bottom-layer driver chip related parameters
#define Bottom_Layer_Driver_ADDR 0x40

// 定义PWM频率 Define PWM frequency
#define PWM_FREQUENCY 50

// 定义按键引脚和控制状态 Define pin and key(button) states
#define KEY_PIN 7
#define Press_KEY 0
#define Release_KEY 1

// 定义三路循迹模块引脚 Define the three-way line patrol module pins
#define L_TRACK_PIN A2
#define M_TRACK_PIN A1
#define R_TRACK_PIN A0

// 定义超声波模块控制引脚 Define ultrasonic control pins
#define TRIG_PIN 11
#define ECHO_PIN 12

// 定义巡线阈值 Define the patrol threshold
const int Threshold = 500;

// 定义距离阈值 Define distance thresholds
const float NearDistance = 15.0;

// 定义小车速度和超时参数 Define car speed and timeout parameters
int iCarSpeed = 50;
unsigned int uTimeOut = 0;

// 定义全局变量 Define global variables
float Distance = 0.0;
bool bCar_Switch = false;

// 枚举小车工作状态 Enumerate car working states
enum CarState {
  CAR_STOP,               // 停止状态 Stop state
  CAR_INITIAL_ROTATING,   // 初始旋转270度阶段 Initial 270-degree rotation stage
  CAR_SEARCH_LINE_LEFT,   // 左转寻找黑线阶段 Left turn searching for black line stage
  CAR_SEARCH_LINE_RIGHT,  // 右转寻找黑线阶段 Right turn searching for black line stage
  CAR_TRACKING,           // 巡线阶段 Line tracking stage
  CAR_AVOIDING            // 避障绕行阶段 Obstacle avoidance stage
};

CarState currentCarState = CAR_STOP;  // 当前小车状态 Current car state

// 枚举全向小车的常见运动方式 Enumerate the common movement modes of omnidirectional cars
enum OmniDirectionalCar {
  STOP,
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  LEFT_ROTATE,
  RIGHT_ROTATE,
  LEFT_FORWARD,
  RIGHT_BACKWARD,
  RIGHT_FORWARD,
  LEFT_BACKWARD,
};

// 创建Adafruit_PWMServoDriver类的实例 Create an instance of the Adafruit_PWMServoDriver class
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(Bottom_Layer_Driver_ADDR);

// 定义旋转和避障相关参数 Define rotation and avoidance related parameters
unsigned long rotateStartTime = 0;
const unsigned long ROTATE_270_TIME = 2500;  // 旋转270度所需时间 Time required for 270-degree rotation

unsigned long avoidStartTime = 0;
const unsigned long AVOID_TURN_TIME = 1400;   // 避障转弯时间 Obstacle avoidance turning time
const unsigned long AVOID_FORWARD_TIME = 1400; // 避障前进时间 Obstacle avoidance forward time
int avoidStep = 0;  // 避障步骤 Avoidance step

// 函数声明 Function declarations
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin, int motor_speed);
void setCarMove(uint8_t Movement, int Speed);
int getKeyState(uint8_t pin);
bool setCarSwitch();
float getDistance(int trigPin, int echoPin);
void PatrolCar(int iLeftPin, int iMidPin, int iRightPin);
void startRotationSequence();
void updateRotationState();
bool checkLineDetected();
void performAvoidance();
void searchLineLeft();
void searchLineRight();

void setup() {
  pinMode(KEY_PIN, INPUT_PULLUP);     // 设置按键引脚为上拉输入 Set key pin as input with pull-up
  Wire.begin();                       // 初始化I2C通讯 Initialize I2C communication
  delay(1000);                        // 延时等待系统稳定 Delay for system stability
  pwm.begin();                        // PWM初始化 Initialize the Pulse Width Modulation (PWM) library
  pwm.setPWMFreq(PWM_FREQUENCY);      // 设置PWM频率 Set the PWM frequency
  setCarMove(STOP, 0);                // 设置小车停止状态 Set the car to stop state
}

void loop() {
  // 按键控制小车功能启停 Key control car function start and stop
  if (setCarSwitch()) {
    // 根据当前状态执行相应操作 Execute corresponding operations according to current state
    switch (currentCarState) {
      case CAR_INITIAL_ROTATING:
        updateRotationState();
        break;
      case CAR_SEARCH_LINE_LEFT:
        searchLineLeft();
        break;
      case CAR_SEARCH_LINE_RIGHT:
        searchLineRight();
        break;
      case CAR_TRACKING:
        Car_Track_Avoid();
        break;
      case CAR_AVOIDING:
        performAvoidance();
        break;
      default:
        setCarMove(STOP, 0);
        break;
    }
  } else {
    currentCarState = CAR_STOP;
    setCarMove(STOP, 0);
  }
}

/**
 * @brief 设置单个电机速度 Setting the Motor Speed
 * @param motor_forward_pin: 控制电机前进引脚 Control the motor forward pin
 * @param motor_backward_pin: 控制电机后退引脚 Control the motor backward pin
 * @param motor_speed: 设置电机速度 Setting the Motor Speed
 * @retval 无 None
 */
void setMotorSpeed(uint16_t motor_forward_pin, uint16_t motor_backward_pin, int motor_speed) {
  motor_speed = map(motor_speed, -255, 255, -4095, 4095);
  if (motor_speed >= 0) {
    pwm.setPWM(motor_forward_pin, 0, motor_speed);
    pwm.setPWM(motor_backward_pin, 0, 0);
  } else if (motor_speed < 0) {
    pwm.setPWM(motor_forward_pin, 0, 0);
    pwm.setPWM(motor_backward_pin, 0, -(motor_speed));
  }
}

/**
 * @brief 设置小车运动方式和速度 Set the car movement mode and speed
 * @param Movement: 小车运动方式 Car movement
 * @param Speed: 小车运动速度 Car speed
 * @retval 无 None
 */
void setCarMove(uint8_t Movement, int Speed) {
  switch (Movement) {
    case STOP:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case LEFT:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case RIGHT:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case LEFT_ROTATE:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case RIGHT_ROTATE:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    case LEFT_FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case RIGHT_BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, -Speed);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, -Speed);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
    case RIGHT_FORWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, Speed);
      break;
    case LEFT_BACKWARD:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, -Speed);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, -Speed);
      break;
    default:
      setMotorSpeed(Motor_L1_F_PIN, Motor_L1_B_PIN, 0);
      setMotorSpeed(Motor_L2_F_PIN, Motor_L2_B_PIN, 0);
      setMotorSpeed(Motor_R1_F_PIN, Motor_R1_B_PIN, 0);
      setMotorSpeed(Motor_R2_F_PIN, Motor_R2_B_PIN, 0);
      break;
  }
}

/**
 * @brief 获取按键状态 Get key(button) status
 * @param pin: 按键控制引脚 Control key(button) pins
 * @retval 按键状态 Key(button) Status
 */
int getKeyState(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW)
        ;
      return Press_KEY;
    }
    return Release_KEY;
  } else {
    return Release_KEY;
  }
}

/**
 * @brief 获取超声波距离 Get ultrasonic distance
 * @param trigPin: 触发测距引脚 Trigger pin
 * @param echoPin: 接收测距引脚 Echo pin
 * @retval 转换的距离 Measured distance (cm)
 */
float getDistance(int trigPin, int echoPin) {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance = (duration * 0.034 / 2);
  return distance;
}

/**
 * @brief 小车巡线功能 Car line patrol function
 * @param iLeftPin: 左边循迹传感器 Left tracking sensor
 * @param iMidPin: 中间循迹传感器 Middle tracking sensor
 * @param iRightPin: 右边循迹传感器 Right tracking sensor
 * @retval 无 None
 */
void PatrolCar(int iLeftPin, int iMidPin, int iRightPin) {
  int leftValue = analogRead(iLeftPin);
  int middleValue = analogRead(iMidPin);
  int rightValue = analogRead(iRightPin);

  // 修正逻辑：数值越大表示越黑（在线上的值） Corrected logic: larger value means darker (on the line)
  bool leftOnLine = (leftValue > Threshold);    // 左边传感器在黑线上 Left sensor on black line
  bool midOnLine = (middleValue > Threshold);   // 中间传感器在黑线上 Middle sensor on black line
  bool rightOnLine = (rightValue > Threshold);  // 右边传感器在黑线上 Right sensor on black line

  if (!leftOnLine && midOnLine && !rightOnLine) {
    // ○●○ 线在中间：直行 Line in middle: go straight
    setCarMove(FORWARD, iCarSpeed);
  }
  else if (leftOnLine && midOnLine && !rightOnLine) {
    // ●●○ 线偏左：向左转 Line偏向left: turn left
    setCarMove(LEFT, iCarSpeed);
  }
  else if (!leftOnLine && midOnLine && rightOnLine) {
    // ○●● 线偏右：向右转 Line偏向right: turn right
    setCarMove(RIGHT, iCarSpeed);
  }
  else if (leftOnLine && !midOnLine && !rightOnLine) {
    // ●○○ 只有左边在线：大角度左转 Only left on line: sharp left turn
    setCarMove(LEFT_ROTATE, iCarSpeed);
  }
  else if (!leftOnLine && !midOnLine && rightOnLine) {
    // ○○● 只有右边在线：大角度右转 Only right on line: sharp right turn
    setCarMove(RIGHT_ROTATE, iCarSpeed);
  }
  else if (leftOnLine && midOnLine && rightOnLine) {
    // ●●● 全在线内：直行 All on line: go straight
    setCarMove(FORWARD, iCarSpeed);
  }
  else {
    // ○○○ 全在线外：寻线策略 All off line: search strategy
    uTimeOut++;
    delay(20);
    if (uTimeOut > 100) {
      uTimeOut = 0;
      setCarMove(STOP, 0);
    } else {
      setCarMove(LEFT_ROTATE, iCarSpeed/2);
    }
  }
}

/**
 * @brief 小车巡线避障功能 Tracking and obstacle avoidance function
 * @param 无 None
 * @retval 无 None
 */
void Car_Track_Avoid() {
  Distance = getDistance(TRIG_PIN, ECHO_PIN);
  delay(10);
  
  if (Distance > NearDistance) {
    // 前方安全，继续巡线 Front safe, continue line tracking
    PatrolCar(L_TRACK_PIN, M_TRACK_PIN, R_TRACK_PIN);
  } else {
    // 检测到障碍物，开始避障绕行 Obstacle detected, start avoidance maneuver
    currentCarState = CAR_AVOIDING;
    avoidStartTime = millis();
    avoidStep = 0;
  }
}

/**
 * @brief 左转寻找黑线 Search for black line by turning left
 * @param 无 None
 * @retval 无 None
 */
void searchLineLeft() {
  setCarMove(LEFT_ROTATE, iCarSpeed);
  if (checkLineDetected()) {
    currentCarState = CAR_TRACKING;
  }
}

/**
 * @brief 右转寻找黑线 Search for black line by turning right
 * @param 无 None
 * @retval 无 None
 */
void searchLineRight() {
  setCarMove(RIGHT, iCarSpeed);
  if (checkLineDetected()) {
    currentCarState = CAR_TRACKING;
  }
}

/**
 * @brief 检查是否检测到黑线 Check if black line is detected
 * @param 无 None
 * @retval 是否检测到黑线 true if line detected
 */
bool checkLineDetected() {
  int middleValue = analogRead(M_TRACK_PIN);
  return (middleValue > Threshold);
}

/**
 * @brief 开始旋转序列 Start rotation sequence
 * @param 无 None
 * @retval 无 None
 */
void startRotationSequence() {
  currentCarState = CAR_INITIAL_ROTATING;
  rotateStartTime = millis();
}

/**
 * @brief 更新旋转状态 Update rotation state
 * @param 无 None
 * @retval 无 None
 */
void updateRotationState() {
  unsigned long elapsedTime = millis() - rotateStartTime;
  
  if (elapsedTime < ROTATE_270_TIME) {
    setCarMove(LEFT_ROTATE, iCarSpeed);
  } else {
    currentCarState = CAR_SEARCH_LINE_LEFT;
    setCarMove(STOP, 0);
    delay(500);
  }
}

/**
 * @brief 执行避障绕行 Perform avoidance maneuver
 * @param 无 None
 * @retval 无 None
 */
void performAvoidance() {
  unsigned long elapsedTime = millis() - avoidStartTime;
  
  switch (avoidStep) {
    case 0:  // 第一步：向左转避开障碍物 Step 1: Turn left to avoid obstacle
      if (elapsedTime < AVOID_TURN_TIME) {
        setCarMove(LEFT, iCarSpeed);
      } else {
        avoidStep = 1;
        avoidStartTime = millis();
      }
      break;
      
    case 1:  // 第二步：前进一段距离 Step 2: Move forward for a distance
      if (elapsedTime < AVOID_FORWARD_TIME) {
        setCarMove(FORWARD, iCarSpeed);
      } else {
        currentCarState = CAR_SEARCH_LINE_RIGHT;
        setCarMove(STOP, 0);
        delay(500);
      }
      break;
  }
}

/**
 * @brief 设置小车功能开关 Set the car function switch
 * @param 无 None
 * @retval 开启/关闭 true/false
 */
bool setCarSwitch() {
  if (getKeyState(KEY_PIN) == Press_KEY) {
    bCar_Switch = !bCar_Switch;
    if (bCar_Switch) {
      startRotationSequence();
    } else {
      currentCarState = CAR_STOP;
    }
  }
  return bCar_Switch;
}