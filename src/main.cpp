#include <Arduino.h>
#include <math.h>

/* ================= micro-ROS ================= */
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

/* ================= micro-ROS globals ================= */
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

rcl_publisher_t odom_pub;
rcl_subscription_t cmd_vel_sub;
rclc_executor_t executor;

nav_msgs_msg_Odometry odom_msg;
geometry_msgs_msg_Twist cmd_vel_msg;

/* ================= cmd_vel ================= */
float vx_ros = 0, vy_ros = 0, wz_ros = 0;
unsigned long last_ros_ms = 0;
const unsigned long ROS_TIMEOUT = 700;

/* ================= Robot config ================= */
constexpr int MOTOR_COUNT = 4;
const uint8_t MOTOR_L[MOTOR_COUNT] = {13, 27, 21, 12};
const uint8_t MOTOR_R[MOTOR_COUNT] = {25, 23, 19, 18};

const uint8_t ENC_A[MOTOR_COUNT] = {32, 33, 4, 5};
const uint8_t ENC_B[MOTOR_COUNT] = {15, 35, 2, 22};
const int8_t ENC_DIR[MOTOR_COUNT] = {+1, -1, +1, -1};

volatile long encCount[MOTOR_COUNT] = {0};

/* ================= Encoder ISR ================= */
void updateEncoder(uint8_t i){
  bool same = digitalRead(ENC_A[i]) == digitalRead(ENC_B[i]);
  encCount[i] += same ? ENC_DIR[i] : -ENC_DIR[i];
}
void enc1(){ updateEncoder(0); }
void enc2(){ updateEncoder(1); }
void enc3(){ updateEncoder(2); }
void enc4(){ updateEncoder(3); }

/* ================= Motor ================= */
void driveMotor(uint8_t L, uint8_t R, int pwm){
  pwm = constrain(pwm, -255, 255);
  if(pwm > 0){ analogWrite(L, pwm); analogWrite(R, 0); }
  else if(pwm < 0){ analogWrite(L, 0); analogWrite(R, -pwm); }
  else { analogWrite(L, 0); analogWrite(R, 0); }
}

/* ================= ODOM ================= */
float odom_x = 0, odom_y = 0, odom_yaw = 0;
uint64_t last_odom_ns = 0;

const float WHEEL_RADIUS = 0.05f;
const float WHEEL_BASE   = 0.05f;
const float CPR = 2048.0;

long lastEnc[4] = {0};

void update_odometry(float dt){
  long d[4];
  for(int i=0;i<4;i++){
    d[i] = encCount[i] - lastEnc[i];
    lastEnc[i] = encCount[i];
  }

  float v1 = d[0] * 2 * PI * WHEEL_RADIUS / (CPR * dt);
  float v2 = d[1] * 2 * PI * WHEEL_RADIUS / (CPR * dt);
  float v3 = d[2] * 2 * PI * WHEEL_RADIUS / (CPR * dt);
  float v4 = d[3] * 2 * PI * WHEEL_RADIUS / (CPR * dt);

  float vx = (v1 + v2 + v3 + v4) / 4.0;
  float vy = (-v1 + v2 + v3 - v4) / 4.0;
  float wz = (-v1 + v2 - v3 + v4) / (4.0 * WHEEL_BASE);

  odom_yaw += wz * dt;

  odom_x += (vx * cos(odom_yaw) - vy * sin(odom_yaw)) * dt;
  odom_y += (vx * sin(odom_yaw) + vy * cos(odom_yaw)) * dt;

  odom_msg.twist.twist.linear.x = vx;
  odom_msg.twist.twist.linear.y = vy;
  odom_msg.twist.twist.angular.z = wz;
}

/* ================= cmd_vel callback ================= */
void cmd_vel_callback(const void *msg){
  auto *m = (const geometry_msgs_msg_Twist *)msg;
  vx_ros = m->linear.x;
  vy_ros = m->linear.y;
  wz_ros = m->angular.z;
  last_ros_ms = millis();
}

/* ================= setup ================= */
void setup(){
  Serial.begin(115200);
  delay(2000);

  /* micro-ROS transport */
  set_microros_wifi_transports(
    "TP-Link_2.4GHz_D447AA",
    "",
    "192.168.0.5",
    8888
  );

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_robot_node", "", &support);

  rmw_uros_sync_session(500);

  rclc_publisher_init_default(
    &odom_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odom"
  );

  rclc_subscription_init_default(
    &cmd_vel_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &cmd_vel_sub, &cmd_vel_msg,
    &cmd_vel_callback, ON_NEW_DATA
  );

  /* Motor & encoder init */
  for(int i=0;i<MOTOR_COUNT;i++){
    pinMode(MOTOR_L[i], OUTPUT);
    pinMode(MOTOR_R[i], OUTPUT);
    pinMode(ENC_A[i], INPUT);
    pinMode(ENC_B[i], INPUT);
  }

  attachInterrupt(digitalPinToInterrupt(ENC_A[0]), enc1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[1]), enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[2]), enc3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A[3]), enc4, CHANGE);

  last_odom_ns = rmw_uros_epoch_nanos();
}

/* ================= loop ================= */
void loop(){
  rclc_executor_spin_some(&executor, 0);

  /* Motor control from ROS */
  driveMotor(MOTOR_L[0], MOTOR_R[0], (vy_ros - vx_ros - wz_ros) * 200);
  driveMotor(MOTOR_L[1], MOTOR_R[1], (vy_ros + vx_ros + wz_ros) * 200);
  driveMotor(MOTOR_L[2], MOTOR_R[2], (vy_ros - vx_ros + wz_ros) * 200);
  driveMotor(MOTOR_L[3], MOTOR_R[3], (vy_ros + vx_ros - wz_ros) * 200);

  /* ODOM */
  int64_t now_ns = rmw_uros_epoch_nanos();
  float dt = (now_ns - last_odom_ns) * 1e-9f;

  if(dt > 0.002f){
    last_odom_ns = now_ns;
    update_odometry(dt);

    odom_msg.header.stamp.sec = now_ns / 1000000000ULL;
    odom_msg.header.stamp.nanosec = now_ns % 1000000000ULL;
    odom_msg.header.frame_id.data = (char *)"odom";
    odom_msg.header.frame_id.size = strlen("odom");

    odom_msg.child_frame_id.data = (char *)"base_footprint";
    odom_msg.child_frame_id.size = strlen("base_footprint");


    odom_msg.pose.pose.position.x = odom_x;
    odom_msg.pose.pose.position.y = odom_y;
    odom_msg.pose.pose.orientation.z = sin(odom_yaw * 0.5);
    odom_msg.pose.pose.orientation.w = cos(odom_yaw * 0.5);

    rcl_publish(&odom_pub, &odom_msg, NULL);
  }
}