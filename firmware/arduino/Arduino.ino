#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// micro-ROS 相关
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN  150
#define SERVOMAX  600
// 宏定义Wi-Fi链接和ROSAgentport
#define WIFISSD "WHEELTEC_S300"
#define WIFIPASSWORD "dongguan"
#define ROSHOSTIP "192.168.0.100"
#define ROSAGENTPORT 8888

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg = 
    (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // 根据接收到的数据控制舵机
  for (size_t i = 0; i < msg->data.size && i < 16; i++) {
    int pulse = map(msg->data.data[i], 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(i, 0, pulse);
  }
}

void setup() {
  // 初始化 I2C 和 PCA9685
  Wire.begin(8, 9);  // ESP32-S3 的 I2C 引脚，根据实际连接修改
  pwm.begin();
  pwm.setPWMFreq(50);

  // micro-ROS 传输层设置（串口或WiFi）
  set_microros_wifi_transports();
  
  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_pca9685_node", "", &support);

  // 初始化消息内存
  msg.data.capacity = 16;
  msg.data.size = 0;
  msg.data.data = (int32_t *)malloc(16 * sizeof(int32_t));

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "servo_commands"
  );

  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(
    &executor, &subscriber, &msg, 
    &subscription_callback, ON_NEW_DATA
  );
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}