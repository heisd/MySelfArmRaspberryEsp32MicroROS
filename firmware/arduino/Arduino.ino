#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "config.h"

// micro-ROS 相关
rcl_subscription_t subscriber;
std_msgs__msg__Int32MultiArray msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

void subscription_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  for (size_t i = 0; i < msg->data.size && i < 16; i++) {
    int pulse = map(msg->data.data[i], 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(i, 0, pulse);
  }
}

void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);

  set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);

  delay(2000);

  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_pca9685_node", "", &support);

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
