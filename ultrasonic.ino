#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32.h>  // Publishing float distance

#define TRIG_PIN 5
#define ECHO_PIN 18
#define LED_PIN 13

// Micro-ROS types
rcl_publisher_t publisher;
std_msgs__msg__Float32 distance_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Macros for error handling
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Trigger ultrasonic pulse
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // Read echo time (with timeout of 30ms)
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    if (duration == 0) {
      distance_msg.data = -1.0;  // No echo detected
    } else {
      distance_msg.data = duration * 0.034 / 2.0;  // Convert to cm
    }

    // Publish to ROS 2
    RCSOFTCHECK(rcl_publish(&publisher, &distance_msg, NULL));
  }
}

void setup() {
  // Init serial and pins
  pinMode(LED_PIN, OUTPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);  // Allow system to stabilize

  // Set Micro-ROS transport (serial)
  set_microros_transports();

  // Init allocator
  allocator = rcl_get_default_allocator();

  // Init Micro-ROS support
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Init node
  RCCHECK(rclc_node_init_default(&node, "ultrasonic_node", "", &support));

  // Init publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "ultrasonic_distance"
  ));

  // Init timer to call callback every 500ms
  const unsigned int timer_timeout = 500;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback
  ));

  // Init executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}

