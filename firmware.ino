// Copyright (c) 2021 Juan Miguel Jimeno
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <Servo.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){rclErrorLoop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_subscription_t twist_subscriber;
rcl_publisher_t publisher;

std_msgs__msg__Float32 rpm_msg;
geometry_msgs__msg__Twist twist_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t control_timer;

Servo motor;
Servo steering_servo;

int steer_pin = 22;
int motor_pin = 23;

int steerL = 60;
int steerN = 90;
int steerR = 120;

int motorF = 500;
int motorN = 0;
int motorB = -500;

float last_rpm = 999;

#define MAX_STEERING_ANGLE 1.83  // max steering angle.
#define MIN_STEERING_ANGLE -1.81  // max steering angle.
#define MAX_LINEAR_SPEED 0.26    // max linear speed.
#define max_steering_pwm 120
#define min_steering_pwm 60
#define wheel_d 0.0653

unsigned long long time_offset = 0;
unsigned long prev_cmd_time = 0;
bool micro_ros_init_successful = false;

//    ACKERMANN,
//    MOTOR_MAX_RPM, //16000
//    MAX_RPM_RATIO, //whats this?
//    MOTOR_OPERATING_VOLTAGE, //7.2V
//    MOTOR_POWER_MAX_VOLTAGE, //7.2V
//    0.1588  //LR wheel distance

int LED_PIN = 13;

const byte interruptPin = 19;
int state = 0;
int last = 999;
float lastrpm = 999;
float rpm = 0;
float time_1 = 0;
float time_2 = 0;

// Set message value
//msg.data = 0;

void blink() {
  state++;
  //  Serial.println(state);
}

float get_rpm() {
  if (state == 1) {
    time_1 = millis();
  }
  else if (state >= 3) {
    detachInterrupt(interruptPin);
    time_2 = millis();
    rpm = 60000 / (time_2 - time_1);
    state = 0;
    attachInterrupt(interruptPin, blink, FALLING);
  }

  return rpm;
}

void setup()
{
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  steering_servo.attach(steer_pin);
  motor.attach(motor_pin);

  steering_servo.write(steerN);
  motor.writeMicroseconds(1500 + motorN);

  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(interruptPin, blink, FALLING);

  delay(1000);

  micro_ros_init_successful = false;
  set_microros_transports();
}

void loop()
{
  static unsigned long prev_connect_test_time;
  // check if the agent got disconnected at 10Hz
  if (millis() - prev_connect_test_time >= 100)
  {
    prev_connect_test_time = millis();
    // check if the agent is connected
    if (RMW_RET_OK == rmw_uros_ping_agent(10, 2))
    {
      // reconnect if agent got disconnected or haven't at all
      if (!micro_ros_init_successful)
      {
        createEntities();
      }
    }
    else if (micro_ros_init_successful)
    {
      // stop the robot when the agent got disconnected
      fullStop();
      // clean up micro-ROS components
      destroyEntities();
    }
  }

  if (micro_ros_init_successful)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  }

  //  get_rpm();
  //  if (lastrpm != rpm) {
  //    lastrpm = rpm;
  //    Serial.println(rpm);
  //  }
}

void controlCallback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    moveBase();
    publishData();
  }

}

void twistCallback(const void * msgin)
{
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));

  prev_cmd_time = millis();
}

void createEntities()
{
  allocator = rcl_get_default_allocator();
  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  // create node
  rcl_node_options_t node_ops = rcl_node_get_default_options();
  node_ops.domain_id = 5;
  RCCHECK(rclc_node_init_with_options(&node, "base_node", "", &support, &node_ops));
  //    RCCHECK(rclc_node_init_default(&node, "base_node", "", &support));
  // create twist command subscriber


  RCCHECK(rclc_publisher_init_default(
            &publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
            "rpm"
          ));
  RCCHECK(rclc_subscription_init_default(
            &twist_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "cmd_vel"
          ));
  // create timer for actuating the motors at 50 Hz (1000/20)
  const unsigned int control_timeout = 20;
  RCCHECK(rclc_timer_init_default(
            &control_timer,
            &support,
            RCL_MS_TO_NS(control_timeout),
            controlCallback
          ));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, & allocator));
  RCCHECK(rclc_executor_add_subscription(
            &executor,
            &twist_subscriber,
            &twist_msg,
            &twistCallback,
            ON_NEW_DATA
          ));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));
  // synchronize time with the agent
  syncTime();
  digitalWrite(LED_PIN, HIGH);
  micro_ros_init_successful = true;
}

void destroyEntities()
{
  digitalWrite(LED_PIN, LOW);
  rcl_publisher_fini(&publisher, &node);
  rcl_subscription_fini(&twist_subscriber, &node);
  rcl_node_fini(&node);
  rcl_timer_fini(&control_timer);
  rclc_executor_fini(&executor);
  rclc_support_fini(&support);

  micro_ros_init_successful = false;
}

void fullStop()
{
  twist_msg.linear.x = 0.0;
  //    twist_msg.linear.y = 0.0;
  twist_msg.angular.z = 0.0;
}

void moveBase()
{
  // brake if there's no command received, or when it's only the first command sent
  if (((millis() - prev_cmd_time) >= 200))
  {
    twist_msg.linear.x = 0.0;
    //        twist_msg.linear.y = 0.0;
    twist_msg.angular.z = 0.0;

    digitalWrite(LED_PIN, HIGH);
  }
  //    steer.write();

  float current_steering_angle;
  float current_linear_speed;

  current_steering_angle = steer(twist_msg.angular.z);
  current_linear_speed = spin(twist_msg.linear.x);

}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float steer(float steering_angle)
{
  //steering function for ACKERMANN base
  float servo_steering_angle;

  servo_steering_angle = mapFloat(steering_angle, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE, PI, 0) * (180 / PI);
  servo_steering_angle = mapFloat(servo_steering_angle, 0, 180, min_steering_pwm, max_steering_pwm);

  steering_servo.write(servo_steering_angle);

  return steering_angle;
}

float spin(float lin_speed) {
  float motor_speed;

  motor_speed = mapFloat(lin_speed, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED, motorF, motorB);

  motor.writeMicroseconds(1500 + motor_speed);

  return lin_speed;
}

void publishData()
{
  rpm_msg.data = get_rpm();
  if (last_rpm != rpm_msg.data) {
    last_rpm = rpm_msg.data;
    RCSOFTCHECK(rcl_publish(&publisher, &rpm_msg, NULL));
  }
}

void syncTime()
{
  // get the current time from the agent
  unsigned long now = millis();
  RCCHECK(rmw_uros_sync_session(10));
  unsigned long long ros_time_ms = rmw_uros_epoch_millis();
  // now we can find the difference between ROS time and uC time
  time_offset = ros_time_ms - now;
}

struct timespec getTime()
{
  struct timespec tp = {0};
  // add time difference between uC time and ROS time to
  // synchronize time with ROS
  unsigned long long now = millis() + time_offset;
  tp.tv_sec = now / 1000;
  tp.tv_nsec = (now % 1000) * 1000000;

  return tp;
}

void rclErrorLoop()
{
  while (true)
  {
    flashLED(2);
  }
}

void flashLED(int n_times)
{
  for (int i = 0; i < n_times; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  delay(1000);
}
