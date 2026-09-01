# pragma once
// Linux library:
#include <stdio.h>
#include "serialib.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ROS library
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
// ROS Msg
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/time_reference.hpp"
#include <cstdint>

// To use the communication library, we need to include the following
// two header files:
#include "EasyObjectDictionary.h" // TransducerM Data
#include "EasyProfile.h"          // TransducerM communication protocol

using namespace std::chrono_literals;

// Debug switch:
//#define  DEBUG_MODE

class TMSerial : public rclcpp::Node
{
public:
  TMSerial();
  ~TMSerial();
  
private:
  void TimerCallback();
  void StatusRequestCallback();
  char SerialportOpen();
  bool OnSerialRX();
  void FillCovarianceMatrices();
  void PublishTransform();
  void UpdateSensorTimestamp(uint32_t raw_timestamp_us);
  #ifdef DEBUG_MODE
  rclcpp::TimerBase::SharedPtr timer_10;
  void    TimerCallback2();
  int     count;
  int     count2;
  #endif

private:
  serialib* serialib1;  // We use linux serialib to interface with serial port.

  sensor_msgs::msg::Imu imu_data_msg;
  sensor_msgs::msg::MagneticField imu_data_rpy_msg; // RPY msg uses the same data structure as msg::MagneticField
  sensor_msgs::msg::MagneticField imu_data_mag_msg;
  sensor_msgs::msg::Temperature imu_temperature_msg;
  sensor_msgs::msg::TimeReference imu_sensor_time_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_IMU;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_RPY;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr publisher_IMU_MAG;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr publisher_IMU_TEMP;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr publisher_IMU_SENSOR_TIME;

  bool sensor_time_updated_{false};
  bool temperature_updated_{false};
  bool combo_seen_logged_{false};
  bool status_seen_logged_{false};

  bool sensor_id_known_{false};
  EP_ID_TYPE_ sensor_short_id_{EP_ID_UNSPECIFIED_};

  bool sensor_timestamp_initialized_{false};
  uint32_t last_sensor_timestamp_raw_{0};
  uint64_t sensor_timestamp_wrap_count_{0};
  uint64_t sensor_timestamp_us_{0};

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

