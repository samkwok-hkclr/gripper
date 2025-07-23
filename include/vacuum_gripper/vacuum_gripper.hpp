#ifndef VACUUM_GRIPPER_HPP__
#define VACUUM_GRIPPER_HPP__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "can_msgs/msg/frame.hpp"

#include "fluid_pressure_tracker.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VacuumGripper : public rclcpp::Node
{
  using Bool = std_msgs::msg::Bool;
  using SetBool = std_srvs::srv::SetBool;

  using FluidPressure = sensor_msgs::msg::FluidPressure;
  using Temperature = sensor_msgs::msg::Temperature;
  using Range = sensor_msgs::msg::Range;

  using Frame = can_msgs::msg::Frame;

public:
  explicit VacuumGripper(const rclcpp::NodeOptions& options);
  ~VacuumGripper();

  void valve_status_cb(void);
  void pub_pressure_cb(void);
  void pub_range_cb(void);

  void rpdo_cb(const Frame::SharedPtr msg);

  void ctrl_ultrasonic_sensor(bool enable);

  void valve_control_cb(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);

  void insert_pressure(const uint8_t* data);
  void insert_range(const uint8_t* data);
  void insert_valve_state(const uint8_t* data);

private:
  std::mutex mutex_;

  bool simulation_;
  uint8_t can_id_;
  uint8_t no_of_channel_;

  float field_of_view_;
  float min_range_;
  float max_range_;
  float range_;
  float temp_;
  bool enable_ultrasonic_;

  std::vector<bool> gripper_status;

  FluidPressureTracker tracker_;

  rclcpp::CallbackGroup::SharedPtr status_cbg_;
  rclcpp::CallbackGroup::SharedPtr pub_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;

  rclcpp::TimerBase::SharedPtr status_pub_timer_;
  rclcpp::TimerBase::SharedPtr pressure_pub_timer_;
  rclcpp::TimerBase::SharedPtr range_pub_timer_;

  rclcpp::Publisher<Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<Range>::SharedPtr range_pub_;
  rclcpp::Publisher<Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<Frame>::SharedPtr tpdo_pub_;

  rclcpp::Subscription<Frame>::SharedPtr rpdo_sub_;

  rclcpp::Service<SetBool>::SharedPtr ctrl_srv_;

  std::unordered_map<uint16_t, std::function<void(const uint8_t*)>> VALID_FRAME_;

  const uint16_t TPDO_1 = 0x180;
  const uint16_t TPDO_2 = 0x280;
  const uint16_t TPDO_3 = 0x380;
  const uint16_t TPDO_4 = 0x480;

  const uint16_t RPDO_1 = 0x200;
  const uint16_t RPDO_2 = 0x300;
  const uint16_t RPDO_3 = 0x400;
  const uint16_t RPDO_4 = 0x500;

};

#endif // VACUUM_GRIPPER_HPP__