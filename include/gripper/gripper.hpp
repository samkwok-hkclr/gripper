#ifndef GRIPPER_HPP__
#define GRIPPER_HPP__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"

#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/range.hpp"

#include "can_msgs/msg/frame.hpp"

#include "robotic_platform_msgs/msg/gripper_status.hpp"
#include "robotic_platform_msgs/srv/control_gripper.hpp"

#include "fluid_pressure_tracker.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class Gripper : public rclcpp::Node
{
  using Bool = std_msgs::msg::Bool;

  using Trigger = std_srvs::srv::Trigger;
  using SetBool = std_srvs::srv::SetBool;

  using FluidPressure = sensor_msgs::msg::FluidPressure;
  using Temperature = sensor_msgs::msg::Temperature;
  using Range = sensor_msgs::msg::Range;

  using Frame = can_msgs::msg::Frame;
  
  using GripperStatus = robotic_platform_msgs::msg::GripperStatus;
  using ControlGripper = robotic_platform_msgs::srv::ControlGripper;

public:
  explicit Gripper(const rclcpp::NodeOptions& options);
  ~Gripper();
  
  void insert_pressure(float pressure);
  void insert_range(float range, float temp);

  void vac_state_cb(void);
  void pub_pressure_cb(void);
  void pub_range_cb(void);
  void gripper_status_cb(void);
  void release_pressure_cb(void);
  void auto_init_gripper_cb(void);

  void tpdo_1_cb(const uint8_t* data);
  void tpdo_2_cb(const uint8_t* data);
  void tpdo_3_cb(const uint8_t* data);
  void tpdo_4_cb(const uint8_t* data);
  void nmt_cb(const uint8_t* data);

  void rpdo_cb(const Frame::SharedPtr msg);

  void ctrl_ultrasonic_sensor(bool enable);

  void pump_ctrl_cb(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);

  void init_gripper_cb(
    const std::shared_ptr<Trigger::Request> request, 
    std::shared_ptr<Trigger::Response> response,
    bool re_init);
  void ctrl_gripper_cb(
    const std::shared_ptr<ControlGripper::Request> request, 
    std::shared_ptr<ControlGripper::Response> response);

private:
  std::mutex mutex_;

  bool simulation_;
  uint8_t can_id_;
  std::string can_ns_;
  std::string can_interface_;

  float field_of_view_;
  float min_range_;
  float max_range_;
  float range_;
  float temp_;
  bool enable_ultrasonic_;
  
  uint8_t min_gripper_force_;
  uint8_t max_gripper_force_;
  float min_gripper_position_;
  float max_gripper_position_;
  std::atomic<bool> gripper_is_initialized{false};
  std::atomic<uint8_t> gripper_state{0};
  std::atomic<uint16_t> gripper_position{0};
  std::atomic<uint16_t> gripper_force{20};

  std::atomic<bool> suction_gripper_state{false};

  FluidPressureTracker tracker_;

  rclcpp::CallbackGroup::SharedPtr status_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;

  rclcpp::TimerBase::SharedPtr state_pub_timer_;
  rclcpp::TimerBase::SharedPtr pressure_pub_timer_;
  rclcpp::TimerBase::SharedPtr range_pub_timer_;
  rclcpp::TimerBase::SharedPtr gripper_status_pub_timer_;
  rclcpp::TimerBase::SharedPtr release_pressure_timer_;
  rclcpp::TimerBase::SharedPtr auto_init_gripper_timer_;

  rclcpp::Publisher<Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<Range>::SharedPtr range_pub_;
  rclcpp::Publisher<Temperature>::SharedPtr temp_pub_;
  rclcpp::Publisher<FluidPressure>::SharedPtr pressure_pub_;
  rclcpp::Publisher<Frame>::SharedPtr tpdo_pub_;
  rclcpp::Publisher<GripperStatus>::SharedPtr gripper_pub_;

  rclcpp::Subscription<Frame>::SharedPtr rpdo_sub_;

  rclcpp::Service<SetBool>::SharedPtr ctrl_suction_srv_;
  rclcpp::Service<Trigger>::SharedPtr init_gripper_srv_;
  rclcpp::Service<Trigger>::SharedPtr re_init_gripper_srv_;
  rclcpp::Service<ControlGripper>::SharedPtr ctrl_gripper_srv_;

  std::unordered_map<uint16_t, std::function<void(const uint8_t*)>> valid_cob_frame_;

  constexpr static uint16_t TPDO_1 = 0x180;
  constexpr static uint16_t TPDO_2 = 0x280;
  constexpr static uint16_t TPDO_3 = 0x380;
  constexpr static uint16_t TPDO_4 = 0x480;

  constexpr static uint16_t RPDO_1 = 0x200;
  constexpr static uint16_t RPDO_2 = 0x300;
  constexpr static uint16_t RPDO_3 = 0x400;
  constexpr static uint16_t RPDO_4 = 0x500;

  constexpr static uint16_t NMT = 0x700;

  constexpr static uint16_t MAX_GRIPPER_UNITS = 1000;
};

#endif // GRIPPER_HPP__