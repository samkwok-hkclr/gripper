#ifndef VACUUM_GRIPPER_NODE_HPP__
#define VACUUM_GRIPPER_NODE_HPP__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "canopen_interfaces/msg/co_data.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class ValveControl : public rclcpp::Node
{
  using Bool = std_msgs::msg::Bool;
  using SetBool = std_srvs::srv::SetBool;
  using COData = canopen_interfaces::msg::COData;

public:
  explicit ValveControl(const rclcpp::NodeOptions& options);
  ~ValveControl() = default;

  bool is_valid_rpdo_index(uint16_t index) const;

  void status_cb(void);
  
  void rpdo_cb(const COData::SharedPtr msg);

  void valve_control_cb(
    const std::shared_ptr<SetBool::Request> request, 
    std::shared_ptr<SetBool::Response> response);

private:
  bool simulation_;

  std::mutex mutex_;

  bool gripper_status;

  rclcpp::CallbackGroup::SharedPtr status_cbg_;
  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;

  rclcpp::TimerBase::SharedPtr status_pub_timer_;

  rclcpp::Publisher<Bool>::SharedPtr status_pub_;
  rclcpp::Publisher<COData>::SharedPtr tpdo_pub_;

  rclcpp::Subscription<COData>::SharedPtr rpdo_sub_;

  rclcpp::Service<SetBool>::SharedPtr ctrl_srv_;

  std::unordered_set<uint16_t> VALID_RPDO_INDEXES;
  
};

#endif