#ifndef VACUUM_SENSOR_NODE_HPP__
#define VACUUM_SENSOR_NODE_HPP__

#pragma once

#include <functional>
#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp"

#include "canopen_interfaces/msg/co_data.hpp"

#include "fluid_pressure_tracker.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class VacuumSensor : public rclcpp::Node
{
  using FluidPressure = sensor_msgs::msg::FluidPressure;
  using COData = canopen_interfaces::msg::COData;

public:
  explicit VacuumSensor(const rclcpp::NodeOptions& options);
  ~VacuumSensor() = default;

  bool is_valid_rpdo_index(uint16_t index) const;

  void pub_pressure_cb(void);

  void rpdo_cb(const COData::SharedPtr msg);

private:
  bool simulation_;

  std::mutex mutex_;

  FluidPressureTracker tracker_;

  rclcpp::CallbackGroup::SharedPtr pub_cbg_;
  rclcpp::CallbackGroup::SharedPtr rpdo_cbg_;

  rclcpp::TimerBase::SharedPtr pressure_pub_timer_;

  rclcpp::Publisher<FluidPressure>::SharedPtr pressure_pub_;

  rclcpp::Subscription<COData>::SharedPtr rpdo_sub_;

  std::unordered_set<uint16_t> VALID_RPDO_INDEXES;

};

#endif