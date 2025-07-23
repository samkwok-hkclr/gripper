#include "vacuum_gripper/vacuum_gripper.hpp"

void VacuumGripper::insert_pressure(const uint8_t* data)
{
  if (!data) 
  {
    RCLCPP_ERROR(get_logger(), "Null pointer in %s", __FUNCTION__);
    return;
  }

  // According to STM32 implementation to decode the value
  float pressure = static_cast<int16_t>((data[1] << 8) | data[0]) * 1000.0f;
  RCLCPP_DEBUG(get_logger(), "pressure: %f", pressure);

  const std::lock_guard<std::mutex> lock(mutex_);
  tracker_.insert_pressure(pressure);
}

void VacuumGripper::pub_pressure_cb(void)
{
  FluidPressure msg;
  msg.header.stamp = get_clock()->now();
  msg.header.frame_id = "tcp";

  if (simulation_)
  {
    msg.fluid_pressure = -77.7;
    msg.variance = 0.7;
  }
  else
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    msg.fluid_pressure = tracker_.back();
    msg.variance = tracker_.variance();
  }
  
  pressure_pub_->publish(msg);
}
