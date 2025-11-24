#include "gripper/gripper.hpp"

void Gripper::ctrl_ultrasonic_sensor(bool enable)
{
  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  msg.id = RPDO_3 + can_id_;
  msg.dlc = 1;

  msg.data[0] = enable ? 0xFF : 0x0;

  tpdo_pub_->publish(msg);
  
  RCLCPP_INFO(get_logger(), "%s ultrasonic sensor", enable ? "Enable" : "Disable");
}

void Gripper::insert_range(float range, float temp)
{
  RCLCPP_DEBUG(get_logger(), "range: %.4f, temp: %.1f", range, temp);

  const std::lock_guard<std::mutex> lock(mutex_);
  range_ = range;
  temp_ = temp;
}

void Gripper::pub_range_cb(void)
{
  if (!range_pub_ || range_pub_->get_subscription_count() == 0)
    return;

  if (!temp_pub_ || temp_pub_->get_subscription_count() == 0)
    return;

  Range range_msg;
  range_msg.header.stamp = get_clock()->now();
  range_msg.header.frame_id = "ultrasonic_link";
  range_msg.radiation_type = Range::ULTRASOUND;
  
  Temperature temp_msg;
  temp_msg.header.stamp = get_clock()->now();
  temp_msg.header.frame_id = "ultrasonic_link";

  if (simulation_)
  {
    range_msg.field_of_view = 0.007;
    range_msg.min_range = 0.07;
    range_msg.max_range = 0.77;
    range_msg.range = 0.7;

    temp_msg.temperature = 25.52;
  }
  else
  {
    const std::lock_guard<std::mutex> lock(mutex_);

    range_msg.field_of_view = field_of_view_;
    range_msg.min_range = min_range_;
    range_msg.max_range = max_range_;
    range_msg.range = range_;

    temp_msg.temperature = temp_;
  }

  range_pub_->publish(range_msg);
  temp_pub_->publish(temp_msg);
}