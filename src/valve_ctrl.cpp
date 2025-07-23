#include "vacuum_gripper/vacuum_gripper.hpp"

void VacuumGripper::insert_valve_state(const uint8_t* data)
{
  if (!data) 
  {
    RCLCPP_ERROR(get_logger(), "Null pointer in %s", __FUNCTION__);
    return;
  }
  
  const std::lock_guard<std::mutex> lock(mutex_);
  RCLCPP_DEBUG(get_logger(), "1: %s, 2: %s", 
    data[0] ? "On" : "Off",
    data[1] ? "On" : "Off");

  gripper_status[0] = static_cast<bool>(data[0]);
  gripper_status[1] = static_cast<bool>(data[1]);
} 
 
void VacuumGripper::valve_status_cb(void)
{
  Bool msg;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    msg.data = gripper_status[0];
  }
  
  status_pub_->publish(msg);
}

void VacuumGripper::valve_control_cb(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (simulation_)
  {
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      gripper_status[0] = request->data;
    }
    response->success = true;
    RCLCPP_WARN(get_logger(), "Completed simulation valve control");
    return;
  }

  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
 
  msg.id = RPDO_1 + can_id_;
  msg.dlc = 2;

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    // Send non-zero value turn-on the valve
    // Turn on both channel
    msg.data[0] = request->data ? 0xFF : 0x0;
    msg.data[1] = request->data ? 0xFF : 0x0;
  }

  tpdo_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "Completed valve control");
  response->success = true;
}