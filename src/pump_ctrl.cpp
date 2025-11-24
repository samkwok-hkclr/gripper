#include "gripper/gripper.hpp"

void Gripper::vac_state_cb(void)
{
  if (!status_pub_ || status_pub_->get_subscription_count() == 0)
    return;
  
  Bool msg;
  msg.data = suction_gripper_state.load();
  
  status_pub_->publish(msg);
}

void Gripper::pump_ctrl_cb(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (simulation_)
  {
    suction_gripper_state.store(request->data);
    response->success = true;

    if (!request->data)
      release_pressure_timer_->reset();

    RCLCPP_WARN(get_logger(), "Completed simulation pump control");
    return;
  }

  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
 
  msg.id = RPDO_1 + can_id_;
  msg.dlc = 2;

  if (request->data)
  {
    msg.data[0] = 0x1;
    msg.data[1] = 0x0;
    tpdo_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Turn-on");
  }
  else
  {
    msg.data[0] = 0x1;
    msg.data[1] = 0x1;
    tpdo_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Turn-off");
    release_pressure_timer_->reset();
  }

  RCLCPP_INFO(get_logger(), "Completed pump control");
  response->success = true;
}

void Gripper::release_pressure_cb(void)
{
  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  msg.id = RPDO_1 + can_id_;
  msg.dlc = 2;

  msg.data[0] = 0x0;
  msg.data[1] = 0x0;

  tpdo_pub_->publish(msg);

  RCLCPP_INFO(get_logger(), "Released pump pressure");
  release_pressure_timer_->cancel();
}