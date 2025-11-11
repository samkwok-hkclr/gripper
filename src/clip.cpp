#include "gripper/gripper.hpp"

void Gripper::init_gripper_cb(
  const std::shared_ptr<Trigger::Request> request, 
  std::shared_ptr<Trigger::Response> response,
  bool re_init)
{
  (void) request;

  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

  msg.id = RPDO_2 + can_id_;
  msg.dlc = 2;

  msg.data[0] = 0x01;
  msg.data[1] = re_init ? 0xA5 : 0x01;

  tpdo_pub_->publish(msg);

  response->success = true;
  RCLCPP_INFO(get_logger(), "%sInit the gripper", re_init ? "Re-" : "");
}

void Gripper::ctrl_gripper_cb(
  const std::shared_ptr<ControlGripper::Request> request, 
  std::shared_ptr<ControlGripper::Response> response)
{  
  if (request->position < min_gripper_position_ || request->position > max_gripper_position_) 
  {
    RCLCPP_ERROR(get_logger(), "Invalid position value: %.3f. Must be between %.3f and %.3f", 
      request->position, min_gripper_position_, max_gripper_position_);
    response->success = false;
    return;
  }

  if (request->force < min_gripper_force_ || request->force > max_gripper_force_) 
  {
    RCLCPP_WARN(get_logger(), "Force value: %d. Should be between %d and %d", 
      request->force, min_gripper_force_, max_gripper_force_);
    RCLCPP_WARN(get_logger(), "Use default force value: 20");
  }

  auto pub_force = [this](uint8_t force) -> bool {
    Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    msg.id = RPDO_3 + can_id_;
    msg.dlc = 2;

    const uint8_t clamped_force = std::clamp(force, min_gripper_force_, max_gripper_force_);

    msg.data[0] = clamped_force;
    msg.data[1] = 0;
    
    tpdo_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published force command: %d", clamped_force);

    return true;
  };
  
  auto pub_pos = [this](float position) -> bool {
    Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    msg.id = RPDO_4 + can_id_;
    msg.dlc = 2;

    const float clamped_position = std::clamp(position, min_gripper_position_, max_gripper_position_);

    uint16_t position_units = static_cast<uint16_t>(
      clamped_position / max_gripper_position_ * MAX_GRIPPER_UNITS
    );
    position_units = std::min(position_units, MAX_GRIPPER_UNITS);

    msg.data[0] = position_units & 0xFF;
    msg.data[1] = (position_units >> 8) & 0xFF;

    tpdo_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Published position command: %.3f -> %d units", clamped_position, position_units);

    return true;
  };
  
  response->success = pub_force(request->force) && pub_pos(request->position);
  RCLCPP_INFO(get_logger(), "Completed gripper control");
}

void Gripper::gripper_status_cb(void)
{
  if (!gripper_pub_ || gripper_pub_->get_subscription_count() == 0)
    return;

  GripperStatus msg;

  if (simulation_)
  {
    msg.is_initialized = true;
    msg.state = 2;
    msg.force = 20;
    msg.position = 50.0;
  }
  else
  {
    msg.is_initialized = gripper_is_initialized.load();
    msg.state = gripper_state.load();
    msg.force = gripper_force.load();
    msg.position = gripper_position.load();
  }
  
  gripper_pub_->publish(msg);
}

void Gripper::auto_init_gripper_cb(void)
{
  if (!gripper_is_initialized.load())
  {
    Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
    msg.id = RPDO_2 + can_id_;
    msg.dlc = 2;

    msg.data[0] = 0x01;
    msg.data[1] = 0xA5;

    tpdo_pub_->publish(msg);

    RCLCPP_INFO(get_logger(), "Initialized the gripper");

    auto_init_gripper_timer_->cancel();
  }
}