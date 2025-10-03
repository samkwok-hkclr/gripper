#include "vacuum_gripper/vacuum_gripper.hpp"

void VacuumGripper::vac_state_cb(void)
{
  Bool msg;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    msg.data = vac_gripper_state;
  }
  
  status_pub_->publish(msg);
}

void VacuumGripper::vac_ctrl_cb(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  if (simulation_)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      vac_gripper_state = request->data;
    }
    response->success = true;

    RCLCPP_WARN(get_logger(), "Completed simulation valve control");
    return;
  }

  Frame msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
 
  msg.id = RPDO_1 + can_id_;
  msg.dlc = 2;

  {
    if (request->data)
    {
      msg.data[0] = 0x1;
      msg.data[1] = 0x0;
      tpdo_pub_->publish(msg);
    }
    else
    {
      msg.data[0] = 0x0;
      msg.data[1] = 0x1;
      tpdo_pub_->publish(msg);

      rclcpp::TimerBase::SharedPtr timer = create_wall_timer(std::chrono::milliseconds(500), 
        [this, &timer]() {
          Frame release_msg(rosidl_runtime_cpp::MessageInitialization::ZERO);

          release_msg.id = RPDO_1 + can_id_;
          release_msg.dlc = 2;

          release_msg.data[0] = 0x0;
          release_msg.data[1] = 0x0;

          tpdo_pub_->publish(release_msg);
          
          timer->cancel();
      });
    }
  }

  RCLCPP_INFO(get_logger(), "Completed valve control");
  response->success = true;
}