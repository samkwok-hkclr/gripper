#include "vacuum_gripper/valve_control.hpp"

ValveControl::ValveControl(const rclcpp::NodeOptions& options)
: Node("valve_control", options)
{
  declare_parameter<bool>("sim", true);
  get_parameter<bool>("sim", simulation_);

  std::string co_dev_ns;

  declare_parameter<std::string>("co_dev_ns", "canopen_module");
  get_parameter<std::string>("co_dev_ns", co_dev_ns);

  VALID_RPDO_INDEXES = {0x6024, 0x6025};

  status_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;
  
  status_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(500), 
    std::bind(&ValveControl::status_cb, this), 
    status_cbg_);

  status_pub_ = create_publisher<Bool>("gripper_status", 10);
  tpdo_pub_ = create_publisher<COData>("/" + co_dev_ns + "/tpdo", 10);

  rpdo_sub_ = create_subscription<COData>(
    "/" + co_dev_ns + "/rpdo", 
    1000,
    std::bind(&ValveControl::rpdo_cb, this, _1),
    rpdo_options);

  ctrl_srv_ = create_service<SetBool>(
    "valve_control", 
    std::bind(&ValveControl::valve_control_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cbg_);

  RCLCPP_INFO(get_logger(), "Valve Control is up.");
}

bool ValveControl::is_valid_rpdo_index(uint16_t index) const
{
  return VALID_RPDO_INDEXES.find(index) != VALID_RPDO_INDEXES.end();
}

void ValveControl::rpdo_cb(const COData::SharedPtr msg)
{
  if (!is_valid_rpdo_index(msg->index)) 
    return;
    
  const std::lock_guard<std::mutex> lock(mutex_);

  switch (msg->index)
  {
    case 0x6024:
      gripper_status = static_cast<bool>(msg->data);
    break;
    case 0x6025:
      // not used 2nd valve
    break;
  }
}

void ValveControl::status_cb(void)
{
  Bool msg;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    msg.data = gripper_status;
  }
  
  status_pub_->publish(msg);
}

void ValveControl::valve_control_cb(
  const std::shared_ptr<SetBool::Request> request, 
  std::shared_ptr<SetBool::Response> response)
{
  COData msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
 
  msg.index = 0x6020;
  msg.subindex = 0x0;

  // Send non-zero value turn-on the valve
  msg.data = request->data ? 0xFF : 0x0;

  tpdo_pub_->publish(msg);

  response->success = true;
}