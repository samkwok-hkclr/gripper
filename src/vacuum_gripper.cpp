#include "vacuum_gripper/vacuum_gripper.hpp"

VacuumGripper::VacuumGripper(const rclcpp::NodeOptions& options)
: Node("vacuum_gripper", options)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<uint8_t>("can_id", 0);
  declare_parameter<uint8_t>("no_of_channel", 0);
  declare_parameter<float>("field_of_view", 0.0);
  declare_parameter<float>("min_range", 0.0);
  declare_parameter<float>("max_range", 0.0);
  declare_parameter<bool>("enable_ultrasonic", false);

  get_parameter<bool>("sim", simulation_);
  get_parameter<uint8_t>("can_id", can_id_);
  get_parameter<uint8_t>("no_of_channel", no_of_channel_);
  get_parameter<float>("field_of_view", field_of_view_);
  get_parameter<float>("min_range", min_range_);
  get_parameter<float>("max_range", max_range_);
  get_parameter<bool>("enable_ultrasonic", enable_ultrasonic_);

  if (can_id_ == 0 || can_id_ > 0x7F)
  {
    RCLCPP_INFO(get_logger(), "CAN ID does not set. Shutdown");
    rclcpp::shutdown();
    return;
  }

  if (no_of_channel_ == 0 || no_of_channel_ > 2)
  {
    RCLCPP_INFO(get_logger(), "Number of valve channel does not set. Shutdown");
    rclcpp::shutdown();
    return;
  }

  gripper_status.assign(no_of_channel_, false); 
  
  VALID_FRAME_ = { 
    { static_cast<uint16_t>(TPDO_1 + can_id_), 
      std::bind(&VacuumGripper::insert_pressure, this, _1) }, 
    { static_cast<uint16_t>(TPDO_2 + can_id_), 
      std::bind(&VacuumGripper::insert_range, this, _1) },
    { static_cast<uint16_t>(TPDO_3 + can_id_), 
      std::bind(&VacuumGripper::insert_valve_state, this, _1)} };

  status_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  pub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;

  status_pub_timer_ = create_wall_timer(
    std::chrono::seconds(1), 
    std::bind(&VacuumGripper::valve_status_cb, this), 
    status_cbg_);
  
  pressure_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&VacuumGripper::pub_pressure_cb, this), 
    pub_cbg_);

  range_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&VacuumGripper::pub_range_cb, this), 
    pub_cbg_);

  tpdo_pub_ = create_publisher<Frame>("/to_can_bus", 10);
  status_pub_ = create_publisher<Bool>("gripper_status", 10);
  pressure_pub_ = create_publisher<FluidPressure>("vaccum_pressure", 10);
  range_pub_ = create_publisher<Range>("ultrasonic_range", 10); 
  temp_pub_ = create_publisher<Temperature>("ultrasonic_sensor_internal_temperature", 10); 

  rpdo_sub_ = create_subscription<Frame>(
    "/from_can_bus", 
    1000,
    std::bind(&VacuumGripper::rpdo_cb, this, _1),
    rpdo_options);

  ctrl_srv_ = create_service<SetBool>(
    "valve_control", 
    std::bind(&VacuumGripper::valve_control_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cbg_);

  if (enable_ultrasonic_)
    ctrl_ultrasonic_sensor(true);

  RCLCPP_INFO(get_logger(), "Vacuum gripper [CAN ID: 0x%02X] is up.", static_cast<int>(can_id_));
}

VacuumGripper::~VacuumGripper()
{
  if (rclcpp::ok())
  {
    ctrl_ultrasonic_sensor(false);
  }
}

void VacuumGripper::rpdo_cb(const Frame::SharedPtr msg)
{
  if (auto it = VALID_FRAME_.find(msg->id); it != VALID_FRAME_.end()) 
  {
    it->second(msg->data.data());
  }
}

