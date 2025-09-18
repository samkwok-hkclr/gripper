#include "gripper/gripper.hpp"

Gripper::Gripper(const rclcpp::NodeOptions& options)
: Node("gripper", options)
{
  declare_parameter<bool>("sim", false);
  declare_parameter<uint8_t>("can_id", 0);
  declare_parameter<std::string>("can_namespace", "");
  declare_parameter<std::string>("can_interface", "");
  declare_parameter<float>("field_of_view", 0.0);
  declare_parameter<float>("min_range", 0.0);
  declare_parameter<float>("max_range", 0.0);
  declare_parameter<bool>("enable_ultrasonic", false);
  declare_parameter<uint8_t>("min_gripper_force", 0.0);
  declare_parameter<uint8_t>("max_gripper_force", 0.0);
  declare_parameter<float>("min_gripper_position", 0.0);
  declare_parameter<float>("max_gripper_position", 0.0);

  get_parameter<bool>("sim", simulation_);
  get_parameter<uint8_t>("can_id", can_id_);
  get_parameter<std::string>("can_namespace", can_ns_);
  get_parameter<std::string>("can_interface", can_interface_);
  get_parameter<float>("field_of_view", field_of_view_);
  get_parameter<float>("min_range", min_range_);
  get_parameter<float>("max_range", max_range_);
  get_parameter<bool>("enable_ultrasonic", enable_ultrasonic_);
  get_parameter<uint8_t>("min_gripper_force", min_gripper_force_);
  get_parameter<uint8_t>("max_gripper_force", max_gripper_force_);
  get_parameter<float>("min_gripper_position", min_gripper_position_);
  get_parameter<float>("max_gripper_position", max_gripper_position_);

  if (simulation_)
  {
    can_interface_ = "sim";
  }
  else
  {
    if (can_id_ == 0)
    {
      RCLCPP_INFO(get_logger(), "CAN ID does not set. Shutdown");
      rclcpp::shutdown();
      return;
    }
      
    if (can_interface_.empty())
    {
      RCLCPP_INFO(get_logger(), "CAN interface does not set. Shutdown");
      rclcpp::shutdown();
      return;
    }
  }

  valid_cob_frame_ = { 
    { static_cast<uint16_t>(TPDO_1 + can_id_), std::bind(&Gripper::tpdo_1_cb, this, _1) }, 
    { static_cast<uint16_t>(TPDO_2 + can_id_), std::bind(&Gripper::tpdo_2_cb, this, _1) },
    { static_cast<uint16_t>(TPDO_3 + can_id_), std::bind(&Gripper::tpdo_3_cb, this, _1) },
    { static_cast<uint16_t>(TPDO_4 + can_id_), std::bind(&Gripper::tpdo_4_cb, this, _1) },
    { static_cast<uint16_t>(NMT + can_id_), std::bind(&Gripper::nmt_cb, this, _1) }
  };

  srv_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;

  state_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&Gripper::vac_state_cb, this), 
    timer_cbg_);
  
  pressure_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&Gripper::pub_pressure_cb, this), 
    timer_cbg_);

  range_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&Gripper::pub_range_cb, this), 
    timer_cbg_);
    
  if(!enable_ultrasonic_)
    range_pub_timer_->cancel();

  gripper_status_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&Gripper::gripper_status_cb, this), 
    timer_cbg_);

  release_pressure_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&Gripper::release_pressure_cb, this), 
    timer_cbg_);

  release_pressure_timer_->cancel();

  auto_init_gripper_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000), 
    std::bind(&Gripper::auto_init_gripper_cb, this), 
    timer_cbg_);

  if (simulation_)
    auto_init_gripper_timer_->cancel();

  std::string tpdo_topic, rpdo_topic;
  if (can_ns_.empty())
  {
    tpdo_topic = "/" + can_interface_ + "/to_can_bus";
    rpdo_topic = "/" + can_interface_ + "/from_can_bus";
  }
  else
  {
    tpdo_topic = "/" + can_ns_ + "/" + can_interface_ + "/to_can_bus";
    rpdo_topic = "/" + can_ns_ + "/" + can_interface_ + "/from_can_bus";
  }
  RCLCPP_INFO(get_logger(), "can_recv_topic: %s", tpdo_topic.c_str());
  RCLCPP_INFO(get_logger(), "can_send_topic: %s", rpdo_topic.c_str());

  tpdo_pub_ = create_publisher<Frame>(tpdo_topic, 10);
  status_pub_ = create_publisher<Bool>("vacuum_gripper_status", 10);
  pressure_pub_ = create_publisher<FluidPressure>("vaccum_pressure", 10);
  range_pub_ = create_publisher<Range>("ultrasonic_range", 10); 
  temp_pub_ = create_publisher<Temperature>("ultrasonic_sensor_internal_temperature", 10); 
  gripper_pub_ = create_publisher<GripperStatus>("gripper_status", 10); 

  rpdo_sub_ = create_subscription<Frame>(
    rpdo_topic,
    1000,
    std::bind(&Gripper::rpdo_cb, this, _1),
    rpdo_options);

  ctrl_suction_srv_ = create_service<SetBool>(
    "pump_control", 
    std::bind(&Gripper::pump_ctrl_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cbg_);

  init_gripper_srv_ = create_service<Trigger>(
    "init_gripper", 
    std::bind(&Gripper::init_gripper_cb, this, _1, _2, false),
    rmw_qos_profile_services_default,
    srv_cbg_);

  re_init_gripper_srv_ = create_service<Trigger>(
    "re_init_gripper", 
    std::bind(&Gripper::init_gripper_cb, this, _1, _2, true),
    rmw_qos_profile_services_default,
    srv_cbg_);

  ctrl_gripper_srv_ = create_service<ControlGripper>(
    "gripper_control", 
    std::bind(&Gripper::ctrl_gripper_cb, this, _1, _2),
    rmw_qos_profile_services_default,
    srv_cbg_);

  if (enable_ultrasonic_)
    ctrl_ultrasonic_sensor(true);

  if (simulation_)
    RCLCPP_INFO(get_logger(), "Vacuum gripper [simulation] is up.");
  else
    RCLCPP_INFO(get_logger(), "Vacuum gripper [CAN ID: 0x%02X] is up.", static_cast<int>(can_id_));
}

Gripper::~Gripper()
{
  if (rclcpp::ok())
  {
    ctrl_ultrasonic_sensor(false);
  }
}

void Gripper::rpdo_cb(const Frame::SharedPtr msg)
{
  if (auto it = valid_cob_frame_.find(msg->id); it != valid_cob_frame_.end()) 
  {
    it->second(msg->data.data());
  }
}

