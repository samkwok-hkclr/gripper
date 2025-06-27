#include "vacuum_gripper/vacuum_sensor.hpp"

VacuumSensor::VacuumSensor(const rclcpp::NodeOptions& options)
: Node("vacuum_sensor", options)
{
  declare_parameter<bool>("sim", true);
  get_parameter<bool>("sim", simulation_);

  std::string co_dev_ns;

  declare_parameter<std::string>("co_dev_ns", "canopen_module");
  get_parameter<std::string>("co_dev_ns", co_dev_ns);

  VALID_RPDO_INDEXES = {0x6000, 0x6001};

  pub_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rpdo_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  rclcpp::SubscriptionOptions rpdo_options;
  rpdo_options.callback_group = rpdo_cbg_;
  
  pressure_pub_timer_ = create_wall_timer(
    std::chrono::milliseconds(100), 
    std::bind(&VacuumSensor::pub_pressure_cb, this), 
    pub_cbg_);

  pressure_pub_ = create_publisher<FluidPressure>("vaccum_pressure", 10);

  rpdo_sub_ = create_subscription<COData>(
    "/" + co_dev_ns + "/rpdo", 
    1000,
    std::bind(&VacuumSensor::rpdo_cb, this, _1),
    rpdo_options);

  RCLCPP_INFO(get_logger(), "Vacuum Sensor is up.");
}

bool VacuumSensor::is_valid_rpdo_index(uint16_t index) const
{
  return VALID_RPDO_INDEXES.find(index) != VALID_RPDO_INDEXES.end();
}

void VacuumSensor::rpdo_cb(const COData::SharedPtr msg)
{
  if (!is_valid_rpdo_index(msg->index)) 
    return;

  const std::lock_guard<std::mutex> lock(mutex_);

  switch (msg->index)
  {
    case 0x6000:
      tracker_.insert_pressure(static_cast<float>(msg->data));
    break;
    case 0x6001:
      // not used
    break;
  }
}

void VacuumSensor::pub_pressure_cb(void)
{
  FluidPressure msg(rosidl_runtime_cpp::MessageInitialization::ZERO);
  msg.header.stamp = get_clock()->now();

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
