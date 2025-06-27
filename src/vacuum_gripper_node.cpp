#include "vacuum_gripper/vacuum_sensor.hpp"
#include "vacuum_gripper/valve_control.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  
  auto valve_ctrl_node = std::make_shared<ValveControl>(options);
  auto vac_sensor_node = std::make_shared<VacuumSensor>(options);

  exec->add_node(valve_ctrl_node->get_node_base_interface());
  exec->add_node(vac_sensor_node->get_node_base_interface());
  exec->spin();
    
  rclcpp::shutdown();
}
