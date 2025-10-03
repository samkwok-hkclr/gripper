#include "vacuum_gripper/vacuum_gripper.hpp"

void VacuumGripper::tpdo_1_cb(const uint8_t* data)
{
  std::lock_guard<std::mutex> lock(mutex_);
  vac_gripper_state = data[3];
}

void VacuumGripper::tpdo_2_cb(const uint8_t* data)
{
  (void) data;
}

void VacuumGripper::tpdo_3_cb(const uint8_t* data)
{
  (void) data;
}

void VacuumGripper::tpdo_4_cb(const uint8_t* data)
{
  (void) data;
}