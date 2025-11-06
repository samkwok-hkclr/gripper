#include "gripper/gripper.hpp"

void Gripper::tpdo_1_cb(const uint8_t* data)
{
  insert_pressure(static_cast<int16_t>(data[1] << 8 | data[0]));
  suction_gripper_state.store(data[3]);
}

void Gripper::tpdo_2_cb(const uint8_t* data)
{
  (void) data;
}

void Gripper::tpdo_3_cb(const uint8_t* data)
{
  gripper_is_initialized.store(data[0]);
  gripper_state.store(data[1]);
  gripper_position.store(static_cast<int16_t>(data[3] << 8 | data[2]));
  gripper_force.store(static_cast<int16_t>(data[5] << 8 | data[4]));
}

void Gripper::tpdo_4_cb(const uint8_t* data)
{
  (void) data;
}

void Gripper::nmt_cb(const uint8_t* data)
{
  (void) data;
}