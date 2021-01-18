/**
 * @file fake_joint_driver.cpp
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include "fake_joint_driver/fake_joint_driver.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <fstream>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("fake_joint_driver");

std::vector<hardware_interface::StateInterface> FakeJointDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &act_dis[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &act_vel[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &act_eff[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeJointDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_dis[i]));
  }

  return command_interfaces;
}
hardware_interface::return_type FakeJointDriver::configure(const hardware_interface::HardwareInfo& info)
{
  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }
  // Default start position is zero
  cmd_dis.resize(info_.joints.size(), 0.0);
  act_dis.resize(info_.joints.size(), 0.0);
  act_vel.resize(info_.joints.size(), 0.0);
  act_eff.resize(info_.joints.size(), 0.0);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto& joint = info_.joints.at(i);
    if (auto it = joint.parameters.find("start_position"); it != joint.parameters.end())
    {
      cmd_dis.at(i) = std::stod(it->second);
      act_dis.at(i) = std::stod(it->second);
    }
  }
  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(FakeJointDriver, hardware_interface::SystemInterface)
