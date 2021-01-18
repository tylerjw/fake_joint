/**
 * @file fake_joint_driver.h
 * @author Ryosuke Tajima
 * @copyright 2016, 2017, Tokyo Opensource Robotics Kyokai Association
 * @license http://www.apache.org/licenses/LICENSE-2.0 Apache-2.0
 *
 * FakeJointDriver class (only do loopback from command to status)
 * derived from the hardware_interface class
 */
#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"

class FakeJointDriver : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{
private:
  std::vector<double> cmd_dis;
  std::vector<double> act_dis;
  std::vector<double> act_vel;
  std::vector<double> act_eff;

public:

  hardware_interface::return_type configure(const hardware_interface::HardwareInfo& info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type start() override
  {
    status_ = hardware_interface::status::STARTED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type stop() override
  {
    status_ = hardware_interface::status::STOPPED;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type read() override
  {
    // only do loopback
    act_dis = cmd_dis;
    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type write() override
  {
    return hardware_interface::return_type::OK;
  }
};
