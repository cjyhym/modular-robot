#ifndef EROBOT_ISAAC_HARDWARE__ISAAC_SIM_SYSTEM_HPP_
#define EROBOT_ISAAC_HARDWARE__ISAAC_SIM_SYSTEM_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

namespace erobot_isaac_hardware
{

class IsaacSimSystem final : public hardware_interface::SystemInterface
{
public:
  IsaacSimSystem() = default;
  ~IsaacSimSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  void joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr message);

  void stop_ros_thread();

  std::vector<std::string> joint_names_;
  std::unordered_map<std::string, std::size_t> joint_index_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_position_commands_;

  std::vector<double> latest_positions_;
  std::vector<double> latest_velocities_;
  std::vector<double> latest_efforts_;

  std::string state_topic_{"/isaac_joint_states"};
  std::string command_topic_{"/isaac_joint_commands"};

  std::mutex state_mutex_;

  std::atomic<bool> received_complete_state_{false};
  std::atomic<bool> active_{false};

  bool commands_initialized_{false};

  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    command_publisher_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    state_subscription_;

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor>
    executor_;

  std::thread spin_thread_;
};

}  // namespace erobot_isaac_hardware

#endif  // EROBOT_ISAAC_HARDWARE__ISAAC_SIM_SYSTEM_HPP_
