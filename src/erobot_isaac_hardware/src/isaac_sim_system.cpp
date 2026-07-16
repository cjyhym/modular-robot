#include "erobot_isaac_hardware/isaac_sim_system.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace erobot_isaac_hardware
{

IsaacSimSystem::~IsaacSimSystem()
{
  stop_ros_thread();
}


hardware_interface::CallbackReturn IsaacSimSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info.joints.empty())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("IsaacSimSystem"),
      "URDF 中没有 ros2_control joint。");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joint_count = info.joints.size();

  joint_names_.reserve(joint_count);

  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_efforts_.assign(joint_count, 0.0);
  hw_position_commands_.assign(joint_count, 0.0);

  latest_positions_.assign(joint_count, 0.0);
  latest_velocities_.assign(joint_count, 0.0);
  latest_efforts_.assign(joint_count, 0.0);

  for (std::size_t i = 0; i < joint_count; ++i)
  {
    const auto & joint = info.joints[i];

    joint_names_.push_back(joint.name);
    joint_index_[joint.name] = i;

    // 当前仿真接口只接受位置命令。
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name !=
      hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("IsaacSimSystem"),
        "关节 %s 必须且只能声明 position command interface。",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position_state = false;

    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;
      }
      else if (
        state_interface.name != hardware_interface::HW_IF_VELOCITY &&
        state_interface.name != hardware_interface::HW_IF_EFFORT)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("IsaacSimSystem"),
          "关节 %s 包含不支持的状态接口：%s",
          joint.name.c_str(),
          state_interface.name.c_str());

        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!has_position_state)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("IsaacSimSystem"),
        "关节 %s 缺少 position state interface。",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  const auto state_topic_it =
    info.hardware_parameters.find("state_topic");

  if (state_topic_it != info.hardware_parameters.end())
  {
    state_topic_ = state_topic_it->second;
  }

  const auto command_topic_it =
    info.hardware_parameters.find("command_topic");

  if (command_topic_it != info.hardware_parameters.end())
  {
    command_topic_ = command_topic_it->second;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("IsaacSimSystem"),
    "已读取 %zu 个关节，状态话题=%s，命令话题=%s",
    joint_count,
    state_topic_.c_str(),
    command_topic_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IsaacSimSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  try
  {
    node_ = std::make_shared<rclcpp::Node>(
      "erobot_isaac_hardware_node");

    // 状态订阅使用 best-effort，兼容仿真传感器类数据。
    state_subscription_ =
      node_->create_subscription<sensor_msgs::msg::JointState>(
        state_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(
          &IsaacSimSystem::joint_state_callback,
          this,
          std::placeholders::_1));

    // 控制命令使用可靠通信。
    command_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>(
        command_topic_,
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

    executor_ =
      std::make_shared<
      rclcpp::executors::SingleThreadedExecutor>();

    executor_->add_node(
      node_->get_node_base_interface());

    spin_thread_ = std::thread(
      [this]()
      {
        executor_->spin();
      });

    RCLCPP_INFO(
      node_->get_logger(),
      "Isaac Sim 话题硬件接口配置完成。");
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("IsaacSimSystem"),
      "配置 Isaac Sim 话题接口失败：%s",
      exception.what());

    stop_ros_thread();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IsaacSimSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);
  stop_ros_thread();

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IsaacSimSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  // 在收到第一帧完整状态之前，不发布任何位置命令。
  commands_initialized_ = false;
  active_.store(true);

  RCLCPP_INFO(
    rclcpp::get_logger("IsaacSimSystem"),
    "Isaac Sim 硬件接口已激活，等待第一帧关节状态。");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn IsaacSimSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  active_.store(false);

  RCLCPP_INFO(
    rclcpp::get_logger("IsaacSimSystem"),
    "Isaac Sim 硬件接口已停用。");

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
IsaacSimSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface>
    state_interfaces;

  for (std::size_t i = 0; i < info_.joints.size(); ++i)
  {
    const auto & joint = info_.joints[i];

    for (const auto & interface : joint.state_interfaces)
    {
      if (interface.name == hardware_interface::HW_IF_POSITION)
      {
        state_interfaces.emplace_back(
          joint.name,
          hardware_interface::HW_IF_POSITION,
          &hw_positions_[i]);
      }
      else if (
        interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        state_interfaces.emplace_back(
          joint.name,
          hardware_interface::HW_IF_VELOCITY,
          &hw_velocities_[i]);
      }
      else if (
        interface.name == hardware_interface::HW_IF_EFFORT)
      {
        state_interfaces.emplace_back(
          joint.name,
          hardware_interface::HW_IF_EFFORT,
          &hw_efforts_[i]);
      }
    }
  }

  return state_interfaces;
}


std::vector<hardware_interface::CommandInterface>
IsaacSimSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface>
    command_interfaces;

  for (std::size_t i = 0; i < joint_names_.size(); ++i)
  {
    command_interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_position_commands_[i]);
  }

  return command_interfaces;
}


void IsaacSimSystem::joint_state_callback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  std::vector<bool> position_received(
    joint_names_.size(), false);

  std::lock_guard<std::mutex> lock(state_mutex_);

  for (std::size_t msg_index = 0;
    msg_index < message->name.size();
    ++msg_index)
  {
    const auto found =
      joint_index_.find(message->name[msg_index]);

    if (found == joint_index_.end())
    {
      continue;
    }

    const std::size_t joint_index = found->second;

    if (msg_index < message->position.size())
    {
      latest_positions_[joint_index] =
        message->position[msg_index];

      position_received[joint_index] = true;
    }

    if (msg_index < message->velocity.size())
    {
      latest_velocities_[joint_index] =
        message->velocity[msg_index];
    }

    if (msg_index < message->effort.size())
    {
      latest_efforts_[joint_index] =
        message->effort[msg_index];
    }
  }

  const bool complete =
    std::all_of(
      position_received.begin(),
      position_received.end(),
      [](bool received)
      {
        return received;
      });

  if (complete)
  {
    received_complete_state_.store(true);
  }
}


hardware_interface::return_type IsaacSimSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!received_complete_state_.load())
  {
    return hardware_interface::return_type::OK;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);

    hw_positions_ = latest_positions_;
    hw_velocities_ = latest_velocities_;
    hw_efforts_ = latest_efforts_;
  }

  // 第一次收到 Isaac Sim 的真实姿态后，让命令与当前姿态同步，
  // 防止控制器刚启动时突然发送全零目标。
  if (!commands_initialized_)
  {
    hw_position_commands_ = hw_positions_;
    commands_initialized_ = true;

    RCLCPP_INFO(
      rclcpp::get_logger("IsaacSimSystem"),
      "已接收到完整关节状态，命令已与当前姿态同步。");
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type IsaacSimSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (
    !active_.load() ||
    !received_complete_state_.load() ||
    !commands_initialized_ ||
    !command_publisher_)
  {
    return hardware_interface::return_type::OK;
  }

  for (const double command : hw_position_commands_)
  {
    if (!std::isfinite(command))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("IsaacSimSystem"),
        "检测到非有限位置命令，拒绝发送。");

      return hardware_interface::return_type::ERROR;
    }
  }

  sensor_msgs::msg::JointState command_message;

  command_message.name = joint_names_;
  command_message.position = hw_position_commands_;

  command_publisher_->publish(command_message);

  return hardware_interface::return_type::OK;
}


void IsaacSimSystem::stop_ros_thread()
{
  if (executor_)
  {
    executor_->cancel();
  }

  if (spin_thread_.joinable())
  {
    spin_thread_.join();
  }

  if (executor_ && node_)
  {
    try
    {
      executor_->remove_node(
        node_->get_node_base_interface());
    }
    catch (...)
    {
      // 关闭阶段忽略重复移除异常。
    }
  }

  state_subscription_.reset();
  command_publisher_.reset();
  executor_.reset();
  node_.reset();
}

}  // namespace erobot_isaac_hardware


PLUGINLIB_EXPORT_CLASS(
  erobot_isaac_hardware::IsaacSimSystem,
  hardware_interface::SystemInterface)
