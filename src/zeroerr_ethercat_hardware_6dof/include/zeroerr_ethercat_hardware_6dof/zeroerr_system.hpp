#pragma once

#include <ecrt.h>

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <unordered_map>
#include <vector>
#include <atomic>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace zeroerr_ethercat_hardware_6dof
{

class ZeroErrSystem : public hardware_interface::SystemInterface
{
public:
  ~ZeroErrSystem() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  struct JointRuntime
  {
    std::string name;
    std::string physical_joint;

    uint16_t slave_position{0};
    int32_t encoder_resolution{524288};
    int32_t zero_count{0};
    double direction{1.0};

    double lower{-3.14};
    double upper{3.14};
    double max_velocity{0.10};
    double max_effort{100.0};
    double rated_torque{1.0};   // 电机侧额定转矩(N·m),用于 0x6077 转矩实际值换算

    uint32_t digital_output_value{0};

    ec_slave_config_t * slave_config{nullptr};

    unsigned int offset_target_position{0};
    unsigned int offset_digital_outputs{0};
    unsigned int offset_control_word{0};
    unsigned int offset_actual_position{0};
    unsigned int offset_digital_inputs{0};
    unsigned int offset_status_word{0};
    unsigned int offset_actual_torque{0};

    int32_t actual_count{0};
    int32_t target_count{0};
    uint32_t digital_inputs{0};
    uint16_t status_word{0};
    uint16_t last_status_word{0xFFFF};
    uint16_t control_word{0};
    uint32_t fault_reset_counter{0};

    double position{
      std::numeric_limits<double>::quiet_NaN()};
    double previous_position{
      std::numeric_limits<double>::quiet_NaN()};
    double velocity{0.0};
    double effort{0.0};   // 电机侧实际力矩(N·m),effort state_interface 源变量
    double sdo_torque_raw{0.0};  // SDO 后台线程写入(N·m)，offset_actual_torque=0 时 read() 从此取值
    double command{
      std::numeric_limits<double>::quiet_NaN()};
    double applied_command{
      std::numeric_limits<double>::quiet_NaN()};

    bool initialized{false};
  };

  static std::string required_string(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name);

  static int64_t required_integer(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name);

  static double required_double(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name);

  static int64_t optional_integer(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name,
    int64_t default_value);

  static double optional_double(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name,
    double default_value);

  static bool optional_bool(
    const std::unordered_map<std::string, std::string> & parameters,
    const std::string & name,
    bool default_value);

  static uint64_t monotonic_time_ns();
  static double clamp_double(double value, double lower, double upper);
  static double abs_double(double value);
  static int64_t round_to_int64(double value);

  bool configure_ethercat();
  bool configure_one_slave(JointRuntime & joint);
  void send_process_data();
  void release_ethercat();

  uint16_t decode_drive_state(uint16_t status_word) const;
  uint16_t next_control_word(JointRuntime & joint) const;

  double counts_to_radians(
    const JointRuntime & joint,
    int32_t count) const;

  int32_t radians_to_counts(
    const JointRuntime & joint,
    double radians) const;

  rclcpp::Logger logger_{
    rclcpp::get_logger("ZeroErrSystem")};

  std::vector<JointRuntime> joints_;

  ec_master_t * master_{nullptr};
  ec_domain_t * domain_{nullptr};
  uint8_t * domain_data_{nullptr};
  std::vector<ec_pdo_entry_reg_t> domain_regs_;

  ec_domain_state_t domain_state_{};

  unsigned int master_index_{0};
  uint16_t slave_alias_{0};
  uint32_t vendor_id_{0x5a65726f};
  uint32_t product_code_{0x00029252};
  uint8_t operation_mode_{8};
  uint32_t cycle_ns_{1000000};

  bool enable_drives_{false};
  double max_following_error_rad_{0.08726646259971647};
  unsigned int wc_error_limit_{3000};

  bool master_activated_{false};
  bool active_{false};
  bool safety_stop_{false};
  unsigned int incomplete_wc_cycles_{0};
  unsigned int last_wc_state_{999};

  // SDO 后台轮询 0x6077(转矩实际值)
  std::thread sdo_poll_thread_;
  std::atomic<bool> sdo_poll_running_{false};
  void sdo_poll_loop();

  // 周期抖动测量
  std::chrono::steady_clock::time_point last_read_time_{};
  int64_t cycle_count_{0};
  double jitter_min_us_{std::numeric_limits<double>::max()};
  double jitter_max_us_{0.0};
  double jitter_sum_us_{0.0};

  // PDO round-trip 测量
  std::chrono::steady_clock::time_point pdort_write_time_{};
  int64_t pdort_count_{0};
  double pdort_min_us_{std::numeric_limits<double>::max()};
  double pdort_max_us_{0.0};
  double pdort_sum_us_{0.0};
};

}  // namespace zeroerr_ethercat_hardware_6dof
