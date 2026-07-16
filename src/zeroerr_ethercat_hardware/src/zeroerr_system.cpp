#include "zeroerr_ethercat_hardware/zeroerr_system.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <time.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace zeroerr_ethercat_hardware
{

namespace
{

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

// ros2_control计划运行频率：1000 Hz
constexpr uint32_t PERIOD_NS = 1000000U;

// CiA402状态
constexpr uint16_t STATE_FAULT = 0x0008;
constexpr uint16_t STATE_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t STATE_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t STATE_SWITCHED_ON = 0x0023;
constexpr uint16_t STATE_OPERATION_ENABLED = 0x0027;

// CiA402控制字
constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CW_SHUTDOWN = 0x0006;
constexpr uint16_t CW_SWITCH_ON = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_FAULT_RESET = 0x0080;

/*
 * Slave 5真实PDO映射。
 *
 * RxPDO 0x1600：
 *   0x607A:00 Target Position
 *   0x60FE:00 Digital Outputs
 *   0x6040:00 Control Word
 *
 * TxPDO 0x1A00：
 *   0x6064:00 Position Actual Value
 *   0x60FD:00 Digital Inputs
 *   0x6041:00 Status Word
 */
ec_pdo_entry_info_t zeroerr_pdo_entries[] = {
  {0x607A, 0x00, 32},
  {0x60FE, 0x00, 32},
  {0x6040, 0x00, 16},

  {0x6064, 0x00, 32},
  {0x60FD, 0x00, 32},
  {0x6041, 0x00, 16},
};

ec_pdo_info_t zeroerr_pdos[] = {
  {0x1600, 3, zeroerr_pdo_entries + 0},
  {0x1A00, 3, zeroerr_pdo_entries + 3},
};

ec_sync_info_t zeroerr_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 1, zeroerr_pdos + 0, EC_WD_ENABLE},
  {3, EC_DIR_INPUT, 1, zeroerr_pdos + 1, EC_WD_DISABLE},
  {0xFF}
};

uint16_t decode_drive_state(const uint16_t status_word)
{
  return status_word & 0x006F;
}

const char * drive_state_to_string(const uint16_t state)
{
  switch (state) {
    case STATE_FAULT:
      return "FAULT";

    case STATE_SWITCH_ON_DISABLED:
      return "SWITCH ON DISABLED";

    case STATE_READY_TO_SWITCH_ON:
      return "READY TO SWITCH ON";

    case STATE_SWITCHED_ON:
      return "SWITCHED ON";

    case STATE_OPERATION_ENABLED:
      return "OPERATION ENABLED";

    default:
      return "OTHER/UNKNOWN";
  }
}

uint64_t monotonic_time_ns()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);

  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
         static_cast<uint64_t>(now.tv_nsec);
}

}  // namespace

// ============================================================
// ROS 2生命周期
// ============================================================

ZeroErrSystem::CallbackReturn ZeroErrSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "SystemInterface::on_init() failed");

    return CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "单轴验证版本要求URDF中恰好配置1个关节，当前为%zu个",
      info_.joints.size());

    return CallbackReturn::ERROR;
  }

  joint_name_ = info_.joints[0].name;

  const auto & joint = info_.joints[0];

  if (joint.command_interfaces.size() != 1) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "关节%s必须恰好配置1个command interface",
      joint_name_.c_str());

    return CallbackReturn::ERROR;
  }

  if (
    joint.command_interfaces[0].name !=
    hardware_interface::HW_IF_POSITION)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "关节%s的command interface必须是position",
      joint_name_.c_str());

    return CallbackReturn::ERROR;
  }

  bool has_position_state = false;
  bool has_velocity_state = false;

  for (const auto & state_interface : joint.state_interfaces) {
    if (
      state_interface.name ==
      hardware_interface::HW_IF_POSITION)
    {
      has_position_state = true;
    }

    if (
      state_interface.name ==
      hardware_interface::HW_IF_VELOCITY)
    {
      has_velocity_state = true;
    }
  }

  if (!has_position_state || !has_velocity_state) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "关节%s必须配置position和velocity状态接口",
      joint_name_.c_str());

    return CallbackReturn::ERROR;
  }

  if (!parse_hardware_parameters()) {
    return CallbackReturn::ERROR;
  }

  joint_position_ = 0.0;
  joint_velocity_ = 0.0;
  joint_position_command_ = 0.0;
  previous_joint_position_ = 0.0;

  position_received_ = false;
  command_initialized_ = false;
  hardware_active_ = false;
  communication_error_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "初始化完成：joint=%s, master=%u, slave=%u, "
    "vendor=0x%08X, product=0x%08X",
    joint_name_.c_str(),
    master_index_,
    slave_position_,
    vendor_id_,
    product_code_);

  return CallbackReturn::SUCCESS;
}

ZeroErrSystem::CallbackReturn ZeroErrSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "正在配置EtherLab和Slave %u...",
    slave_position_);

  release_ethercat();

  if (!configure_ethercat()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "EtherLab配置失败");

    release_ethercat();
    return CallbackReturn::ERROR;
  }

  position_received_ = false;
  command_initialized_ = false;
  hardware_active_ = false;
  communication_error_ = false;

  cycle_count_ = 0;
  valid_receive_cycles_ = 0;
  last_working_counter_ = 0;

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "EtherLab配置完成，等待硬件激活");

  return CallbackReturn::SUCCESS;
}

ZeroErrSystem::CallbackReturn ZeroErrSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (!master_ || !domain_ || !domain_data_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "无法激活：EtherLab对象未初始化");

    return CallbackReturn::ERROR;
  }

  /*
   * 不使用默认的0 rad命令。
   * read()成功读取实际位置后，会把命令初始化为当前位置。
   */
  position_received_ = false;
  command_initialized_ = false;
  drive_enabled_ = false;
  valid_receive_cycles_ = 0;
  hardware_active_ = true;

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "硬件已激活，将先读取实际位置并保持当前位置");

  return CallbackReturn::SUCCESS;
}

ZeroErrSystem::CallbackReturn ZeroErrSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "正在撤销Slave %u使能...",
    slave_position_);

  hardware_active_ = false;

  /*
   * 连续发送约200 ms Disable Voltage。
   * 退出过程中仍保持最后一个目标位置。
   */
  if (master_ && domain_ && domain_data_) {
    for (int i = 0; i < 200; ++i) {
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      if (command_initialized_) {
        EC_WRITE_S32(
          domain_data_ + offset_target_position_,
          target_position_counts_);
      }

      EC_WRITE_U16(
        domain_data_ + offset_control_word_,
        CW_DISABLE_VOLTAGE);

      ecrt_domain_queue(domain_);

      ecrt_master_application_time(
        master_,
        monotonic_time_ns());

      ecrt_master_sync_reference_clock(master_);
      ecrt_master_sync_slave_clocks(master_);
      ecrt_master_send(master_);

      usleep(1000);
    }
  }

  drive_enabled_ = false;

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "关节使能已撤销");

  return CallbackReturn::SUCCESS;
}

ZeroErrSystem::CallbackReturn ZeroErrSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  hardware_active_ = false;
  drive_enabled_ = false;

  release_ethercat();

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "EtherCAT Master已释放");

  return CallbackReturn::SUCCESS;
}

// ============================================================
// ROS 2接口导出
// ============================================================

std::vector<hardware_interface::StateInterface>
ZeroErrSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  interfaces.emplace_back(
    joint_name_,
    hardware_interface::HW_IF_POSITION,
    &joint_position_);

  interfaces.emplace_back(
    joint_name_,
    hardware_interface::HW_IF_VELOCITY,
    &joint_velocity_);

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ZeroErrSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  interfaces.emplace_back(
    joint_name_,
    hardware_interface::HW_IF_POSITION,
    &joint_position_command_);

  return interfaces;
}

// ============================================================
// EtherLab配置
// ============================================================

bool ZeroErrSystem::configure_ethercat()
{
  master_ = ecrt_request_master(master_index_);

  if (!master_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "申请EtherCAT Master %u失败",
      master_index_);

    return false;
  }

  /*
   * 在主站激活之前通过SDO设置CSP模式。
   */
  uint8_t mode = static_cast<uint8_t>(operation_mode_);
  uint32_t abort_code = 0;

  const int sdo_result = ecrt_master_sdo_download(
    master_,
    slave_position_,
    0x6060,
    0x00,
    &mode,
    sizeof(mode),
    &abort_code);

  if (sdo_result < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "写入CSP模式失败：result=%d, abort=0x%08X",
      sdo_result,
      abort_code);

    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "Slave %u已写入运行模式0x6060=%d",
    slave_position_,
    static_cast<int>(operation_mode_));

  domain_ = ecrt_master_create_domain(master_);

  if (!domain_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "创建EtherCAT Domain失败");

    return false;
  }

  slave_config_ = ecrt_master_slave_config(
    master_,
    slave_alias_,
    slave_position_,
    vendor_id_,
    product_code_);

  if (!slave_config_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "获取Slave %u配置失败",
      slave_position_);

    return false;
  }

  if (!configure_pdos()) {
    return false;
  }

  /*
   * 保留原程序的DC Sync0配置：
   * AssignActivate = 0x0300
   * Sync0周期 = 1 ms
   */
  ecrt_slave_config_dc(
    slave_config_,
    0x0300,
    PERIOD_NS,
    0,
    0,
    0);

  if (!register_pdo_entries()) {
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "正在激活EtherCAT Master...");

  if (ecrt_master_activate(master_) != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "激活EtherCAT Master失败");

    return false;
  }

  master_activated_ = true;

  domain_data_ = ecrt_domain_data(domain_);

  if (!domain_data_) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "获取Domain Process Data失败");

    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "EtherCAT Master激活成功");

  return true;
}

bool ZeroErrSystem::configure_pdos()
{
  if (!slave_config_) {
    return false;
  }

  const int result = ecrt_slave_config_pdos(
    slave_config_,
    EC_END,
    zeroerr_syncs);

  if (result != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "配置Slave %u PDO失败，result=%d",
      slave_position_,
      result);

    return false;
  }

  return true;
}

bool ZeroErrSystem::register_pdo_entries()
{
  if (!domain_) {
    return false;
  }

  ec_pdo_entry_reg_t domain_regs[] = {
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x607A,
      0x00,
      &offset_target_position_
    },
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x60FE,
      0x00,
      &offset_digital_outputs_
    },
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x6040,
      0x00,
      &offset_control_word_
    },
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x6064,
      0x00,
      &offset_actual_position_
    },
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x60FD,
      0x00,
      &offset_digital_inputs_
    },
    {
      slave_alias_,
      slave_position_,
      vendor_id_,
      product_code_,
      0x6041,
      0x00,
      &offset_status_word_
    },
    {}
  };

  const int result =
    ecrt_domain_reg_pdo_entry_list(domain_, domain_regs);

  if (result != 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "注册PDO Entry失败，result=%d",
      result);

    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("ZeroErrSystem"),
    "PDO偏移注册完成："
    "607A=%u, 60FE=%u, 6040=%u, "
    "6064=%u, 60FD=%u, 6041=%u",
    offset_target_position_,
    offset_digital_outputs_,
    offset_control_word_,
    offset_actual_position_,
    offset_digital_inputs_,
    offset_status_word_);

  return true;
}

void ZeroErrSystem::release_ethercat()
{
  if (master_) {
    ecrt_release_master(master_);
  }

  master_ = nullptr;
  domain_ = nullptr;
  slave_config_ = nullptr;
  domain_data_ = nullptr;

  master_activated_ = false;
  slave_online_ = false;
  slave_operational_ = false;
}

void ZeroErrSystem::update_ethercat_states()
{
  if (master_) {
    ecrt_master_state(master_, &master_state_);
  }

  if (domain_) {
    ecrt_domain_state(domain_, &domain_state_);
  }

  if (slave_config_) {
    ecrt_slave_config_state(
      slave_config_,
      &slave_config_state_);

    slave_online_ = slave_config_state_.online;
    slave_operational_ =
      slave_config_state_.operational;
  }

  communication_error_ =
    domain_state_.wc_state != EC_WC_COMPLETE;

  last_working_counter_ =
    domain_state_.working_counter;
}

bool ZeroErrSystem::ethercat_ready() const
{
  return
    master_activated_ &&
    domain_data_ != nullptr &&
    slave_online_ &&
    domain_state_.wc_state == EC_WC_COMPLETE &&
    status_word_ != 0;
}

// ============================================================
// 周期读取
// ============================================================

hardware_interface::return_type ZeroErrSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!master_ || !domain_ || !domain_data_) {
    return hardware_interface::return_type::ERROR;
  }

  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);

  update_ethercat_states();

  actual_position_counts_ = EC_READ_S32(
    domain_data_ + offset_actual_position_);

  digital_inputs_ = EC_READ_U32(
    domain_data_ + offset_digital_inputs_);

  status_word_ = EC_READ_U16(
    domain_data_ + offset_status_word_);

  drive_enabled_ =
    is_operation_enabled(status_word_);

  const bool valid_feedback =
    domain_state_.wc_state == EC_WC_COMPLETE &&
    status_word_ != 0;

  if (valid_feedback) {
    ++valid_receive_cycles_;

    const double new_position =
      counts_to_radians(actual_position_counts_);

    if (!position_received_) {
      joint_position_ = new_position;
      previous_joint_position_ = new_position;
      joint_velocity_ = 0.0;

      /*
       * 最重要的启动安全逻辑：
       * ROS命令先初始化成真实当前位置。
       */
      joint_position_command_ = new_position;

      target_position_counts_ =
        actual_position_counts_;

      previous_target_counts_ =
        actual_position_counts_;

      position_received_ = true;
      command_initialized_ = true;

      RCLCPP_INFO(
        rclcpp::get_logger("ZeroErrSystem"),
        "当前位置初始化完成：actual=%d counts, "
        "position=%.9f rad",
        actual_position_counts_,
        joint_position_);
    } else {
      joint_position_ = new_position;

      const double period_seconds =
        period.seconds();

      if (period_seconds > 0.0) {
        joint_velocity_ =
          (joint_position_ -
          previous_joint_position_) /
          period_seconds;
      } else {
        joint_velocity_ = 0.0;
      }

      previous_joint_position_ =
        joint_position_;
    }
  } else {
    valid_receive_cycles_ = 0;
  }

  if (
    cycle_count_ % 1000 == 0 ||
    communication_error_)
  {
    const uint16_t drive_state =
      decode_drive_state(status_word_);

    RCLCPP_INFO(
      rclcpp::get_logger("ZeroErrSystem"),
      "cycle=%llu, state=%s, status=0x%04X, "
      "actual=%d, target=%d, wc=%u, wc_state=%u, "
      "online=%d, operational=%d",
      static_cast<unsigned long long>(cycle_count_),
      drive_state_to_string(drive_state),
      status_word_,
      actual_position_counts_,
      target_position_counts_,
      domain_state_.working_counter,
      static_cast<unsigned int>(domain_state_.wc_state),
      slave_online_ ? 1 : 0,
      slave_operational_ ? 1 : 0);
  }

  return hardware_interface::return_type::OK;
}

// ============================================================
// 周期写入
// ============================================================

hardware_interface::return_type ZeroErrSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!master_ || !domain_ || !domain_data_) {
    return hardware_interface::return_type::ERROR;
  }

  if (command_initialized_) {
    /*
     * 未激活时只保持当前位置，不接收外部运动命令。
     */
    if (!hardware_active_) {
      target_position_counts_ =
        actual_position_counts_;

      previous_target_counts_ =
        target_position_counts_;
    } else if (
      drive_enabled_ &&
      valid_receive_cycles_ >= required_valid_cycles_)
    {
      if (!std::isfinite(joint_position_command_)) {
        RCLCPP_ERROR(
          rclcpp::get_logger("ZeroErrSystem"),
          "收到非有限位置命令，保持当前位置");

        joint_position_command_ =
          joint_position_;
      }

      const double command_jump =
        std::abs(
          joint_position_command_ -
          joint_position_);

      /*
       * 外部目标与当前位置跳变过大时拒绝执行。
       */
      if (command_jump > max_command_jump_) {
        if (cycle_count_ % 1000 == 0) {
          RCLCPP_ERROR(
            rclcpp::get_logger("ZeroErrSystem"),
            "拒绝位置命令：command=%.9f rad, "
            "actual=%.9f rad, jump=%.9f rad, "
            "limit=%.9f rad",
            joint_position_command_,
            joint_position_,
            command_jump,
            max_command_jump_);
        }
      } else {
        const int32_t requested_counts =
          radians_to_counts(
            joint_position_command_);

        target_position_counts_ =
          limit_target_step(
            requested_counts,
            period.seconds());
      }
    }

    /*
     * 在任何CiA402使能控制字发出前，
     * 始终先写入有效目标位置。
     */
    EC_WRITE_S32(
      domain_data_ + offset_target_position_,
      target_position_counts_);

    previous_target_counts_ =
      target_position_counts_;
  }

  if (hardware_active_) {
    control_word_ =
      make_control_word(status_word_);
  } else {
    control_word_ =
      CW_DISABLE_VOLTAGE;
  }

  EC_WRITE_U16(
    domain_data_ + offset_control_word_,
    control_word_);

  /*
   * 不操作0x60FE。
   * 保持与原测试程序相同，不随意改变数字输出。
   */

  ecrt_domain_queue(domain_);

  ecrt_master_application_time(
    master_,
    monotonic_time_ns());

  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
  ecrt_master_send(master_);

  ++cycle_count_;

  return hardware_interface::return_type::OK;
}

// ============================================================
// CiA402
// ============================================================

uint16_t ZeroErrSystem::make_control_word(
  const uint16_t status_word) const
{
  const uint16_t state =
    decode_drive_state(status_word);

  switch (state) {
    case STATE_FAULT:
      return CW_FAULT_RESET;

    case STATE_SWITCH_ON_DISABLED:
      return CW_SHUTDOWN;

    case STATE_READY_TO_SWITCH_ON:
      return CW_SWITCH_ON;

    case STATE_SWITCHED_ON:
      /*
       * 未取得有效当前位置时，
       * 不允许进入Operation Enabled。
       */
      return command_initialized_
             ? CW_ENABLE_OPERATION
             : CW_SWITCH_ON;

    case STATE_OPERATION_ENABLED:
      return command_initialized_
             ? CW_ENABLE_OPERATION
             : CW_SWITCH_ON;

    default:
      return CW_SHUTDOWN;
  }
}

bool ZeroErrSystem::is_operation_enabled(
  const uint16_t status_word) const
{
  return
    decode_drive_state(status_word) ==
    STATE_OPERATION_ENABLED;
}

bool ZeroErrSystem::is_fault(
  const uint16_t status_word) const
{
  return
    decode_drive_state(status_word) ==
    STATE_FAULT;
}

// ============================================================
// 单位转换和安全限制
// ============================================================

double ZeroErrSystem::counts_to_radians(
  const int32_t counts) const
{
  const double denominator =
    counts_per_revolution_ * gear_ratio_;

  return
    direction_ *
    static_cast<double>(
      counts - zero_offset_counts_) *
    TWO_PI /
    denominator;
}

int32_t ZeroErrSystem::radians_to_counts(
  const double radians) const
{
  const double counts =
    static_cast<double>(zero_offset_counts_) +
    direction_ *
    radians *
    counts_per_revolution_ *
    gear_ratio_ /
    TWO_PI;

  const double limited_counts =
    std::max(
      static_cast<double>(
        std::numeric_limits<int32_t>::min()),
      std::min(
        counts,
        static_cast<double>(
          std::numeric_limits<int32_t>::max())));

  return static_cast<int32_t>(
    std::llround(limited_counts));
}

int32_t ZeroErrSystem::limit_target_step(
  const int32_t requested_counts,
  const double period_seconds) const
{
  if (period_seconds <= 0.0) {
    return previous_target_counts_;
  }

  const double max_step_double =
    max_command_velocity_ *
    counts_per_revolution_ *
    gear_ratio_ *
    period_seconds /
    TWO_PI;

  const int32_t max_step_counts =
    std::max<int32_t>(
      1,
      static_cast<int32_t>(
        std::ceil(max_step_double)));

  const int64_t delta =
    static_cast<int64_t>(requested_counts) -
    static_cast<int64_t>(previous_target_counts_);

  if (delta > max_step_counts) {
    return
      previous_target_counts_ +
      max_step_counts;
  }

  if (delta < -max_step_counts) {
    return
      previous_target_counts_ -
      max_step_counts;
  }

  return requested_counts;
}

// ============================================================
// URDF参数读取
// ============================================================

bool ZeroErrSystem::parse_hardware_parameters()
{
  const auto get_parameter =
    [this](
      const std::string & name,
      const std::string & default_value)
    {
      const auto iterator =
        info_.hardware_parameters.find(name);

      if (
        iterator ==
        info_.hardware_parameters.end())
      {
        return default_value;
      }

      return iterator->second;
    };

  try {
    master_index_ = static_cast<unsigned int>(
      std::stoul(
        get_parameter("master_index", "0"),
        nullptr,
        0));

    slave_alias_ = static_cast<uint16_t>(
      std::stoul(
        get_parameter("slave_alias", "0"),
        nullptr,
        0));

    slave_position_ = static_cast<uint16_t>(
      std::stoul(
        get_parameter("slave_position", "5"),
        nullptr,
        0));

    vendor_id_ = static_cast<uint32_t>(
      std::stoul(
        get_parameter(
          "vendor_id",
          "0x5a65726f"),
        nullptr,
        0));

    product_code_ = static_cast<uint32_t>(
      std::stoul(
        get_parameter(
          "product_code",
          "0x00029252"),
        nullptr,
        0));

    operation_mode_ = static_cast<int8_t>(
      std::stoi(
        get_parameter(
          "operation_mode",
          "8")));

    counts_per_revolution_ =
      std::stod(
        get_parameter(
          "counts_per_revolution",
          "524288"));

    zero_offset_counts_ =
      static_cast<int32_t>(
        std::stol(
          get_parameter(
            "zero_offset_counts",
            "262144"),
          nullptr,
          0));

    direction_ =
      std::stod(
        get_parameter(
          "direction",
          "1.0"));

    gear_ratio_ =
      std::stod(
        get_parameter(
          "gear_ratio",
          "1.0"));

    max_command_velocity_ =
      std::stod(
        get_parameter(
          "max_command_velocity",
          "0.034906585"));

    max_command_jump_ =
      std::stod(
        get_parameter(
          "max_command_jump",
          "0.087266463"));

    required_valid_cycles_ =
      static_cast<uint32_t>(
        std::stoul(
          get_parameter(
            "required_valid_cycles",
            "100"),
          nullptr,
          0));
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "解析硬件参数失败：%s",
      exception.what());

    return false;
  }

  if (counts_per_revolution_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "counts_per_revolution必须大于0");

    return false;
  }

  if (gear_ratio_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "gear_ratio必须大于0");

    return false;
  }

  if (std::abs(direction_) < 1.0e-12) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "direction不能为0");

    return false;
  }

  direction_ = direction_ > 0.0 ? 1.0 : -1.0;

  if (max_command_velocity_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "max_command_velocity必须大于0");

    return false;
  }

  if (max_command_jump_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ZeroErrSystem"),
      "max_command_jump必须大于0");

    return false;
  }

  return true;
}

}  // namespace zeroerr_ethercat_hardware

PLUGINLIB_EXPORT_CLASS(
  zeroerr_ethercat_hardware::ZeroErrSystem,
  hardware_interface::SystemInterface)
