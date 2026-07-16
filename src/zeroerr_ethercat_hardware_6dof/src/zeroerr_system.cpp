#include "zeroerr_ethercat_hardware_6dof/zeroerr_system.hpp"

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <thread>

#include <time.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace
{

constexpr double TWO_PI = 6.283185307179586476925286766559;

// SDO 后台轮询 0x6077 的间隔(ms)
constexpr int SDO_POLL_INTERVAL_MS = 100;

constexpr uint16_t STATE_FAULT = 0x0008;
constexpr uint16_t STATE_SWITCH_ON_DISABLED = 0x0040;
constexpr uint16_t STATE_READY_TO_SWITCH_ON = 0x0021;
constexpr uint16_t STATE_SWITCHED_ON = 0x0023;
constexpr uint16_t STATE_OPERATION_ENABLED = 0x0027;

constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CW_SHUTDOWN = 0x0006;
constexpr uint16_t CW_SWITCH_ON = 0x0007;
constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
constexpr uint16_t CW_FAULT_RESET = 0x0080;

/*
 * 与已经验证成功的单关节程序保持一致：
 * RxPDO: 607A, 60FE, 6040
 * TxPDO: 6064, 60FD, 6041
 *
 * 注意：0x6077(转矩实际值)暂未映射进 PDO——
 * 设备 PRE-OP 拒绝自定义 PDO 重映射(追加 entry 和新增
 * 0x1A02 组均会导致 SIGABRT)。effort 读取代码已就绪
 * (read 中用 offset_actual_torque 判断)，待 PDO 映射
 * 问题解决后启用。
 */
ec_pdo_entry_info_t joint_pdo_entries[] = {
  {0x607A, 0x00, 32},
  {0x60FE, 0x00, 32},
  {0x6040, 0x00, 16},
  {0x6064, 0x00, 32},
  {0x60FD, 0x00, 32},
  {0x6041, 0x00, 16},
};

ec_pdo_info_t joint_pdos[] = {
  {0x1600, 3, joint_pdo_entries + 0},
  {0x1A00, 3, joint_pdo_entries + 3},
};

ec_sync_info_t joint_syncs[] = {
  {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
  {1, EC_DIR_INPUT, 0, nullptr, EC_WD_DISABLE},
  {2, EC_DIR_OUTPUT, 1, joint_pdos + 0, EC_WD_ENABLE},
  {3, EC_DIR_INPUT, 1, joint_pdos + 1, EC_WD_DISABLE},
  {0xFF, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE},
};

}  // namespace

namespace zeroerr_ethercat_hardware_6dof
{

ZeroErrSystem::~ZeroErrSystem()
{
  release_ethercat();
}

std::string ZeroErrSystem::required_string(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name)
{
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    throw std::runtime_error("缺少参数：" + name);
  }
  return iterator->second;
}

int64_t ZeroErrSystem::required_integer(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name)
{
  return std::stoll(required_string(parameters, name), nullptr, 0);
}

double ZeroErrSystem::required_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name)
{
  return std::stod(required_string(parameters, name));
}

int64_t ZeroErrSystem::optional_integer(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  int64_t default_value)
{
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    return default_value;
  }
  return std::stoll(iterator->second, nullptr, 0);
}

double ZeroErrSystem::optional_double(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  double default_value)
{
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    return default_value;
  }
  return std::stod(iterator->second);
}

bool ZeroErrSystem::optional_bool(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  bool default_value)
{
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    return default_value;
  }

  std::string value = iterator->second;
  std::transform(value.begin(), value.end(), value.begin(),
    [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });

  if (value == "true" || value == "1" || value == "yes" || value == "on") {
    return true;
  }
  if (value == "false" || value == "0" || value == "no" || value == "off") {
    return false;
  }

  throw std::runtime_error("布尔参数格式错误：" + name + "=" + iterator->second);
}

uint64_t ZeroErrSystem::monotonic_time_ns()
{
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return static_cast<uint64_t>(now.tv_sec) * 1000000000ULL +
    static_cast<uint64_t>(now.tv_nsec);
}

double ZeroErrSystem::clamp_double(
  double value,
  double lower,
  double upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

double ZeroErrSystem::abs_double(double value)
{
  return value >= 0.0 ? value : -value;
}

int64_t ZeroErrSystem::round_to_int64(double value)
{
  return value >= 0.0 ?
    static_cast<int64_t>(value + 0.5) :
    static_cast<int64_t>(value - 0.5);
}

hardware_interface::CallbackReturn ZeroErrSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 6) {
    RCLCPP_ERROR(
      logger_,
      "六轴插件要求恰好6个关节，当前为%zu个",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  try {
    master_index_ = static_cast<unsigned int>(
      optional_integer(info_.hardware_parameters, "master_index", 0));
    slave_alias_ = static_cast<uint16_t>(
      optional_integer(info_.hardware_parameters, "alias", 0));
    vendor_id_ = static_cast<uint32_t>(
      optional_integer(info_.hardware_parameters, "vendor_id", 0x5a65726f));
    product_code_ = static_cast<uint32_t>(
      optional_integer(info_.hardware_parameters, "product_code", 0x00029252));
    operation_mode_ = static_cast<uint8_t>(
      optional_integer(info_.hardware_parameters, "operation_mode", 8));
    cycle_ns_ = static_cast<uint32_t>(
      optional_integer(info_.hardware_parameters, "cycle_ns", 1000000));

    enable_drives_ = optional_bool(
      info_.hardware_parameters, "enable_drives", false);
    max_following_error_rad_ = optional_double(
      info_.hardware_parameters,
      "max_following_error_rad",
      0.08726646259971647);
    wc_error_limit_ = static_cast<unsigned int>(
      optional_integer(info_.hardware_parameters, "wc_error_limit", 3000));

    if (cycle_ns_ == 0) {
      throw std::runtime_error("cycle_ns必须大于0");
    }
    if (max_following_error_rad_ <= 0.0) {
      throw std::runtime_error("max_following_error_rad必须大于0");
    }

    joints_.clear();
    joints_.reserve(info_.joints.size());

    std::vector<uint16_t> used_slaves;

    for (std::size_t index = 0; index < info_.joints.size(); ++index) {
      const auto & joint_info = info_.joints[index];

      if (joint_info.command_interfaces.size() != 1 ||
          joint_info.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION)
      {
        throw std::runtime_error(
          joint_info.name + "必须且只能导出position命令接口");
      }

      bool has_position_state = false;
      bool has_velocity_state = false;

      for (const auto & interface : joint_info.state_interfaces) {
        if (interface.name == hardware_interface::HW_IF_POSITION) {
          has_position_state = true;
        }
        if (interface.name == hardware_interface::HW_IF_VELOCITY) {
          has_velocity_state = true;
        }
      }

      if (!has_position_state || !has_velocity_state) {
        throw std::runtime_error(
          joint_info.name + "必须包含position和velocity状态接口");
      }

      JointRuntime joint;
      joint.name = joint_info.name;

      const auto physical_iterator =
        joint_info.parameters.find("physical_joint");
      joint.physical_joint =
        physical_iterator == joint_info.parameters.end() ?
        ("J" + std::to_string(index + 1)) :
        physical_iterator->second;

      const int64_t slave_position = required_integer(
        joint_info.parameters, "slave_position");
      const int64_t encoder_resolution = required_integer(
        joint_info.parameters, "encoder_resolution");
      const int64_t zero_count = required_integer(
        joint_info.parameters, "zero_count");

      if (slave_position < 0 || slave_position > 65535) {
        throw std::runtime_error(joint.name + ".slave_position超出uint16范围");
      }
      if (encoder_resolution <= 0 ||
          encoder_resolution > std::numeric_limits<int32_t>::max())
      {
        throw std::runtime_error(joint.name + ".encoder_resolution无效");
      }
      if (zero_count < std::numeric_limits<int32_t>::min() ||
          zero_count > std::numeric_limits<int32_t>::max())
      {
        throw std::runtime_error(joint.name + ".zero_count超出int32范围");
      }

      joint.slave_position = static_cast<uint16_t>(slave_position);
      joint.encoder_resolution = static_cast<int32_t>(encoder_resolution);
      joint.zero_count = static_cast<int32_t>(zero_count);
      joint.direction = required_double(joint_info.parameters, "direction");
      joint.lower = required_double(joint_info.parameters, "lower");
      joint.upper = required_double(joint_info.parameters, "upper");
      joint.max_velocity = required_double(
        joint_info.parameters, "max_velocity");
      joint.max_effort = required_double(
        joint_info.parameters, "max_effort");
      joint.rated_torque = required_double(
        joint_info.parameters, "rated_torque");
      joint.digital_output_value = static_cast<uint32_t>(
        optional_integer(joint_info.parameters, "digital_output_value", 0));

      if (joint.direction != 1.0 && joint.direction != -1.0) {
        throw std::runtime_error(joint.name + ".direction只能为1或-1");
      }
      if (joint.lower >= joint.upper) {
        throw std::runtime_error(joint.name + "关节限位无效");
      }
      if (joint.max_velocity <= 0.0) {
        throw std::runtime_error(joint.name + ".max_velocity必须大于0");
      }

      if (std::find(
          used_slaves.begin(), used_slaves.end(),
          joint.slave_position) != used_slaves.end())
      {
        throw std::runtime_error(
          "Slave编号重复：" + std::to_string(joint.slave_position));
      }

      used_slaves.push_back(joint.slave_position);
      joints_.push_back(joint);

      RCLCPP_INFO(
        logger_,
        "%s <-> %s <-> slave%u, zero=%d, direction=%.0f",
        joint.name.c_str(),
        joint.physical_joint.c_str(),
        joint.slave_position,
        joint.zero_count,
        joint.direction);
    }

    RCLCPP_WARN(
      logger_,
      "enable_drives=%s；首次联调必须保持false",
      enable_drives_ ? "true" : "false");
  }
  catch (const std::exception & exception) {
    RCLCPP_ERROR(logger_, "六轴参数解析失败：%s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ZeroErrSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(joints_.size() * 3);

  for (auto & joint : joints_) {
    interfaces.emplace_back(
      joint.name,
      hardware_interface::HW_IF_POSITION,
      &joint.position);

    interfaces.emplace_back(
      joint.name,
      hardware_interface::HW_IF_VELOCITY,
      &joint.velocity);

    interfaces.emplace_back(
      joint.name,
      hardware_interface::HW_IF_EFFORT,
      &joint.effort);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ZeroErrSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(joints_.size());

  for (auto & joint : joints_) {
    interfaces.emplace_back(
      joint.name,
      hardware_interface::HW_IF_POSITION,
      &joint.command);
  }

  return interfaces;
}

bool ZeroErrSystem::configure_one_slave(JointRuntime & joint)
{
  joint.slave_config = ecrt_master_slave_config(
    master_,
    slave_alias_,
    joint.slave_position,
    vendor_id_,
    product_code_);

  if (joint.slave_config == nullptr) {
    RCLCPP_ERROR(
      logger_,
      "获取slave%u配置失败",
      joint.slave_position);
    return false;
  }

  if (ecrt_slave_config_pdos(
      joint.slave_config,
      EC_END,
      joint_syncs) != 0)
  {
    RCLCPP_ERROR(
      logger_,
      "配置slave%u PDO失败",
      joint.slave_position);
    return false;
  }

  ecrt_slave_config_dc(
    joint.slave_config,
    0x0300,
    cycle_ns_,
    0,
    0,
    0);

  return true;
}

bool ZeroErrSystem::configure_ethercat()
{
  release_ethercat();

  master_ = ecrt_request_master(master_index_);
  if (master_ == nullptr) {
    RCLCPP_ERROR(logger_, "申请EtherCAT Master %u失败", master_index_);
    return false;
  }

  for (const auto & joint : joints_) {
    uint8_t mode = operation_mode_;
    uint32_t abort_code = 0;

    const int result = ecrt_master_sdo_download(
      master_,
      joint.slave_position,
      0x6060,
      0x00,
      &mode,
      sizeof(mode),
      &abort_code);

    if (result < 0) {
      RCLCPP_ERROR(
        logger_,
        "设置slave%u CSP失败：result=%d abort=0x%08X",
        joint.slave_position,
        result,
        abort_code);
      release_ethercat();
      return false;
    }
  }

  domain_ = ecrt_master_create_domain(master_);
  if (domain_ == nullptr) {
    RCLCPP_ERROR(logger_, "创建EtherCAT Domain失败");
    release_ethercat();
    return false;
  }

  for (auto & joint : joints_) {
    if (!configure_one_slave(joint)) {
      release_ethercat();
      return false;
    }
  }

  if (!joints_.empty()) {
    const int reference_result = ecrt_master_select_reference_clock(
      master_, joints_.front().slave_config);

    if (reference_result != 0) {
      RCLCPP_WARN(logger_, "选择显式DC参考时钟失败，继续运行");
    }
  }

  domain_regs_.clear();
  domain_regs_.reserve(joints_.size() * 6 + 1);

  for (auto & joint : joints_) {
    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x607A, 0x00,
      &joint.offset_target_position, nullptr});

    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x60FE, 0x00,
      &joint.offset_digital_outputs, nullptr});

    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x6040, 0x00,
      &joint.offset_control_word, nullptr});

    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x6064, 0x00,
      &joint.offset_actual_position, nullptr});

    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x60FD, 0x00,
      &joint.offset_digital_inputs, nullptr});

    domain_regs_.push_back({
      slave_alias_, joint.slave_position,
      vendor_id_, product_code_,
      0x6041, 0x00,
      &joint.offset_status_word, nullptr});
  }

  domain_regs_.push_back({});

  if (ecrt_domain_reg_pdo_entry_list(
      domain_, domain_regs_.data()) != 0)
  {
    RCLCPP_ERROR(logger_, "注册六轴PDO Entry失败");
    release_ethercat();
    return false;
  }

  if (ecrt_master_activate(master_) != 0) {
    RCLCPP_ERROR(logger_, "激活EtherCAT Master失败");
    release_ethercat();
    return false;
  }

  master_activated_ = true;
  domain_data_ = ecrt_domain_data(domain_);

  if (domain_data_ == nullptr) {
    RCLCPP_ERROR(logger_, "获取Domain Process Data失败");
    release_ethercat();
    return false;
  }

  RCLCPP_INFO(logger_, "六轴EtherCAT Master配置完成");
  return true;
}

hardware_interface::CallbackReturn ZeroErrSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  if (!configure_ethercat()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    joint.position = std::numeric_limits<double>::quiet_NaN();
    joint.previous_position = std::numeric_limits<double>::quiet_NaN();
    joint.velocity = 0.0;
    joint.effort = 0.0;
    joint.command = std::numeric_limits<double>::quiet_NaN();
    joint.applied_command = std::numeric_limits<double>::quiet_NaN();
    joint.initialized = false;
    joint.last_status_word = 0xFFFF;
    joint.fault_reset_counter = 0;
  }

  active_ = false;
  safety_stop_ = false;
  incomplete_wc_cycles_ = 0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ZeroErrSystem::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  // 停止 SDO 力矩轮询线程
  sdo_poll_running_.store(false);
  if (sdo_poll_thread_.joinable()) {
    sdo_poll_thread_.join();
  }

  release_ethercat();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ZeroErrSystem::send_process_data()
{
  ecrt_domain_queue(domain_);
  ecrt_master_application_time(master_, monotonic_time_ns());
  ecrt_master_sync_reference_clock(master_);
  ecrt_master_sync_slave_clocks(master_);
  ecrt_master_send(master_);
}

double ZeroErrSystem::counts_to_radians(
  const JointRuntime & joint,
  int32_t count) const
{
  const int64_t delta =
    static_cast<int64_t>(count) -
    static_cast<int64_t>(joint.zero_count);

  return joint.direction *
    static_cast<double>(delta) *
    TWO_PI /
    static_cast<double>(joint.encoder_resolution);
}

int32_t ZeroErrSystem::radians_to_counts(
  const JointRuntime & joint,
  double radians) const
{
  const double raw =
    static_cast<double>(joint.zero_count) +
    joint.direction *
    radians *
    static_cast<double>(joint.encoder_resolution) /
    TWO_PI;

  const double limited = clamp_double(
    raw,
    static_cast<double>(std::numeric_limits<int32_t>::min()),
    static_cast<double>(std::numeric_limits<int32_t>::max()));

  return static_cast<int32_t>(round_to_int64(limited));
}


hardware_interface::CallbackReturn ZeroErrSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (
    master_ == nullptr ||
    domain_ == nullptr ||
    domain_data_ == nullptr)
  {
    RCLCPP_ERROR(
      logger_,
      "硬件尚未完成configure");

    return hardware_interface::CallbackReturn::ERROR;
  }

  /*
   * 关键修改：
   * on_activate不能阻塞几十秒，否则ros2controlcli的10秒超时
   * 会自动重试，产生多个重叠的生命周期请求。
   *
   * 这里只做500 ms安全预热，然后返回SUCCESS。
   * 后续由Controller Manager正常的read/write循环继续推动
   * 六个Slave进入OP。
   */
  active_ = true;
  safety_stop_ = false;
  incomplete_wc_cycles_ = 0;

  constexpr unsigned int WARMUP_CYCLES = 500;

  for (
    unsigned int cycle = 0;
    cycle < WARMUP_CYCLES;
    ++cycle)
  {
    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);
    ecrt_domain_state(
      domain_,
      &domain_state_);

    for (auto & joint : joints_) {
      joint.actual_count = EC_READ_S32(
        domain_data_
        + joint.offset_actual_position);

      joint.status_word = EC_READ_U16(
        domain_data_
        + joint.offset_status_word);

      joint.digital_inputs = EC_READ_U32(
        domain_data_
        + joint.offset_digital_inputs);

      /*
       * 预热阶段严格保持当前编码器位置，
       * 控制字始终为Disable Voltage。
       */
      joint.target_count =
        joint.actual_count;

      EC_WRITE_S32(
        domain_data_
        + joint.offset_target_position,
        joint.target_count);

      EC_WRITE_U32(
        domain_data_
        + joint.offset_digital_outputs,
        joint.digital_output_value);

      EC_WRITE_U16(
        domain_data_
        + joint.offset_control_word,
        CW_DISABLE_VOLTAGE);
    }

    send_process_data();
    usleep(1000);
  }

  ec_master_state_t master_state{};
  ecrt_master_state(
    master_,
    &master_state);

  RCLCPP_INFO(
    logger_,
    "预热完成：responding=%u，"
    "master_al=0x%02X，link=%u，"
    "wc_state=%u，working_counter=%u",
    master_state.slaves_responding,
    master_state.al_states,
    master_state.link_up,
    static_cast<unsigned int>(
      domain_state_.wc_state),
    domain_state_.working_counter);

  /*
   * 物理链路不存在或从站数量不对，才判定激活失败。
   * WC暂时不完整不再导致激活失败。
   */
  if (
    master_state.link_up == 0 ||
    master_state.slaves_responding !=
      joints_.size())
  {
    RCLCPP_ERROR(
      logger_,
      "EtherCAT链路异常："
      "link=%u responding=%u expected=%zu",
      master_state.link_up,
      master_state.slaves_responding,
      joints_.size());

    active_ = false;

    return hardware_interface::CallbackReturn::ERROR;
  }

  for (auto & joint : joints_) {
    ec_slave_config_state_t slave_state{};

    if (joint.slave_config != nullptr) {
      ecrt_slave_config_state(
        joint.slave_config,
        &slave_state);
    }

    /*
     * 只有收到有效状态字后才初始化ROS关节角。
     * 尚未进入过程数据通信的轴保持未初始化，
     * 后续在read()中自动初始化。
     */
    if (joint.status_word != 0) {
      joint.position =
        counts_to_radians(
          joint,
          joint.actual_count);

      joint.previous_position =
        joint.position;

      joint.velocity = 0.0;
      joint.command = joint.position;
      joint.applied_command =
        joint.position;

      joint.target_count =
        joint.actual_count;

      joint.initialized = true;
    } else {
      joint.position =
        std::numeric_limits<double>::
          quiet_NaN();

      joint.previous_position =
        std::numeric_limits<double>::
          quiet_NaN();

      joint.velocity = 0.0;

      joint.command =
        std::numeric_limits<double>::
          quiet_NaN();

      joint.applied_command =
        std::numeric_limits<double>::
          quiet_NaN();

      joint.target_count =
        joint.actual_count;

      joint.initialized = false;
    }

    RCLCPP_INFO(
      logger_,
      "%s slave%u："
      "online=%u operational=%u "
      "al_state=0x%02X "
      "status=0x%04X count=%d "
      "initialized=%s",
      joint.name.c_str(),
      joint.slave_position,
      slave_state.online,
      slave_state.operational,
      slave_state.al_state,
      joint.status_word,
      joint.actual_count,
      joint.initialized ? "true" : "false");
  }

  // 启动 SDO 力矩轮询线程(后台低频读 0x6077)
  sdo_poll_running_.store(true);
  sdo_poll_thread_ = std::thread(&ZeroErrSystem::sdo_poll_loop, this);

  if (!enable_drives_) {
    RCLCPP_WARN(
      logger_,
      "硬件已进入active，"
      "但enable_drives=false；"
      "只进行PDO通信和状态读取，"
      "不会使能电机");
  } else {
    RCLCPP_WARN(
      logger_,
      "enable_drives=true；"
      "驱动将由CiA402状态机使能");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

uint16_t ZeroErrSystem::decode_drive_state(uint16_t status_word) const
{
  return status_word & 0x006F;
}

uint16_t ZeroErrSystem::next_control_word(JointRuntime & joint) const
{
  const uint16_t state = decode_drive_state(joint.status_word);

  if (state == STATE_FAULT) {
    ++joint.fault_reset_counter;
    return ((joint.fault_reset_counter / 20U) % 2U == 0U) ?
      CW_DISABLE_VOLTAGE : CW_FAULT_RESET;
  }

  joint.fault_reset_counter = 0;

  if (!enable_drives_ || safety_stop_) {
    return CW_DISABLE_VOLTAGE;
  }

  switch (state) {
    case STATE_SWITCH_ON_DISABLED:
      return CW_SHUTDOWN;
    case STATE_READY_TO_SWITCH_ON:
      return CW_SWITCH_ON;
    case STATE_SWITCHED_ON:
      return joint.initialized ? CW_ENABLE_OPERATION : CW_SWITCH_ON;
    case STATE_OPERATION_ENABLED:
      return CW_ENABLE_OPERATION;
    default:
      return CW_SHUTDOWN;
  }
}


hardware_interface::return_type ZeroErrSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (
    !active_ ||
    master_ == nullptr ||
    domain_ == nullptr ||
    domain_data_ == nullptr)
  {
    return hardware_interface::return_type::OK;
  }

  ecrt_master_receive(master_);
  ecrt_domain_process(domain_);
  ecrt_domain_state(
    domain_,
    &domain_state_);

  const unsigned int wc_state =
    static_cast<unsigned int>(
      domain_state_.wc_state);

  if (wc_state != last_wc_state_) {
    RCLCPP_INFO(
      logger_,
      "Domain WC状态变化：%u -> %u，"
      "working_counter=%u",
      last_wc_state_,
      wc_state,
      domain_state_.working_counter);

    last_wc_state_ = wc_state;
  }

  if (
    domain_state_.wc_state !=
    EC_WC_COMPLETE)
  {
    ++incomplete_wc_cycles_;

    /*
     * 首周期及之后每秒打印一次，
     * 便于定位具体哪个Slave没有进入OP。
     */
    if (
      incomplete_wc_cycles_ == 1 ||
      incomplete_wc_cycles_ % 1000 == 0)
    {
      ec_master_state_t master_state{};
      ecrt_master_state(
        master_,
        &master_state);

      RCLCPP_WARN(
        logger_,
        "WC尚未完整：cycles=%u "
        "wc_state=%u wc=%u "
        "responding=%u master_al=0x%02X "
        "link=%u",
        incomplete_wc_cycles_,
        wc_state,
        domain_state_.working_counter,
        master_state.slaves_responding,
        master_state.al_states,
        master_state.link_up);

      for (auto & joint : joints_) {
        ec_slave_config_state_t state{};

        if (joint.slave_config != nullptr) {
          ecrt_slave_config_state(
            joint.slave_config,
            &state);
        }

        RCLCPP_WARN(
          logger_,
          "  %s slave%u："
          "online=%u operational=%u "
          "al_state=0x%02X "
          "status_word=0x%04X",
          joint.name.c_str(),
          joint.slave_position,
          state.online,
          state.operational,
          state.al_state,
          joint.status_word);
      }
    }

    /*
     * 关键修改：
     * 只读联调阶段enable_drives=false时，
     * WC不完整不能返回ERROR，
     * 否则Controller Manager会把组件退回unconfigured。
     */
    /*
     * 六个Slave从PREOP/SAFEOP进入OP需要一定时间。
     * 前10秒为启动宽限期，期间WC不完整只报警，
     * 不让硬件生命周期退回unconfigured。
     */
    constexpr unsigned int STARTUP_GRACE_CYCLES = 10000;

    if (
      enable_drives_ &&
      incomplete_wc_cycles_ >
        STARTUP_GRACE_CYCLES +
        wc_error_limit_)
    {
      RCLCPP_ERROR(
        logger_,
        "超过10秒启动宽限期后WC仍不完整，"
        "进入安全停止");

      safety_stop_ = true;

      return hardware_interface::
        return_type::ERROR;
    }
  } else {
    incomplete_wc_cycles_ = 0;
  }

  const double dt = period.seconds();

  for (auto & joint : joints_) {
    joint.actual_count = EC_READ_S32(
      domain_data_
      + joint.offset_actual_position);

    joint.digital_inputs = EC_READ_U32(
      domain_data_
      + joint.offset_digital_inputs);

    joint.status_word = EC_READ_U16(
      domain_data_
      + joint.offset_status_word);

    if (
      joint.status_word !=
      joint.last_status_word)
    {
      RCLCPP_INFO(
        logger_,
        "%s status 0x%04X -> 0x%04X",
        joint.name.c_str(),
        joint.last_status_word,
        joint.status_word);

      joint.last_status_word =
        joint.status_word;
    }

    /*
     * 状态字为0通常说明该Slave尚未建立有效PDO。
     * 此时不把0计数误认为真实关节位置。
     */
    if (joint.status_word == 0) {
      continue;
    }

    const double new_position =
      counts_to_radians(
        joint,
        joint.actual_count);

    if (
      !joint.initialized ||
      !std::isfinite(
        joint.previous_position))
    {
      joint.position =
        new_position;

      joint.previous_position =
        new_position;

      joint.velocity = 0.0;
      joint.effort = 0.0;
      joint.command = new_position;
      joint.applied_command =
        new_position;

      joint.target_count =
        joint.actual_count;

      joint.initialized = true;

      RCLCPP_INFO(
        logger_,
        "%s完成延迟初始化："
        "slave%u count=%d position=%.9f",
        joint.name.c_str(),
        joint.slave_position,
        joint.actual_count,
        joint.position);

      continue;
    }

    double raw_velocity = 0.0;

    if (
      dt > 1.0e-6 &&
      dt < 0.1)
    {
      raw_velocity =
        (
          new_position -
          joint.previous_position
        ) / dt;
    }

    constexpr double alpha = 0.15;

    joint.velocity =
      alpha * raw_velocity +
      (1.0 - alpha) *
      joint.velocity;

    // 0x6077 暂未映射进 PDO（设备拒绝重映射），
    // offset_actual_torque=0 时从 SDO 后台线程取值
    if (joint.offset_actual_torque != 0) {
      const int16_t torque_per_mille = EC_READ_S16(
        domain_data_ + joint.offset_actual_torque);

      const double raw_effort =
        (static_cast<double>(torque_per_mille) / 1000.0) *
        joint.rated_torque;

      joint.effort =
        alpha * raw_effort +
        (1.0 - alpha) * joint.effort;
    } else {
      // PDO 无 0x6077，使用 SDO 后台线程轮询值(已有低通)
      joint.effort =
        alpha * joint.sdo_torque_raw +
        (1.0 - alpha) * joint.effort;
    }

    joint.position =
      new_position;

    joint.previous_position =
      new_position;
  }

  return hardware_interface::
    return_type::OK;
}


hardware_interface::return_type ZeroErrSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (
    !active_ ||
    master_ == nullptr ||
    domain_ == nullptr ||
    domain_data_ == nullptr)
  {
    return hardware_interface::
      return_type::OK;
  }

  double dt = period.seconds();

  if (
    dt <= 1.0e-6 ||
    dt > 0.1)
  {
    dt =
      static_cast<double>(
        cycle_ns_) / 1.0e9;
  }

  for (auto & joint : joints_) {
    /*
     * 即使某轴尚未初始化，也必须持续写入安全PDO。
     * 不能continue，否则对应从站可能一直无法完成OP转换。
     */
    /*
     * PDO工作计数器完整之前，只维持实际位置并保持
     * Disable Voltage。总线稳定后才启动CiA402使能流程。
     */
    const bool process_data_ready =
      domain_state_.wc_state ==
      EC_WC_COMPLETE;

    if (
      !joint.initialized ||
      !enable_drives_ ||
      safety_stop_ ||
      !process_data_ready)
    {
      joint.target_count =
        joint.actual_count;

      if (joint.initialized) {
        joint.applied_command =
          joint.position;
      }

      joint.control_word =
        CW_DISABLE_VOLTAGE;
    } else {
      if (!std::isfinite(joint.command)) {
        joint.command =
          joint.position;
      }

      if (
        !std::isfinite(
          joint.applied_command))
      {
        joint.applied_command =
          joint.position;
      }

      const double desired =
        clamp_double(
          joint.command,
          joint.lower,
          joint.upper);

      const double maximum_step =
        joint.max_velocity * dt;

      joint.applied_command =
        clamp_double(
          desired,
          joint.applied_command -
            maximum_step,
          joint.applied_command +
            maximum_step);

      const uint16_t state =
        decode_drive_state(
          joint.status_word);

      if (
        state ==
          STATE_OPERATION_ENABLED &&
        abs_double(
          joint.applied_command -
          joint.position) >
          max_following_error_rad_)
      {
        RCLCPP_ERROR(
          logger_,
          "%s跟随误差超过限制："
          "command=%.6f actual=%.6f "
          "limit=%.6f",
          joint.name.c_str(),
          joint.applied_command,
          joint.position,
          max_following_error_rad_);

        safety_stop_ = true;

        joint.target_count =
          joint.actual_count;

        joint.control_word =
          CW_DISABLE_VOLTAGE;
      } else {
        joint.target_count =
          radians_to_counts(
            joint,
            joint.applied_command);

        joint.control_word =
          next_control_word(joint);
      }
    }

    EC_WRITE_S32(
      domain_data_
      + joint.offset_target_position,
      joint.target_count);

    EC_WRITE_U32(
      domain_data_
      + joint.offset_digital_outputs,
      joint.digital_output_value);

    EC_WRITE_U16(
      domain_data_
      + joint.offset_control_word,
      joint.control_word);
  }

  send_process_data();

  /*
   * 只读模式下即使WC暂时不完整，
   * 也保持硬件active以便继续诊断。
   */
  if (
    safety_stop_ &&
    enable_drives_)
  {
    return hardware_interface::
      return_type::ERROR;
  }

  return hardware_interface::
    return_type::OK;
}

void ZeroErrSystem::sdo_poll_loop()
{
  RCLCPP_INFO(logger_, "SDO 力矩轮询线程启动(0x6077,间隔%dms)", SDO_POLL_INTERVAL_MS);

  rclcpp::Clock throttle_clock{RCL_ROS_TIME};

  while (sdo_poll_running_.load()) {
    for (auto & joint : joints_) {
      if (!sdo_poll_running_.load()) break;

      if (master_ == nullptr) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }

      int16_t torque_per_mille = 0;
      size_t result_size = 0;
      uint32_t abort_code = 0;

      const int ret = ecrt_master_sdo_upload(
        master_,
        joint.slave_position,
        0x6077, 0x00,
        reinterpret_cast<uint8_t *>(&torque_per_mille),
        sizeof(torque_per_mille),
        &result_size,
        &abort_code);

      if (ret == 0 && abort_code == 0 &&
          result_size == sizeof(int16_t)) {
        joint.sdo_torque_raw =
          (static_cast<double>(torque_per_mille) / 1000.0) *
          joint.rated_torque;
      } else if (abort_code != 0) {
        RCLCPP_WARN_THROTTLE(
          logger_,
          throttle_clock,
          10000,
          "%s 0x6077 SDO 上传失败 abort=0x%08X",
          joint.name.c_str(), abort_code);
      }
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(SDO_POLL_INTERVAL_MS));
  }

  RCLCPP_INFO(logger_, "SDO 力矩轮询线程退出");
}

hardware_interface::CallbackReturn ZeroErrSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // 停止 SDO 力矩轮询线程
  sdo_poll_running_.store(false);
  if (sdo_poll_thread_.joinable()) {
    sdo_poll_thread_.join();
  }

  if (master_ != nullptr &&
      domain_ != nullptr &&
      domain_data_ != nullptr)
  {
    for (int cycle = 0; cycle < 200; ++cycle) {
      ecrt_master_receive(master_);
      ecrt_domain_process(domain_);

      for (auto & joint : joints_) {
        const int32_t actual = EC_READ_S32(
          domain_data_ + joint.offset_actual_position);

        EC_WRITE_S32(
          domain_data_ + joint.offset_target_position,
          actual);

        EC_WRITE_U16(
          domain_data_ + joint.offset_control_word,
          CW_DISABLE_VOLTAGE);
      }

      send_process_data();
      usleep(1000);
    }
  }

  active_ = false;
  RCLCPP_INFO(logger_, "六轴硬件已安全撤销使能");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void ZeroErrSystem::release_ethercat()
{
  active_ = false;
  domain_data_ = nullptr;
  domain_ = nullptr;

  for (auto & joint : joints_) {
    joint.slave_config = nullptr;
  }

  if (master_ != nullptr) {
    if (master_activated_) {
      ecrt_master_deactivate(master_);
    }
    ecrt_release_master(master_);
    master_ = nullptr;
  }

  master_activated_ = false;
}

}  // namespace zeroerr_ethercat_hardware_6dof

PLUGINLIB_EXPORT_CLASS(
  zeroerr_ethercat_hardware_6dof::ZeroErrSystem,
  hardware_interface::SystemInterface)
