#ifndef ZEROERR_ETHERCAT_HARDWARE__ZEROERR_SYSTEM_HPP_
#define ZEROERR_ETHERCAT_HARDWARE__ZEROERR_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <ecrt.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace zeroerr_ethercat_hardware
{

/**
 * @brief 基于 EtherLab 的零差云控关节 ros2_control 硬件接口。
 * 当前版本用于单轴验证：
 *
 * ROS 2 position command
 *          ↓
 * ZeroErrSystem::write()
 *          ↓
 * EtherLab PDO 0x607A
 *          ↓
 * 零差云控关节 CSP 运动
 * 关节反馈：
 *
 * EtherLab PDO 0x6064
 *          ↓
 * ZeroErrSystem::read()
 *          ↓
 * ros2_control position state
 */
class ZeroErrSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ZeroErrSystem)

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::
    LifecycleNodeInterface::CallbackReturn;

  ZeroErrSystem() = default;
  ~ZeroErrSystem() override = default;

  /**
   * @brief 读取 URDF 中 ros2_control 的硬件参数。
   *
   * 此阶段只解析参数、检查关节和接口配置，
   * 暂时不启动 EtherCAT 周期通信。
   */
  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief 创建 EtherLab Master、Domain，配置 PDO 并激活主站。
   */
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 激活硬件。
   *
   * 激活时不会直接使用默认的0 rad命令，
   * 而是等待read()取得当前位置后执行当前位置保持。
   */
  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 停止接收外部运动命令并保持安全状态。
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief 释放 EtherLab Master 及相关资源。
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  /**
   * @brief 导出关节状态接口。
   *
   * 当前导出：
   *   joint/position
   *   joint/velocity
   */
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  /**
   * @brief 导出关节命令接口。
   *
   * 当前导出：
   *   joint/position
   */
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * @brief 从 EtherCAT PDO 读取实际位置、状态字等。
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  /**
   * @brief 将 ros2_control 目标位置写入 EtherCAT PDO。
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // ============================================================
  // EtherLab 初始化与资源管理
  // ============================================================

  /**
   * @brief 初始化 EtherLab Master、Domain、SlaveConfig 和 PDO。
   */
  bool configure_ethercat();

  /**
   * @brief 配置从站 PDO 映射。
   *
   * 具体 ec_pdo_entry_info_t、ec_pdo_info_t 和 ec_sync_info_t
   * 数组将在 zeroerr_system.cpp 中定义。
   */
  bool configure_pdos();

  /**
   * @brief 注册需要访问的 PDO entry。
   */
  bool register_pdo_entries();

  /**
   * @brief 释放 EtherLab Master。
   */
  void release_ethercat();

  /**
   * @brief 检查 Master、Domain 和 Slave 状态。
   */
  void update_ethercat_states();

  /**
   * @brief 判断 EtherCAT 通信是否已经达到可控制状态。
   */
  bool ethercat_ready() const;

  // ============================================================
  // CiA402 状态机
  // ============================================================

  /**
   * @brief 根据状态字产生下一步控制字。
   */
  uint16_t make_control_word(uint16_t status_word) const;

  /**
   * @brief 判断驱动器是否处于 Operation Enabled。
   */
  bool is_operation_enabled(uint16_t status_word) const;

  /**
   * @brief 判断驱动器是否存在 Fault。
   */
  bool is_fault(uint16_t status_word) const;

  // ============================================================
  // 单位转换和安全限制
  // ============================================================

  /**
   * @brief 编码器计数转关节弧度。
   */
  double counts_to_radians(int32_t counts) const;

  /**
   * @brief 关节弧度转编码器计数。
   */
  int32_t radians_to_counts(double radians) const;

  /**
   * @brief 对单周期目标位置变化量进行限幅。
   */
  int32_t limit_target_step(
    int32_t requested_counts,
    double period_seconds) const;

  /**
   * @brief 从 URDF hardware parameters 中读取参数。
   */
  bool parse_hardware_parameters();

  // ============================================================
  // ROS 2关节接口变量
  // ============================================================

  /// URDF中的关节名称，例如 joint6
  std::string joint_name_;

  /// 关节实际位置，单位rad
  double joint_position_{0.0};

  /// 关节实际速度，单位rad/s
  double joint_velocity_{0.0};

  /// ros2_control下发的目标位置，单位rad
  double joint_position_command_{0.0};

  /// 上一周期位置，用于估算速度
  double previous_joint_position_{0.0};

  /// 是否已经获得过有效位置反馈
  bool position_received_{false};

  /// 目标命令是否已经初始化为当前位置
  bool command_initialized_{false};

  /// 硬件接口是否处于active状态
  bool hardware_active_{false};

  // ============================================================
  // EtherCAT基本参数
  // ============================================================

  /// EtherLab主站编号，一般为0
  unsigned int master_index_{0};

  /// EtherCAT alias，一般为0
  uint16_t slave_alias_{0};

  /// 从站物理位置，当前验证slave 5
  uint16_t slave_position_{5};

  /// 零差云控Vendor ID
  uint32_t vendor_id_{0x5a65726f};

  /// 当前关节Product Code
  uint32_t product_code_{0x00029252};

  /// CSP模式：8
  int8_t operation_mode_{8};

  // ============================================================
  // 机械和编码器参数
  // ============================================================

  /// 电机或关节一圈对应的编码器计数
  double counts_per_revolution_{524288.0};

  /**
   * @brief ROS关节零位对应的绝对编码器值。
   *
   * 当前使用：
   *   262144 counts → 0 rad
   */
  int32_t zero_offset_counts_{262144};

  /**
   * @brief 运动方向。
   *
   * 1.0：ROS正方向与编码器增加方向一致
   * -1.0：ROS正方向与编码器增加方向相反
   */
  double direction_{1.0};

  /**
   * @brief 关节侧减速比。
   *
   * 如果驱动器的0x6064已经是关节侧计数，则保持1.0。
   */
  double gear_ratio_{1.0};

  /**
   * @brief 最大目标速度限制，单位rad/s。
   *
   * 默认2°/s：
   * 2 × pi / 180 ≈ 0.0349066 rad/s
   */
  double max_command_velocity_{0.034906585};

  /**
   * @brief ROS目标值相对当前位置允许的最大跳变，单位rad。
   *
   * 默认5°，防止错误目标导致大幅运动。
   */
  double max_command_jump_{0.087266463};

  // ============================================================
  // EtherLab对象
  // ============================================================

  /// EtherLab Master
  ec_master_t * master_{nullptr};

  /// EtherLab Process Data Domain
  ec_domain_t * domain_{nullptr};

  /// EtherCAT Slave Configuration
  ec_slave_config_t * slave_config_{nullptr};

  /// Domain过程数据首地址
  uint8_t * domain_data_{nullptr};

  // ============================================================
  // EtherLab状态
  // ============================================================

  ec_master_state_t master_state_{};

  ec_domain_state_t domain_state_{};

  ec_slave_config_state_t slave_config_state_{};

  /// 是否已经成功激活EtherLab Master
  bool master_activated_{false};

  /// 从站是否处于在线状态
  bool slave_online_{false};

  /// 从站是否处于Operational状态
  bool slave_operational_{false};

  // ============================================================
  // RxPDO偏移：主站发送到驱动器
  // ============================================================

  /**
   * 0x607A:00 Target Position，32位有符号。
   */
  unsigned int offset_target_position_{0};

  /**
   * 0x60FE:00 Digital Outputs，32位。
   */
  unsigned int offset_digital_outputs_{0};

  /**
   * 0x6040:00 Control Word，16位。
   */
  unsigned int offset_control_word_{0};

  // ============================================================
  // TxPDO偏移：驱动器发送到主站
  // ============================================================

  /**
   * 0x6064:00 Position Actual Value，32位有符号。
   */
  unsigned int offset_actual_position_{0};

  /**
   * 0x60FD:00 Digital Inputs，32位。
   */
  unsigned int offset_digital_inputs_{0};

  /**
   * 0x6041:00 Status Word，16位。
   */
  unsigned int offset_status_word_{0};

  // ============================================================
  // EtherCAT PDO数据
  // ============================================================

  /// 实际编码器位置
  int32_t actual_position_counts_{0};

  /// 最终写入0x607A的目标位置
  int32_t target_position_counts_{0};

  /// 上一周期写入的目标位置
  int32_t previous_target_counts_{0};

  /// 状态字0x6041
  uint16_t status_word_{0};

  /// 控制字0x6040
  uint16_t control_word_{0};

  /// 数字输入0x60FD
  uint32_t digital_inputs_{0};

  /// 数字输出0x60FE
  uint32_t digital_outputs_{0};

  /// 驱动器是否已经进入Operation Enabled
  bool drive_enabled_{false};

  // ============================================================
  // 运行状态和诊断
  // ============================================================

  /// read/write循环计数
  uint64_t cycle_count_{0};

  /// 连续成功接收PDO的周期数
  uint32_t valid_receive_cycles_{0};

  /**
   * 在允许执行外部位置命令之前，
   * 至少连续成功读取若干周期。
   */
  uint32_t required_valid_cycles_{100};
  /// 上一次Domain Working Counter
  unsigned int last_working_counter_{0};
  /// 是否检测到通信错误
  bool communication_error_{false};
};

}  // namespace zeroerr_ethercat_hardware

#endif  // ZEROERR_ETHERCAT_HARDWARE__ZEROERR_SYSTEM_HPP_
