#include <ecrt.h>

#include <cerrno>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <time.h>
#include <unistd.h>

// ============================================================
// EtherCAT 从站配置
// ============================================================

static constexpr uint16_t MASTER_INDEX = 0;
static constexpr uint16_t SLAVE_ALIAS = 0;
static constexpr uint16_t SLAVE_POSITION = 5;

static constexpr uint32_t VENDOR_ID = 0x5a65726f;
static constexpr uint32_t PRODUCT_CODE = 0x00029252;

// 1 kHz
static constexpr uint32_t FREQUENCY = 1000;
static constexpr uint32_t PERIOD_NS = 1000000000UL / FREQUENCY;

// 程序自动运行 10 秒
static constexpr uint64_t MAX_CYCLES = 25ULL * FREQUENCY;

// 进入 Operation Enabled 后先保持 2 秒
static constexpr uint64_t MOTION_DELAY_CYCLES = 2ULL * FREQUENCY;

/*
 * 第一次必须保持为 0：
 * 0 = 只使能和保持，不主动运动。
 *
 * 完成保持测试后，可改成 100～200 做极小位移测试。
 */
static constexpr int32_t TEST_DELTA_COUNTS = 24564;

// ============================================================
// 运行状态
// ============================================================

static volatile sig_atomic_t g_running = 1;

static ec_master_t *master = nullptr;
static ec_domain_t *domain = nullptr;
static ec_slave_config_t *slave_config = nullptr;
static uint8_t *domain_pd = nullptr;

// PDO 在过程数据区中的偏移量
static unsigned int off_target_position = 0;
static unsigned int off_digital_outputs = 0;
static unsigned int off_control_word = 0;

static unsigned int off_actual_position = 0;
static unsigned int off_digital_inputs = 0;
static unsigned int off_status_word = 0;

// ============================================================
// 按 ethercat cstruct -p 5 的实际结果配置 PDO
// ============================================================

static ec_pdo_entry_info_t slave_5_pdo_entries[] = {
    {0x607A, 0x00, 32},  // Target Position
    {0x60FE, 0x00, 32},  // Digital Outputs
    {0x6040, 0x00, 16},  // Control Word

    {0x6064, 0x00, 32},  // Position Actual Value
    {0x60FD, 0x00, 32},  // Digital Inputs
    {0x6041, 0x00, 16},  // Status Word
};

static ec_pdo_info_t slave_5_pdos[] = {
    {0x1600, 3, slave_5_pdo_entries + 0},  // RxPDO
    {0x1A00, 3, slave_5_pdo_entries + 3},  // TxPDO
};

static ec_sync_info_t slave_5_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
    {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_5_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT,  1, slave_5_pdos + 1, EC_WD_DISABLE},
    {0xFF}
};

static ec_pdo_entry_reg_t domain_regs[] = {
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x607A,
        0x00,
        &off_target_position
    },
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x60FE,
        0x00,
        &off_digital_outputs
    },
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x6040,
        0x00,
        &off_control_word
    },
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x6064,
        0x00,
        &off_actual_position
    },
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x60FD,
        0x00,
        &off_digital_inputs
    },
    {
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE,
        0x6041,
        0x00,
        &off_status_word
    },
    {}
};

// ============================================================
// CiA 402 状态与控制字
// ============================================================

static constexpr uint16_t STATE_FAULT = 0x0008;
static constexpr uint16_t STATE_SWITCH_ON_DISABLED = 0x0040;
static constexpr uint16_t STATE_READY_TO_SWITCH_ON = 0x0021;
static constexpr uint16_t STATE_SWITCHED_ON = 0x0023;
static constexpr uint16_t STATE_OPERATION_ENABLED = 0x0027;

static constexpr uint16_t CW_DISABLE_VOLTAGE = 0x0000;
static constexpr uint16_t CW_SHUTDOWN = 0x0006;
static constexpr uint16_t CW_SWITCH_ON = 0x0007;
static constexpr uint16_t CW_ENABLE_OPERATION = 0x000F;
static constexpr uint16_t CW_FAULT_RESET = 0x0080;

static uint16_t decode_drive_state(uint16_t status_word)
{
    return status_word & 0x006F;
}

static const char *state_to_string(uint16_t state)
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

// ============================================================
// 时间与信号处理
// ============================================================

static uint64_t timespec_to_ns(const timespec &t)
{
    return static_cast<uint64_t>(t.tv_sec) * 1000000000ULL
           + static_cast<uint64_t>(t.tv_nsec);
}

static void add_ns(timespec &t, uint32_t ns)
{
    t.tv_nsec += static_cast<long>(ns);

    while (t.tv_nsec >= 1000000000L) {
        t.tv_nsec -= 1000000000L;
        ++t.tv_sec;
    }
}

static void signal_handler(int)
{
    g_running = 0;
}

// ============================================================
// 设置 CSP 模式
// ============================================================

static bool set_csp_mode(ec_master_t *master_ptr)
{
    // CiA 402 CSP = 8
    uint8_t mode = 8;
    uint32_t abort_code = 0;

    const int result = ecrt_master_sdo_download(
        master_ptr,
        SLAVE_POSITION,
        0x6060,
        0x00,
        &mode,
        sizeof(mode),
        &abort_code
    );

    if (result < 0) {
        std::fprintf(
            stderr,
            "设置 CSP 模式失败，result=%d，abort_code=0x%08X\n",
            result,
            abort_code
        );
        return false;
    }

    std::printf("Slave %u 已写入 CSP 模式：0x6060 = 8\n",
                SLAVE_POSITION);

    return true;
}

// ============================================================
// 发送一帧 PDO
// ============================================================

static void send_process_data()
{
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);

    ecrt_domain_queue(domain);

    ecrt_master_application_time(
        master,
        timespec_to_ns(now)
    );

    ecrt_master_sync_reference_clock(master);
    ecrt_master_sync_slave_clocks(master);

    ecrt_master_send(master);
}

// ============================================================
// 主函数
// ============================================================

int main()
{
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::printf("============================================\n");
    std::printf("ZeroErr Slave 5 CSP 安全保持测试\n");
    std::printf("Slave position       : %u\n", SLAVE_POSITION);
    std::printf("Cycle frequency      : %u Hz\n", FREQUENCY);
    std::printf("Test delta counts    : %d\n", TEST_DELTA_COUNTS);
    std::printf("Automatic stop       : 10 s\n");
    std::printf("============================================\n");

    master = ecrt_request_master(MASTER_INDEX);

    if (master == nullptr) {
        std::fprintf(stderr, "申请 EtherCAT Master 0 失败\n");
        return EXIT_FAILURE;
    }

    /*
     * 在激活主站之前，通过 SDO 将运行模式设为 CSP。
     */
    if (!set_csp_mode(master)) {
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    domain = ecrt_master_create_domain(master);

    if (domain == nullptr) {
        std::fprintf(stderr, "创建 EtherCAT Domain 失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    slave_config = ecrt_master_slave_config(
        master,
        SLAVE_ALIAS,
        SLAVE_POSITION,
        VENDOR_ID,
        PRODUCT_CODE
    );

    if (slave_config == nullptr) {
        std::fprintf(stderr, "获取 Slave 5 配置失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    /*
     * 使用从 cstruct 得到的真实 PDO 配置。
     */
    if (ecrt_slave_config_pdos(
            slave_config,
            EC_END,
            slave_5_syncs) != 0) {

        std::fprintf(stderr, "配置 Slave 5 PDO 失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    /*
     * 配置 DC Sync0，周期为 1 ms。
     */
    ecrt_slave_config_dc(
        slave_config,
        0x0300,
        PERIOD_NS,
        0,
        0,
        0
    );

    if (ecrt_domain_reg_pdo_entry_list(
            domain,
            domain_regs) != 0) {

        std::fprintf(stderr, "注册 PDO Entry 失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    std::printf("正在激活 EtherCAT Master...\n");

    if (ecrt_master_activate(master) != 0) {
        std::fprintf(stderr, "激活 EtherCAT Master 失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    domain_pd = ecrt_domain_data(domain);

    if (domain_pd == nullptr) {
        std::fprintf(stderr, "获取 Domain Process Data 失败\n");
        ecrt_release_master(master);
        return EXIT_FAILURE;
    }

    timespec wakeup_time{};
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    uint64_t cycle_count = 0;
    uint64_t operation_enabled_cycles = 0;

    int32_t actual_position = 0;
    int32_t hold_position = 0;
    int32_t command_position = 0;

    bool command_initialized = false;

    uint16_t last_state = 0xFFFF;

    while (g_running && cycle_count < MAX_CYCLES) {
        add_ns(wakeup_time, PERIOD_NS);

        const int sleep_result = clock_nanosleep(
            CLOCK_MONOTONIC,
            TIMER_ABSTIME,
            &wakeup_time,
            nullptr
        );

        if (sleep_result != 0 && sleep_result != EINTR) {
            std::fprintf(
                stderr,
                "clock_nanosleep 失败：%s\n",
                std::strerror(sleep_result)
            );
        }

        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        ec_domain_state_t domain_state{};
        ecrt_domain_state(domain, &domain_state);

        actual_position = EC_READ_S32(
            domain_pd + off_actual_position
        );

        const uint16_t status_word = EC_READ_U16(
            domain_pd + off_status_word
        );

        const uint16_t drive_state =
            decode_drive_state(status_word);

        /*
         * 只有工作计数完整、状态字有效后，
         * 才使用实际位置初始化目标位置。
         */
        if (!command_initialized
            && domain_state.wc_state == EC_WC_COMPLETE
            && status_word != 0) {

            hold_position = actual_position;
            command_position = actual_position;
            command_initialized = true;

            std::printf(
                "目标位置初始化完成：actual=%d\n",
                actual_position
            );
        }

        /*
         * 在任何使能控制字发出之前，
         * 先把目标位置设置成当前位置。
         */
        if (command_initialized) {
            EC_WRITE_S32(
                domain_pd + off_target_position,
                command_position
            );
        }

        uint16_t control_word = CW_DISABLE_VOLTAGE;

        switch (drive_state) {
            case STATE_FAULT:
                control_word = CW_FAULT_RESET;
                break;

            case STATE_SWITCH_ON_DISABLED:
                control_word = CW_SHUTDOWN;
                break;

            case STATE_READY_TO_SWITCH_ON:
                control_word = CW_SWITCH_ON;
                break;

            case STATE_SWITCHED_ON:
                /*
                 * 没有完成目标位置初始化时，
                 * 不允许进入 Operation Enabled。
                 */
                control_word = command_initialized
                               ? CW_ENABLE_OPERATION
                               : CW_SWITCH_ON;
                break;

            case STATE_OPERATION_ENABLED:
                control_word = CW_ENABLE_OPERATION;

                if (command_initialized) {
                    ++operation_enabled_cycles;

                    /*
                     * 先保持 2 秒，再逐计数缓慢移动。
                     * 第一次测试 TEST_DELTA_COUNTS=0，
                     * 因而不会主动运动。
                     */
                    if (operation_enabled_cycles
                        > MOTION_DELAY_CYCLES) {

                        const int32_t goal_position =
                            hold_position
                            + TEST_DELTA_COUNTS;

                        if (command_position < goal_position) {
                            ++command_position;
                        } else if (
                            command_position > goal_position) {

                            --command_position;
                        }
                    }
                }

                break;

            default:
                control_word = CW_SHUTDOWN;
                break;
        }

        EC_WRITE_U16(
            domain_pd + off_control_word,
            control_word
        );

        /*
         * 本测试不操作 0x60FE 数字输出。
         * 不要在不了解制动器/IO定义时随意改变它。
         */

        send_process_data();

        if (drive_state != last_state
            || cycle_count % FREQUENCY == 0) {

            std::printf(
                "cycle=%llu, "
                "state=%s, "
                "status=0x%04X, "
                "control=0x%04X, "
                "actual=%d, "
                "command=%d, "
                "wc=%u\n",
                static_cast<unsigned long long>(
                    cycle_count),
                state_to_string(drive_state),
                status_word,
                control_word,
                actual_position,
                command_position,
                domain_state.working_counter
            );

            last_state = drive_state;
        }

        ++cycle_count;
    }

    /*
     * 退出前连续发送 Disable Voltage 约 200 ms。
     */
    std::printf("正在撤销关节使能...\n");

    for (int i = 0; i < 200; ++i) {
        ecrt_master_receive(master);
        ecrt_domain_process(domain);

        if (command_initialized) {
            EC_WRITE_S32(
                domain_pd + off_target_position,
                command_position
            );
        }

        EC_WRITE_U16(
            domain_pd + off_control_word,
            CW_DISABLE_VOLTAGE
        );

        send_process_data();

        usleep(1000);
    }

    ecrt_release_master(master);
    master = nullptr;

    std::printf("测试结束，EtherCAT Master 已释放。\n");

    return EXIT_SUCCESS;
}
