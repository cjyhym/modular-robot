
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
