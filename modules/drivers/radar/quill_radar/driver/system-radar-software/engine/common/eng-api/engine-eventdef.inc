/* this file is included by engine code mutliple times to define enumeration and string tables. Define all event
* handlers for this module here.  Use no comments after the macros */

DEFINE_EVENT(RHAL_TRY_SCAN_START)
DEFINE_EVENT(RHAL_WAIT_APROBE_COMPLETE)
DEFINE_EVENT(RHAL_DELAYED_SCAN_START)
DEFINE_EVENT(RHAL_LONG_DELAY_SCAN_START)
DEFINE_EVENT(RHAL_POLL_SCAN_COMPLETE)
DEFINE_EVENT(RHAL_POLL_PROC_COMPLETE)
DEFINE_EVENT(RHAL_POLL_MCEI_COMPLETE)
DEFINE_EVENT(RHAL_MICROCAL_DONE_TRIGGER)
DEFINE_EVENT(RHAL_SHARED_RDC1_TRIGGER)
DEFINE_EVENT(RHAL_DPROBE_PPA_EVENT)
DEFINE_EVENT(RHAL_MICROCAL_TRIGGER)
DEFINE_EVENT(RHAL_MICROCAL_MEASURE_DC_EVT)
DEFINE_EVENT(ISR_RHAL_RAU_START)
DEFINE_EVENT(ISR_RHAL_RAU_STOP)
DEFINE_EVENT(ISR_RHAL_FEU_START)
DEFINE_EVENT(ISR_RHAL_FEU_STOP)
DEFINE_EVENT(ISR_RHAL_PGU_START)
DEFINE_EVENT(ISR_RHAL_PGU_STOP)
DEFINE_EVENT(ISR_RHAL_MCE_START)
DEFINE_EVENT(ISR_RHAL_MCE_STOP)
DEFINE_EVENT(HAL_HSM_SAFETY_MESSAGE)
DEFINE_EVENT(HAL_CCP_CONTROL_MESSAGE)
DEFINE_EVENT(RLA_ANALOG_CONFIG_EVENT)
DEFINE_EVENT(RHAL_ANALOG_CONFIG_EVENT)

DEFINE_EVENT(CAL_DPROBE_COMPLETE)
DEFINE_EVENT(CAL_DPROBE_AVG_DC_READY)
DEFINE_EVENT(CAL_FINALIZE_DC_MEASURE_DC)
DEFINE_EVENT(MEGA_CAL_DC_MEASURE_DC)
DEFINE_EVENT(MEGA_CAL_TEMP_MEASURE_COMPLETE)
DEFINE_EVENT(MEGA_CAL_WRITE_FLASH_COMPLETE)
DEFINE_EVENT(MEGA_CAL_FC_RELEASED)
DEFINE_EVENT(CAL_WRITE_FLASH_COMPLETE)
DEFINE_EVENT(CAL_APROBE_COMPLETE)
DEFINE_EVENT(APROBE_BULK_MEASUREMENT)
DEFINE_EVENT(CAL_LDO_MEASUREMENT_COMPLETE)
DEFINE_EVENT(CAL_PEAK_DET_MEASUREMENT_COMPLETE)
DEFINE_EVENT(CAL_PER_REGULATOR_COMPLETE)
DEFINE_EVENT(CAL_TX_SUP_LDO_CAL_DONE)
DEFINE_EVENT(CAL_TX_SUP_FLASH_DONE)
DEFINE_EVENT(MINI_CAL_DC_MEASURE_DC)
DEFINE_EVENT(DPROBE_PGU_STARTED)
DEFINE_EVENT(RLA_SCAN_APROBE_COMPLETE)
DEFINE_EVENT(RLA_PEAK_DET_COMPLETE)

DEFINE_EVENT(RDC_RHAL_INIT_COMPLETE)
DEFINE_EVENT(RDC_SCAN_STARTED)
DEFINE_EVENT(RDC_SCAN_COMPLETE)
DEFINE_EVENT(RDC_PROC_COMPLETE)
DEFINE_EVENT(RDC_CHAN_COMPLETE)
DEFINE_EVENT(RDC_COV_COMPLETE)
DEFINE_EVENT(RDC_SVD_COMPLETE)
DEFINE_EVENT(RDC_RETRY_SI_START_PROCESSING)
DEFINE_EVENT(RDC1_PLAYBACK_FC_CALLBACK)
DEFINE_EVENT(UHDP_RDC1_PLAYBACK_CALLBACK)
DEFINE_EVENT(RDC_CORRECTION_CAL_CALLBACK)
DEFINE_EVENT(CORRECTION_COMPLETION_CALLBACK)

DEFINE_EVENT(DIAG_TRIGGER_EVENT)
DEFINE_EVENT(DIAG_COMPLETE_EVENT)
DEFINE_EVENT(DIAG_SCAN_PAUSE_START)
DEFINE_EVENT(DIAG_DMA_COMPLETE)
DEFINE_EVENT(DIAG_STD_LAB_COMPLETE)
DEFINE_EVENT(DIAG_STD_LAB_CONT_SCAN)
DEFINE_EVENT(DIAG_STD_RHAL_DCMINICAL_COMPLETE)
DEFINE_EVENT(DIAG_STD_RHAL_DCMICROCAL_COMPLETE)
DEFINE_EVENT(DIAG_CAL_FLASH_WRITE_COMPLETE)
DEFINE_EVENT(DIAG_STD_RHAL_DCMEGACAL_COMPLETE)
DEFINE_EVENT(DIAG_STD_RDC_SCAN_COMPLETE)
DEFINE_EVENT(DIAG_STD_RDC_APROBE_COMPLETE)
DEFINE_EVENT(DIAG_STD_RDC_PEAKDET_COMPLETE)
DEFINE_EVENT(DIAG_ETH_TX_COMPLETE)
DEFINE_EVENT(DIAG_FLASH_COMPLETE)
DEFINE_EVENT(HSM_JUMP_UART_MONITOR)
DEFINE_EVENT(DIAG_CAL_LDO_COMPLETE)
DEFINE_EVENT(DIAG_CAL_STEP_COMPLETE)
DEFINE_EVENT(DIAG_CAL_TX_SUPPRESSION_DONE)
DEFINE_EVENT(DIAG_APROBE_SINGLE_COMPLETE)
DEFINE_EVENT(LDO_CAL_ENGINE)
DEFINE_EVENT(DIAG_CRU_RAMP_WAIT_COMPLETE)
DEFINE_EVENT(DIAG_DSP_UNIT_TEST_COMPLETE)
DEFINE_EVENT(DIAG_STD_RHAL_MEASURE_DC_COMPLETE)

DEFINE_EVENT(DIAG_GUEST_EVENT_CHECK)
DEFINE_EVENT(DIAG_NOC_POWER_IDLE_CHECK)
DEFINE_EVENT(DIAG_FEU_POLL_COMPLETE)
DEFINE_EVENT(DIAG_SCAN_COMPLETE_CHECK)

DEFINE_EVENT(UHDP_DIAG_COMPLETE)
DEFINE_EVENT(UHDP_SCAN_COMPLETE)
DEFINE_EVENT(UHDP_LOGEVENT_TIMER)
DEFINE_EVENT(UHDP_DISCONNECT_TIMER)
DEFINE_EVENT(UHDP_RDATA_TX_COMPLETE)
DEFINE_EVENT(UHDP_DIAG_TX_COMPLETE)
DEFINE_EVENT(UHDP_RDATA_CONTINUE)
DEFINE_EVENT(UHDP_ENV_CONTROL_RESEND_TIMER)
DEFINE_EVENT(UHDP_ENV_CONTROL_TX_COMPLETE)

DEFINE_EVENT(RPC_QUEUE_EVENT)
DEFINE_EVENT(RFPLL_VCO_TIMER)
DEFINE_EVENT(RFPLL_VCO_TUNE)
DEFINE_EVENT(REMOTE_SYS_WRITE_COMPLETE)
DEFINE_EVENT(CCP_PWR_MON)

DEFINE_EVENT(AUTO_TEMP_MEASUREMENT)

DEFINE_EVENT(APROBE_HANDLER_SINGLE_START)
DEFINE_EVENT(APROBE_HANDLER_SINGLE_COMPLETE)
DEFINE_EVENT(APROBE_HANDLER_SINGLE_TX)
DEFINE_EVENT(APROBE_HANDLER_SINGLE_SH)
DEFINE_EVENT(APROBE_HANDLER_SINGLE_RX)

DEFINE_EVENT(SABINE_RESET_FLASH_SHUT_DOWN_EVENT)
DEFINE_EVENT(BLOCK_APROBE_COMPLETE)
DEFINE_EVENT(RHAL_AUTO_MINICAL_EVT_HANDLER)
DEFINE_EVENT(DIAG_FAST_MEASURE_DC_SCAN_DONE_EVT)
DEFINE_EVENT(DIAG_CALFLASH_WRITE_EVENT)

DEFINE_EVENT(RHAL_PERIODIC_MEASUREMENT_EVT)
DEFINE_EVENT(RHAL_ANALOG_MEASUREMENTS_EVT)

DEFINE_EVENT(GTEST_ABORT_MAIN_LOOP)
DEFINE_EVENT(GTEST_COMPOSE_SCANS_1)
DEFINE_EVENT(GTEST_COMPOSE_SCANS_2)
DEFINE_EVENT(GTEST_COMPOSE_SCANS_3)
DEFINE_EVENT(GTEST_COMPOSE_SCANS_4)
DEFINE_EVENT(REMOTE_AES_OPERATION_STATUS)
DEFINE_EVENT(REMOTE_DMAC_OPERATION_STATUS)

DEFINE_EVENT(BIST_DONE_EVENT)
DEFINE_EVENT(BIST_TEST_EVENT)
DEFINE_EVENT(DSP_LBIST_IDMA)

DEFINE_EVENT(THERMAL_ERROR_HANDLER)
DEFINE_EVENT(THERMAL_WARNING_HANDLER)



/* these events are registered by env-uhnder, but we keep them here to make the deployed environment file empty */
DEFINE_EVENT(IP_RECEIVE_EVENT)
DEFINE_EVENT(ARP_RECEIVE_EVENT)
DEFINE_EVENT(UART_INTERRUPT)
DEFINE_EVENT(CAL_MGR_CAL_STEP_COMPLETE)
DEFINE_EVENT(OBJ_RDC_LAYER_READY)
DEFINE_EVENT(OBJ_BOOT_CAL_COMPLETE)
DEFINE_EVENT(OBJ_SCAN_COMPLETE)
DEFINE_EVENT(OBJ_MUSIC_COMPLETE)
DEFINE_EVENT(OBJ_FRAME_CONFIG_RELEASED)
DEFINE_EVENT(FEAT_SOFTWARE_STOP)
DEFINE_EVENT(SSPI_TX_TIMER)
DEFINE_EVENT(DIAG_SSPI_TX_COMPLETE)

DEFINE_EVENT(FLASH_INIT_COMPLETE)
DEFINE_EVENT(CAL_LOAD_FINISH)

DEFINE_EVENT(IQ_CAL_SCAN_COMPLETE)
DEFINE_EVENT(BF_CAL_SCAN_COMPLETE)
DEFINE_EVENT(VRX_ALIGN_SCAN_COMPLETE)
DEFINE_EVENT(VRX_ALIGN_FC_RELEASED)
DEFINE_EVENT(TCS_CAL_SCAN_COMPLETE)
DEFINE_EVENT(DC_CAL_SCAN_COMPLETE)
DEFINE_EVENT(DC_CAL_AUTO_TRIGGER)
DEFINE_EVENT(LDO_CAL_SCAN_COMPLETE)
DEFINE_EVENT(LDO_CAL_SINGLE_COMPLETE)
DEFINE_EVENT(QILO_CAL_SCAN_COMPLETE)
DEFINE_EVENT(PEAK_DET_CAL_SCAN_COMPLETE)
DEFINE_EVENT(PEAK_DET_CAL_SINGLE_COMPLETE)

DEFINE_EVENT(DC_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(TCS_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(VRX_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(IQ_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(TEMP_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(ANGLE_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(DIAGONAL_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(STEERING_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(QILO_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(LDO_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(VTR_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(EFUSE_TRIMS_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(CDATA_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(LDO_PARAMS_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(PKDET_CAL_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(TXGAIN_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(TXBW_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(RXGAIN_FLASH_SAVE_COMPLETE)
DEFINE_EVENT(RXBW_FLASH_SAVE_COMPLETE)

DEFINE_EVENT(POWER_LOG_TIMER_EVENT)
DEFINE_EVENT(POWER_LOG_FLASH_FILE_READ_COMPLETE)
DEFINE_EVENT(POWER_LOG_FLASH_FILE_WRITE_COMPLETE)

DEFINE_EVENT(HSM_TEST)
DEFINE_EVENT(CCP_TEST)
DEFINE_EVENT(SCP_TEST)
DEFINE_EVENT(DSP1_TEST)
DEFINE_EVENT(DSP2_TEST)
DEFINE_EVENT(DSP1_INF_LOOP)
DEFINE_EVENT(DSP2_INF_LOOP)

DEFINE_EVENT(TFTP_FLASH_WRITE_COMPLETE)
DEFINE_EVENT(TFTP_FLASH_READ_COMPLETE)
DEFINE_EVENT(WATCHDOG_TIMER)
DEFINE_EVENT(NETWORK_FLASH_READ_COMPLETE)
DEFINE_EVENT(IOVEC_ALLOC_FAIL_TIMER)
DEFINE_EVENT(SABINE_RESET_EVENT)
DEFINE_EVENT(GDB_INIT_TIMER)
DEFINE_EVENT(DIAG_AES_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_DMAC_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_STU_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_GIC_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_NOC_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_EFUSEREADKEYHASH_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_EFUSEPROGRAMLIFECYCLE_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_EFUSEPROGRAMKEYHASH_OPERATION_COMPLETE)
DEFINE_EVENT(DIAG_PVT_OPERATION_COMPLETE)
DEFINE_EVENT(SCP_THERMAL_MONITOR)
DEFINE_EVENT(SCP_VOLTAGE_MONITOR)
DEFINE_EVENT(DIAG_ADIE_EVENT_CHECK)
DEFINE_EVENT(DIAG_FPU_COMPLETE)
DEFINE_EVENT(DIAG_HSM_MPU_COMPLETE)
DEFINE_EVENT(FPU_EXCP_EVENT)
DEFINE_EVENT(ETH_GMAC_LINK_INTR_EVENT)
DEFINE_EVENT(CHIP_VOLTAGE_CALIBRATION_EVENT)
DEFINE_EVENT(DIAG_VOLTAGE_CALIB_COMPLETE)

DEFINE_EVENT(SCP_WDT_CONFIGURE)
DEFINE_EVENT(BCS_FW_UPDATE)
DEFINE_EVENT(BCS_MCFG_FW_UPDATE)
DEFINE_EVENT(DIAG_I2C_JUMP_EVENT)
DEFINE_EVENT(LIST_CMD_EVENT)
DEFINE_EVENT(CAN_MSG_RECEIVE_UPDATE)
DEFINE_EVENT(CAN_MSG_TRANSMIT_UPDATE)
DEFINE_EVENT(CRYPTO_AES_OPERATION)
DEFINE_EVENT(CAN_MSG_RECEIVE_TIMEOUT)
DEFINE_EVENT(ALL_RAIL_CUR_SEND_REQUEST)
DEFINE_EVENT(ALL_RAIL_CUR_READ_RESPONSE)