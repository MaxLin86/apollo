/* define log message strings, can use up to two 32bit integer string arguments */

DEFINE_MESSAGE(UHDP_INIT, "UhDP protocol agent initialized")
DEFINE_MESSAGE(UHDP_DIAG_DRAM_ALLOC_FAIL, "Unable to alloc %d bytes of DRAM for diagnostic tests")
DEFINE_MESSAGE(UHDP_DIAG_TEST_TOO_BIG, "Diagnostic test instance is too large, max %d bytes")
DEFINE_MESSAGE(UHDP_DIAG_STILL_RUNNING, "Previous diagnostic test has not completed, discarding new request")
DEFINE_MESSAGE(UHDP_LOG_DIAG_COMPLETED, "Diagnostic test completed: %d output bytes")
DEFINE_MESSAGE(UHDP_CONTROL_UPDATED, "Datalog Control Updated, logger mode = %d")
DEFINE_MESSAGE(UHDP_SCAN_CONTROL_UPDATED, "Scan Control Updated, command = %d")
DEFINE_MESSAGE(UHDP_SCAN_READY, "Scan Ready received (seq_no = %d)")
DEFINE_MESSAGE(UHDP_DISCARD_RSI_CAPTURE_MODE, "Data logger not in normal mode, discarding RSI")
DEFINE_MESSAGE(UHDP_SENT_SCAN_INFO, "Sent RSI ScanInfo with UHDP sequence number %u, ScanID %u")
DEFINE_MESSAGE(UHDP_RETURNED_IOVEC, "ethernet driver returned %d iovecs")
DEFINE_MESSAGE(UHDP_CONTINUE_NEXT_RSI, "Continuing to next queued RSI")
DEFINE_MESSAGE(UHDP_WARN_IOVEC_COUNT, "WARNING: More IOVECs being returned than were recorded as in-use")
DEFINE_MESSAGE(UHDP_TOO_SHORT, "UHDP message is smaller than UHDP header, %d bytes")
DEFINE_MESSAGE(UHDP_TYPE_ALREADY_REGISTERED, "UHDP message type %d is already registered")
DEFINE_MESSAGE(UHDP_INVALID_LENGTH, "UHDP total length does not match: %d != %d")
DEFINE_MESSAGE(UHDP_INVALID_VERSION, "UHDP packet version %d is not SRS UHDP version %d")
DEFINE_MESSAGE(UHDP_UNKNOWN_TYPE, "UHDP packet unknown message type %d")
DEFINE_MESSAGE(UHDP_HELLO_WRONG_LENGTH, "UHDP hello packet was malformed")
DEFINE_MESSAGE(UHDP_HELLO_WRONG_MAC, "UHDP hello packet has different MAC address, ignored")
DEFINE_MESSAGE(UHDP_HELLO_WRONG_IP, "UHDP hello packet has different radar IP, ignored")
DEFINE_MESSAGE(UHDP_HELLO_RECEIVED, "UHDP hello packet received")
DEFINE_MESSAGE(UHDP_HELLO_ALREADY_PEERED, "UHDP hello packet ignored, already peered")
DEFINE_MESSAGE(UHDP_WRONG_SOURCE_PORT, "UHDP message received from unexpected source port %d (peer port %d)")
DEFINE_MESSAGE(RDC_INIT, "RDC Layer initializing....")
DEFINE_MESSAGE(RDC_INIT_DONE, "************************ Ready for normal operation ************************")
DEFINE_MESSAGE(RDC_NOT_READY, "RDC Layer initialization not yet complete.  Please wait...")
DEFINE_MESSAGE(RDC_NOT_INITIALIZED, "RDC Layer not initialized.")
DEFINE_MESSAGE(RDC_INTERNAL_ERROR, "Internal error")
DEFINE_MESSAGE(RDC_SCAN_CONF, "Finished configuring scan %d of %d")
DEFINE_MESSAGE(RDC_SCAN_CALLBACK, "Scan complete")
DEFINE_MESSAGE(RDC_ADC_SIZE, "User asked for %u ADC Samples, but scan can only produce %lu ADC samples")
DEFINE_MESSAGE(RDC_VRX_ALIGN_SCAN_CONFIG_FAIL, "Vrx Align :: scan config failed")
DEFINE_MESSAGE(RDC_VRX_ALIGN_TARGET_DETECT_FAIL, "Vrx align :: target/spillover detection failed")
DEFINE_MESSAGE(RDC_VRX_ALIGN_TARGET_DETECT_PASS, "Vrx align :: target/spillover detection passed")
DEFINE_MESSAGE(RDC_VRX_OFFSET_SCAN_CONFIG_FAIL, "Vrx align :: scan config failed for vrx offset measure")
DEFINE_MESSAGE(RDC_VRX_ALIGN_SPILLOVER_DETECT_FAIL, "Vrx align :: spillover detection failed for vrx offset measure")
DEFINE_MESSAGE(RDC_VRX_ALIGN_FAIL, "Vrx align processing :: failed")
DEFINE_MESSAGE(RDC_VRX_ALIGN_INVALID_DB_KEY, "vrx alignment ERROR :: invalid key for vrx offsets")
DEFINE_MESSAGE(RDC_VRX_ALIGN_MISSING_KEY, "Vrx field mode alignment :: required key(s) missing")
DEFINE_MESSAGE(RDC_VRX_ALIGN_ANTNA_CONFIG_FAIL, "vrx alignment ERROR :: failed to configure Rx and Tx antenna set")
DEFINE_MESSAGE(RHAL_VRX_MEDIAN_OPEN_ERROR, "vrx cal write failed :: unable to open offset median file")
DEFINE_MESSAGE(RHAL_VRX_MEDIAN_ACCESS_ERROR, "vrx cal write failed :: unable to acces offset median file")
DEFINE_MESSAGE(RHAL_VRX_MEDIAN_SIZE_ERROR, "vrx cal write failed :: invalid offset median file size %d")
DEFINE_MESSAGE(RDC_RANGE_CAL_INIT_FAIL, "Range Cal :: initialization failed")
DEFINE_MESSAGE(RDC_RANGE_CAL_FILE_WRITE_FAIL, "Range Cal :: tfs file write failed")
DEFINE_MESSAGE(RDC_RANGE_CAL_SCAN_CONFIG_FAIL, "Range Cal :: scan config failed")
DEFINE_MESSAGE(RDC_RANGE_CAL_TARGET_DETECT_FAIL, "Range Cal :: target detection failed for vp mode %d, chip rate %d")
DEFINE_MESSAGE(RDC_VRX_ALIGN_HEADER,    "Vrx align      : misalign (ps), desired (ps)")
DEFINE_MESSAGE(RDC_RX_ALIGN_INFO,       "Rx align       :    %10d,   %10d")
DEFINE_MESSAGE(RDC_TX_PASS1_ALIGN_INFO, "Tx Pass1 align :    %10d,   %10d")
DEFINE_MESSAGE(RDC_TX_PASS2_ALIGN_INFO, "Tx Pass2 align :    %10d,   %10d")
DEFINE_MESSAGE(RDC_RX_ALIGN_MAX_ITER, "Max iterations reached for Vrx Align Rx")
DEFINE_MESSAGE(RDC_TX_ALIGN_MAX_ITER, "Max iterations reached for Vrx Align TX Pass %d")
DEFINE_MESSAGE(RDC_CAL_SCAN_CIU_OVERFLOW, "Calibration scan has CIU overflow errors")
DEFINE_MESSAGE(RDC_CAL_WARMUP_SCAN, "Calibration warmup scan complete, %d remaining")


DEFINE_MESSAGE(RHAL_INIT, "Radar HAL initialized")
DEFINE_MESSAGE(RHAL_INIT_NON_STD_MODE, "RHAL is configured in non-standard mode")
DEFINE_MESSAGE(RHAL_GAINMGR_CIU_NOT_CONVERGED, "CIU gains have not converged underflow %d overflow %d")
DEFINE_MESSAGE(RHAL_GAINMGR_FEU_NOT_CONVERGED, "FEU gains have not converged")
DEFINE_MESSAGE(RHAL_GAINMGR_RAU_NOT_CONVERGED, "RAU gains have not converged max_db %d overflow %d")
DEFINE_MESSAGE(RHAL_GAINMGR_RAU_NOT_CONVERGED_CHAN, "RAU channelizer gains have not converged overflow %d")
DEFINE_MESSAGE(RHAL_DC_VALUE_COULDNOT_CONVERGE, "DC Minical for RX %d: minical didn't converge: DC values are more than +/-8")
DEFINE_MESSAGE(RHAL_DC_VALUE_ACROSS_LANE, "DC Minical for Rx%d: DC difference across Lane is more than %d")
DEFINE_MESSAGE(RHAL_DC_VALUE_NOT_LINEAR_ACROSS_LANE_1, "DC Minical: DC diff across Lane is more than 4")
DEFINE_MESSAGE(RHAL_DC_VALUE_NOT_LINEAR_ACROSS_LANE_2, "DC Minical for RX %d: minical converged, but DC diff across Lane is more than 4")
DEFINE_MESSAGE(RHAL_MICRODC_VAL_COULDNOT_CONVERGE, "DC Microcal for RX %d: microcal didn't converge: DC values are more than +/-8")
DEFINE_MESSAGE(RHAL_MICRODC_VAL_NOT_LINEAR_ACROSS_LANE_1, "DC Microcal: DC diff across Lane is more than 4")
DEFINE_MESSAGE(RHAL_MICRODC_VAL_NOT_LINEAR_ACROSS_LANE_2, "DC Microcal for RX %d: microcal converged, but DC diff across Lane is more than 6")
DEFINE_MESSAGE(RHAL_MINICAL_INCOMPLETE, "Mini-DC-Calibration was incomplete. Scan completed before DC measurement (ADC/VGA1/VGA3 codes didn't converge DC)")
DEFINE_MESSAGE(RHAL_MICROCAL_INCOMPLETE, "Micro DC Cal incomplete. Scan completed before D-Probe DC measurement")
DEFINE_MESSAGE(RHAL_MICROCAL_FALSE_SCAN_END, "False Scan End(s) Detected :-( %d")
DEFINE_MESSAGE(RHAL_NO_FALSE_SCAN_END, "No False Scan Ends Detected :-)")
DEFINE_MESSAGE(RHAL_MICROCAL_CONVERGED_PROD, "Micro-DC-Calibration is DONE with all the DC values are OK.")
DEFINE_MESSAGE(RHAL_MICROCAL_NOT_CONVERGED_PROD, "Micro-DC-Calibration completed but DC didn't converge: DC values are more than +/-8.")
DEFINE_MESSAGE(RHAL_MINICAL_CONVERGED_PROD_1, "Mini-DC-Calibration is DONE with all the DC values are OK.")
DEFINE_MESSAGE(RHAL_MINICAL_CONVERGED_PROD_2, "Mini-DC-Calibration is DONE; minical converged, but DC diff across Lanes is more than 3.")
DEFINE_MESSAGE(RHAL_ANALOG_IMPL_PLL_LOCK_FAIL, "PLL Lock Failed. reverting to boot_mode=0")
DEFINE_MESSAGE(RHAL_ANALOG_CFG_FAIL, "Analog Config Failed")
DEFINE_MESSAGE(RHAL_UNSUPPORTED_CHIP_ID, "Only T1, T5 and T20(AZ-T5) supported for now; Switching back to boot_mode = 0 !!!")
DEFINE_MESSAGE(RHAL_ADC_HW_FAIL, "ADC HW trigger cal failed!!")
DEFINE_MESSAGE(RHAL_ADC_DC_FAIL, "ADC dc correction for Rx%d Failed, Proceeding further")
DEFINE_MESSAGE(RHAL_VGA1_DC_FAIL, "VGA1 dc correction for Rx%d Failed, Proceeding further")
DEFINE_MESSAGE(RHAL_VGA3_DC_FAIL, "VGA3 dc correction for Rx%d Failed, Proceeding further")
DEFINE_MESSAGE(RHAL_ADI_BAD_SYNC, "ADI Sync status BAD (ADI_SYNC_SENSE=%08x)!")
DEFINE_MESSAGE(RHAL_ADI_OK_SYNC, "ADI Sync status OK (ADI_SYNC_SENSE=%08x)!")
DEFINE_MESSAGE(RHAL_ADI_SYNC_ALIGN, "Align RXs based on ADI Sync status")
DEFINE_MESSAGE(RHAL_INVALID_CORR_LENGTH, "Invalid correlation length %d")
DEFINE_MESSAGE(RHAL_INVALID_RX_DELAY, "Invalid rx_delay value %d")
DEFINE_MESSAGE(RHAL_INVALID_RX_DELAY_SET, "rx_delay for non 125/250/500MCPS is set to %d")
DEFINE_MESSAGE(RHAL_INVALID_TX_SQUELCH, "Invalid tx early squelch value %d")
DEFINE_MESSAGE(RHAL_INVALID_TX_SQUELCH_SET, "tx early squelch for non 125/250/500MCPS is set to %d")
DEFINE_MESSAGE(RHAL_INVALID_RX_SQUELCH, "Invalid rx early unsquelch value %d")
DEFINE_MESSAGE(RHAL_INVALID_RX_SQUELCH_SET, "rx early unsquelch for non 125/250/500MCPS is set to %d")
DEFINE_MESSAGE(RHAL_INVALID_RANGE_MAP_PTR, "invalid ptr, range bin map not created")
DEFINE_MESSAGE(RHAL_POWER_OUT_OF_RANGE, "power_set_volt %dmv value is out of range, ignoring the setting")
DEFINE_MESSAGE(RHAL_POWER_I2C_COMM_FAILURE, "power_set_volt %dmv i2c communication failure, setting is not applied")
DEFINE_MESSAGE(RHAL_POWER_36_OUT_OF_RANGE, "power_set_volt 3.6v value is out of range, ignoring the setting")
DEFINE_MESSAGE(RHAL_POWER_36_I2c_COMM_FAILURE, "power_set_volt 3.6v i2c communication failure, setting is not applied")
DEFINE_MESSAGE(RHAL_CONFIG_VERSION_MISMATCH, "requested rhal config version (%d), Flashed SW is at version (%d)")
DEFINE_MESSAGE(RHAL_CONFIG_VERSION_DEPRECATED, "requested rhal config version (%d) deprecated, using factory defaults")
DEFINE_MESSAGE(RHAL_ANALOG_INVALID_SAMPLERATE, "Invalid Sampling rate. Forcing to 1GHz")
DEFINE_MESSAGE(RHAL_UNSUPPORTED_EXT_LO_FORCE_0, "Allowed ext_lo_enable_flags are [0,1] forcing ext_lo_enable_flags[%d] to 0")
DEFINE_MESSAGE(RHAL_UNSUPPORTED_TX_GAIN_FORCE_1, "allowed tx_gain values are [1 to 7]. Forcing tx_gain to 1.")
DEFINE_MESSAGE(RHAL_UNSUPPORTED_TX_POW_MODE_FORCE, "allowed tx power mode values are [1,2,3]. Forcing to %d")
DEFINE_MESSAGE(RHAL_SCAN_SI_ALLOC_FAIL, "Unable to allocate buffers for scan instance")
DEFINE_MESSAGE(RHAL_PROC_ALLOC_FAIL, "Unable to allocate buffers for proc instance")
DEFINE_MESSAGE(RHAL_ADC_LANE_ERROR, "RX%d:DC level in ADC lanes is too large, high DC at requested gain values")
DEFINE_MESSAGE(RHAL_VGA4_I_DC_ERROR, "RX%d: VGA I DC offset(%d) is at the edge and DC is not corrected completely")
DEFINE_MESSAGE(RHAL_VGA4_Q_DC_ERROR, "RX%d: VGA Q DC offset(%d) is at the edge and DC is not corrected completely")
DEFINE_MESSAGE(RHAL_VGA1_I_DC_ERROR, "RX%d: VGA1 I DC offset(%d) is at the edge and DC is not corrected completely. Probably not correctable in minical as well")
DEFINE_MESSAGE(RHAL_VGA1_Q_DC_ERROR, "RX%d: VGA1 Q DC offset(%d) is at the edge and DC is not corrected completely. Probably not correctable in minical as well")
DEFINE_MESSAGE(RHAL_VGA3_I_DC_ERROR, "RX%d: VGA3 I DC offset(%d) is at the edge and DC is not corrected completely. Probably not correctable in minical as well")
DEFINE_MESSAGE(RHAL_VGA3_Q_DC_ERROR, "RX%d: VGA3 Q DC offset(%d) is at the edge and DC is not corrected completely. Probably not correctable in minical as well")
DEFINE_MESSAGE(RHAL_BQF_I_DC_ERROR, "RX%d: BQF I DC offset(%d) is at the edge and DC is not corrected completely.")
DEFINE_MESSAGE(RHAL_BQF_Q_DC_ERROR, "RX%d: BQF Q DC offset(%d) is at the edge and DC is not corrected completely.")
DEFINE_MESSAGE(RHAL_ADC_I_DC_ERROR, "RX%d: I DC in ADC Lane %d cannot be corrected by Color Correction Logic.")
DEFINE_MESSAGE(RHAL_ADC_Q_DC_ERROR, "RX%d: Q DC in ADC Lane %d cannot be corrected by Color Correction Logic.")
DEFINE_MESSAGE(RHAL_FINAL_DC_ERROR, "RX%d: DC correction is not gain independent.")
DEFINE_MESSAGE(RHAL_NON_STD_SCAN, "Running a Non Standard scan")
DEFINE_MESSAGE(RHAL_INVALID_VP_SCAN, "Invalid Variable-Power mode scan")
DEFINE_MESSAGE(RHAL_LDO_CAL_CONVERGE_ERROR, "LDO cal: failed to converge for regulator %d in module %d.")
DEFINE_MESSAGE(RHAL_LDO_CAL_REGULATOR_OFF, "LDO cal: the regulator %d in module %d seems to be off, aborting calibration for this regulator")
DEFINE_MESSAGE(RHAL_ACU_RX_BROKEN_REG_ACCESS, "Analog registers could be corrupted. Reset is recommended")
DEFINE_MESSAGE(RHAL_TCS_PASSED_PROD, "TxCarrierSuppression::PASSED")
DEFINE_MESSAGE(RHAL_VRX_ALIGN_CALIB_FILE_ERROR, "Unable to open vrx align calibration file")
DEFINE_MESSAGE(RHAL_RDC_POINTERS_ERROR, "RDC pointer seems to be not in intended region")
DEFINE_MESSAGE(RHAL_HOOKS_NOT_ENABLED, "RHAL Hooks not enabled in this build")
DEFINE_MESSAGE(RHAL_TCS_CAL_VAL_OUT_OF_RANGE_ERROR, "Tx Carrier Suppression cal values are out of range")
DEFINE_MESSAGE(RHAL_PGU_SWCODES_HOOK_ERROR, "Provided custom codes length (%d) is not matching with SD's Lc (%d)")
DEFINE_MESSAGE(RHAL_SCU_RS_INCOMP, "SCU scan didn't complete, but DCU last RDC1 memory got updated from SCU and CIU Scan completed!!")

DEFINE_MESSAGE(DIAG_INIT, "Diag Manager Initialized")
DEFINE_MESSAGE(DIAG_INVALID_TRIGGER, "Invalid diagnostic message received; class %d test %d")
DEFINE_MESSAGE(DIAG_TRIGGER, "Diagnostic test %d started")
DEFINE_MESSAGE(DIAG_FINISHED, "Diagnostic test %d completed")
DEFINE_MESSAGE(DIAG_INVALID_TESTID, "The test ID %d is out of range for this diag class")
DEFINE_MESSAGE(DIAG_PRECONDITION_FAILURE, "Diagnostic not started, a precondition %d is unsatisfied")
DEFINE_MESSAGE(DIAG_STATE_FAILURE, "Diagnostic not started, system is in the incorrect state %d")
DEFINE_MESSAGE(DIAG_UHDP_UNKNOWN_TYPE, "Diag manager received unexpected UHDP type %d")
DEFINE_MESSAGE(DIAG_INPUT_TOO_SMALL, "Diag given %d bytes of input, expected at least %d")
DEFINE_MESSAGE(DIAG_OUTPUT_TOO_SMALL, "Diag given %d bytes of output buffer, needs at least %d")

DEFINE_MESSAGE(UART_INIT, "UART Driver initialized")
DEFINE_MESSAGE(SSPI_INIT, "SSPI Driver initialized")
DEFINE_MESSAGE(SSPI_SHUTDOWN, "SSPI Driver shutdown")
DEFINE_MESSAGE(IOBUF_COUNT_WRONG, "message has invalid buffer count (%d != %d), leaking iovecs")
DEFINE_MESSAGE(TX_QUEUE_FAIL, "transmit queue tail is not NULL! leaking iovecs")
DEFINE_MESSAGE(IOVEC_NOT_ALLOC, "buffer transmitted in an invalid state")
DEFINE_MESSAGE(OS_FATAL_ERROR, "Fatal error occurred, entering exception loop")
DEFINE_MESSAGE(SMAN_MSG_PAUSE_POST_FAIL, "Failed to register pause event, insufficient room. Max is %d")
DEFINE_MESSAGE(SMAN_MSG_PAUSE_RELEASE_FAIL, "Failed to unregister pause event, event ID was %d")
DEFINE_MESSAGE(SMAN_MSG_PAUSE_STARTED, "Scan pause started")
DEFINE_MESSAGE(IOVEC_LOADED_ALLOC_FAIL, "Unable to allocate a loaded iovec %d times in the last second")
DEFINE_MESSAGE(IOVEC_BARE_ALLOC_FAIL, "Unable to allocate a bare iovec")
DEFINE_MESSAGE(MALLOC_FAIL, "Unable to allocate memory of size %d from memory type %d")
DEFINE_MESSAGE(HEAP_ALLOC_FAIL, "Unable to allocate stack heap of size %d from memory type %d")
DEFINE_MESSAGE(HEAP_ALLOC_INSTANCE_FAIL, "Unable to allocate alloc instances for heap %d, count %d")
DEFINE_MESSAGE(RESERVE_FAIL_STACK, "Unable to reserve lower memory, lower stack allocated. heap %d, rsv_size %d")
DEFINE_MESSAGE(RESERVE_FAIL_ALLOC, "Unable to reserve lower memory, insufficient room. heap %d, rsv_size %d")
DEFINE_MESSAGE(LOWER_ALLOC_FAIL_RESERVED, "Unable to alloc lower stack, lower section is reserved. heap %d, size %d")
DEFINE_MESSAGE(LOWER_ALLOC_FAIL_INSTANCE, "Unable to alloc lower stack, no free alloc instances. heap %d, size %d")
DEFINE_MESSAGE(LOWER_ALLOC_FAIL_INSUFFICIENT, "Unable to alloc lower stack, insufficient free bytes. heap %d, size %d")
DEFINE_MESSAGE(UPPER_ALLOC_FAIL_INSTANCE, "Unable to alloc upper stack, no free alloc instances. heap %d, size %d")
DEFINE_MESSAGE(UPPER_ALLOC_FAIL_INSUFFICIENT, "Unable to alloc upper stack, insufficient free bytes. heap %d, size %d")
DEFINE_MESSAGE(STACK_RELEASE_FAILURE, "Unable to release stack allocation. it is not a last alloc. heap %d")
DEFINE_MESSAGE(RESIZE_FAIL_INSUFFICIENT, "Unable to resize last alloc, insufficient free bytes. heap %d, newsize %d")
DEFINE_MESSAGE(RESIZE_FAIL_NOT_LAST, "Unable to resize alloc, not last allocated. heap %d, newsize %d")
DEFINE_MESSAGE(TFTP_INITIALIZED, "TFTP daemon initialized")
DEFINE_MESSAGE(TFTP_PACKET_TOO_SHORT, "TFTP packet is too short, discarding")
DEFINE_MESSAGE(TFTP_PACKET_UNKNOWN_OPCODE, "TFTP packet has unknown opcode %d, discarding")
DEFINE_MESSAGE(TFTP_ERROR_RECEIVED, "TFTP daemon received error code %d")
DEFINE_MESSAGE(TFTP_TIMEOUT_TRANSACTION, "TFTP daemon timed out last transaction")
DEFINE_MESSAGE(TFTP_START_FILE_READ, "TFTP daemon started file read transaction from %X:%d")
DEFINE_MESSAGE(TFTP_END_FILE_READ, "TFTP daemon ended file read transaction")
DEFINE_MESSAGE(TFTP_START_FILE_WRITE, "TFTP daemon started file write transaction from %X:%d")
DEFINE_MESSAGE(TFTP_END_FILE_WRITE, "TFTP daemon ended file write transaction")

DEFINE_MESSAGE(RHAL_MISSING_CAL_DATA_ERROR, "No calibration data found in flash file for this chip ID")
DEFINE_MESSAGE(RHAL_MISSING_LOT_MAPPING_ERROR, "No lot mapping found in flash for this chip ID")
DEFINE_MESSAGE(RHAL_INVALID_FLASH_FILE_ERROR, "Invalid 'misc_cal' flash file")
DEFINE_MESSAGE(RHAL_MISSING_FLASH_FILE_ERROR, "Missing 'misc_cal' flash file")

DEFINE_MESSAGE(RHAL_ANALOG_INIT_ERROR, "Unknown ADB/PIB version")

DEFINE_MESSAGE(RHAL_TX_CARRIER_SUPPRESSION_ERROR, "Tx Carrier Suppression Failed")
DEFINE_MESSAGE(RHAL_TX_CARRIER_SUPPRESSION_NOISE_ERROR, "No DC pattern for all trim values in any Rx")
DEFINE_MESSAGE(RHAL_TX_NUM_CHANNELS_ERROR, "Number of channels (%d) selected for applying TCS codes is less than 8")

DEFINE_MESSAGE(RHAL_RX_ANTENNA_SEL_ERROR, "Need to select at least one Rx from each Bank")
DEFINE_MESSAGE(RHAL_RX_ANTENNA_DUPLICATE_ERROR, "Cannot select Same Rx from both banks")

DEFINE_MESSAGE(SABINE_BOOTED_PROD, "*********************BOOTED FROM FLASH*********************")
DEFINE_MESSAGE(SRS_VERSION_PROD, "SRS version # %08x%08x")
DEFINE_MESSAGE(CAL_KEYS_MISSING, "%d required calibration keys are missing")
DEFINE_MESSAGE(CAL_KEYS_PRESENT, "all required calibration keys are present")

DEFINE_MESSAGE(ANGLE_CAL_WARN_DIAG,      "Large magnitude correction(s) in 'C' matrix diagonal (>12dB):  %d.%02d dB")
DEFINE_MESSAGE(ANGLE_CAL_WARN_COUPLING,  "Large cross-coupling in 'C' matrix off-diagonal (>-10dB):  %d.%02d dB")

DEFINE_MESSAGE(ADC_LDO_BOUND, "ADC LDO value %d is out of safe range (32,51)")

DEFINE_MESSAGE(ERR_NOT_SUPPORTED, ">>>>>>>>>>>>>>>> NOT SUPPORTED <<<<<<<<<<<<<<<< (%d, %d)")

/* these messages are registered by env-uhnder, but we keep them here to make the deployed environment file empty */
DEFINE_MESSAGE(FEAT_INIT,    "Feature Layer initialized")
DEFINE_MESSAGE(FEAT_SW_STOP, "stopping main loop")

DEFINE_MESSAGE(OBJ_INIT, "Object Layer initialized")
DEFINE_MESSAGE(OBJ_SCANNING_STARTED, "Scanning started")
DEFINE_MESSAGE(OBJ_SCANNING_IDLE, "Scanning idle, it is safe to start new scans")

DEFINE_MESSAGE(OS_BOOT, "Main loop entered")
DEFINE_MESSAGE(OS_QUIT, "Main loop exited")

DEFINE_MESSAGE(ARP_STORE_MAC, "Saving Ethernet address for IP %08X into ARP cache")
DEFINE_MESSAGE(ARP_LOOKUP_FAIL, "No Ethernet address for IP %08X found in ARP cache")
DEFINE_MESSAGE(ARP_PACKET_TOO_SMALL, "Ethernet frame of ethertype ARP too small for ARP header, len %d")
DEFINE_MESSAGE(ARP_PACKET_WRONG_HARDWARE_TYPE, "Unexpected ARP hardware type 0x%02X, execting 1")
DEFINE_MESSAGE(ARP_PACKET_WRONG_PROTOCOL_TYPE, "Unexpected ARP hardware type 0x%02X, execting 0x0800")
DEFINE_MESSAGE(ARP_PACKET_WRONG_ADDRESS_LENGTH, "Ignoring ARP packet with HW addr len %d, prot addr len %d")
DEFINE_MESSAGE(ARP_PACKET_WRONG_OPERATION_CODE, "Ignoring ARP packet with operation code %d")
DEFINE_MESSAGE(ARP_PACKET_OTHER_IP_ADDR, "Ignoring ARP packet for other IP address %08X")

DEFINE_MESSAGE(IP_PACKET_WRONG_DEST_IP, "IP packet received with destination address %08X, host IP is %08X. dropped")
DEFINE_MESSAGE(IP_PACKET_TOO_SMALL, "Ethernet frame of ethertype IP too small for IPV4 header, len %d")
DEFINE_MESSAGE(UNSUPPORTED_IP_PROTOCOL, "IPv4 handler received (non-UDP) packet with protocol %d")

DEFINE_MESSAGE(UDP_TOO_MANY_SOCKETS, "UDP listen port registration failed, MAX_SOCKETS=%d")
DEFINE_MESSAGE(UDP_NON_LISTENING, "Received UDP packet on non-listening port %d")
DEFINE_MESSAGE(UDP_INCOMPLETE_HEADER, "Received UDP packet has incomplete header length %d")
DEFINE_MESSAGE(UDP_INSUFFICIENT_PADDING, "Not enough room before payload to insert UDP headers")

DEFINE_MESSAGE(HSM_ENVIRONMENT_INIT, "HSM Environment processes initialized")

DEFINE_MESSAGE(ANTENNA_CONFIG_UNKNOWN, "Unknown 'Antenna Module Name'.  Set using 'configure_network.py'")
DEFINE_MESSAGE(ANTENNA_CONFIG_UNSET, "'Antenna Module Name' not set.  Set using 'configure_network.py'")

DEFINE_MESSAGE(NEW_LOG_LEVEL_PEDANTIC, "new log level: pedantic")
DEFINE_MESSAGE(NEW_LOG_LEVEL_DEBUG,    "new log level: debug")
DEFINE_MESSAGE(NEW_LOG_LEVEL_VERBOSE,  "new log level: verbose")
DEFINE_MESSAGE(NEW_LOG_LEVEL_INFO,     "new log level: info")
DEFINE_MESSAGE(NEW_LOG_LEVEL_WARN,     "new log level: warn")
DEFINE_MESSAGE(NEW_LOG_LEVEL_ERROR,    "new log level: error")
DEFINE_MESSAGE(NEW_LOG_LEVEL_ALWAYS,   "new log level: always")

DEFINE_MESSAGE(DC_CAL_ENTRY_NO_MATCH,  "No match found for DC calibration key")
DEFINE_MESSAGE(TCS_CAL_ENTRY_NO_MATCH, "No match found for TCS calibration key")
DEFINE_MESSAGE(VRX_CAL_ENTRY_NO_MATCH, "No match found for VRX calibration key")
DEFINE_MESSAGE(IQ_CAL_ENTRY_NO_MATCH,  "No match found for IQ calibration key")
DEFINE_MESSAGE(TEMP_CAL_ENTRY_NO_MATCH, "No match found for TEMP calibration key")
DEFINE_MESSAGE(ANGLE_CAL_ENTRY_NO_MATCH, "No match found for ANGLE (C) calibration key")
DEFINE_MESSAGE(DIAGONAL_CAL_ENTRY_NO_MATCH, "No match found for ANGLE (diagonal) calibration key")
DEFINE_MESSAGE(STEERING_ENTRY_NO_MATCH, "No match found for Steering Angle/Vector key")
DEFINE_MESSAGE(QILO_CAL_ENTRY_NO_MATCH, "No match found for QILO dcap offset key")
DEFINE_MESSAGE(LDO_CAL_ENTRY_NO_MATCH, "No match found for LDO calibration key")
DEFINE_MESSAGE(VTR_CAL_ENTRY_NO_MATCH, "No match found for VTR calibration key")

DEFINE_MESSAGE(POWER_OVERTEMP_LOG, "OVERHEAT EVENT OCCURED (1V8 AND 3V3 RAILS TURNED OFF) %d reboots ago")

DEFINE_MESSAGE(CIU_IFFT_0, "CIU IFFT_0 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_IFFT_1, "CIU IFFT_1 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_IFFT_2, "CIU IFFT_2 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_IFFT_3, "CIU IFFT_3 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_IFFT_4, "CIU IFFT_4 Mismatch between Overflow and RLSB (%d, %d)")

DEFINE_MESSAGE(CIU_FFT_0, "CIU FFT_0 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_FFT_1, "CIU FFT_1 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_FFT_2, "CIU FFT_2 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_FFT_3, "CIU FFT_3 Mismatch between Overflow and RLSB (%d, %d)")
DEFINE_MESSAGE(CIU_FFT_4, "CIU FFT_4 Mismatch between Overflow and RLSB (%d, %d)")

DEFINE_MESSAGE(CIU_DMUL_ERR, "CIU DMUL Mismatch between Overflow and RLSB (%d, %d)")
