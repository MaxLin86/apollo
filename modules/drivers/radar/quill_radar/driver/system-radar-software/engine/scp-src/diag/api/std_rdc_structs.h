#ifndef SRS_HDR_STD_RDC_STRUCTS_H
#define SRS_HDR_STD_RDC_STRUCTS_H 1

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! Diag Input structure for "std_rdc::adc_playback"
struct adc_playback_input
{
    //! ADC file size
    uint32_t adc_len;

    //! PRN file size
    uint32_t prn_len;

    //! ADC file name
    int8_t   adc_file_name[16];

    //! PRN file name
    int8_t   prn_file_name[18];

    //! Set to "1" if first pulse data only to be repeated for all pulses
    uint8_t  repeat_same_pulse;

    //! Set to "1" for a dummy scan to run before adc capture
    //! Set to "2" for a dummy scan to run after adc capture
    uint8_t  dummy_scan_type;
};

struct set_pfar_pid_constants_input
{
    FLOAT pfa_pid_kp;
    FLOAT pfa_pid_ki;
    FLOAT pfa_pid_kd;
    uint32_t num_activations_above_det_thresh;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_STD_RDC_STRUCTS_H
