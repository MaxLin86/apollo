#ifndef SRS_HDR_APROBESINGLE_H
#define SRS_HDR_APROBESINGLE_H 1

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! Test input structure for "aprobesingle::getSingleAprobeValue"
struct aprobeSingle_getSingleAprobeValue_input
{
    //! Specify which MADC to probe: 0 = TX, 1 = SH, 2 = RX
    uint32_t whichMadc;
    //! Specify which channel (0 to 11 for TX, 1 for SH, 0 to 7 for RX)
    uint32_t channel;
    //! Spcify which point to probe
    uint32_t probe;
    //! Specify the type of measurement to make.  Typically use -1 to choose the default
    int32_t measurement;
    //! Choose the number of accumulations to make.  (16384 works well)
    uint32_t numAcc;
    //! Sticky bit - if non zero the probe input to the MADC remains until the next probe is set
    //!   otherwise the probe input reverts to default "off" state.
    uint32_t sticky;
};

//! Test output structure for "aprobesingle::getSingleAprobeValue"
struct aprobeSingle_getSingleAprobeValue_output
{
    //! The output value of the aprobe normalized (typically a voltage)
    FLOAT output;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_APROBESINGLE_H
