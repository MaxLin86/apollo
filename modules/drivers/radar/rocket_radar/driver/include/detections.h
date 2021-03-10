// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()
struct DetectionData;
SRS_CLOSE_NAMESPACE()

class Detections
{
public:

    virtual ~Detections() {}

    //! returns the number of detections reported in this scan
    virtual uint32_t             get_count() const = 0;

    //! returns a specific detection instance
    virtual const DetectionData& get_detection(uint32_t idx) const = 0;

    //! release all of the memory associated with this Detections instance
    virtual void                 release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        DET_NO_ERROR,
        DET_INDEX_OUT_OF_RANGE,
    };

    //! returns the last error encountered by this instance
    virtual Err                  get_last_error() const = 0;
};
