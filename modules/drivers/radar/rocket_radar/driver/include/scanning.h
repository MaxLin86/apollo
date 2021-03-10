// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

class ScanObject;

class Scanning
{
public:

    virtual ~Scanning() {}

    //! Blocking wait for a scan object, returns NULL if scanning is interrupted
    virtual ScanObject* wait_for_scan() = 0;

    //! returns true if wait_for_scan() would not block if called
    virtual bool        scan_available() = 0;

    //! release all of the memory held by the Scanning object and stop the radar
    //! scans
    virtual void        release() = 0;

    //! an enumeration of the errors which might be returned by this class
    enum Err
    {
        SCANNING_NO_ERROR,
        SCANNING_COMPLETED,
        SCANNING_CONNECTION_RELEASED,
        SCANNING_NOT_IN_MANUAL_MODE,
    };

    //! returns the last error encountered by this instance
    virtual Err         get_last_error() const = 0;

    //! A factory method returning a Scanning instance which can replay a session folder
    static Scanning*    replay_session(const char* session_path, uint32_t first_seq=0);
};
