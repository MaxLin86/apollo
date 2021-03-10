// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/scanning.h"

class Connection_Impl;
class SessionFolderScanSerializer;

class Scanning_Impl: public Scanning
{
public:

    Scanning_Impl(Connection_Impl* c);                                 // live
    Scanning_Impl(const char* session_path, uint32_t first_seq = 0);   // replay

    virtual ~Scanning_Impl();

    virtual ScanObject* wait_for_scan();

    virtual bool        scan_available();

    virtual void        release();

    virtual Err         get_last_error() const { return last_err; }

    Connection_Impl*    mycon;

    uint32_t            cur_sequence_number;

    Err                 last_err;

    SessionFolderScanSerializer* session_serializer;
};
