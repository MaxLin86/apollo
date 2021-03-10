// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DIAG_MANAGER_H
#define SRS_HDR_DIAG_MANAGER_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/diag-desc.h"

SRS_DECLARE_NAMESPACE()

class DiagManager
{
public:

    UHVIRTDESTRUCT(DiagManager)
    static DiagManager& instance();

    virtual void        init() = 0;

    virtual uint32_t    get_num_present_classes() const = 0;

    virtual uint32_t    get_num_tests_in_class(uint32_t c) const = 0;

    virtual const CHAR* get_class_name(uint32_t c) const = 0;

    virtual const CHAR* get_test_name(uint32_t c, uint32_t test_id) const = 0;

    virtual uint8_t     get_test_api_version(uint32_t c, uint32_t test_id) const = 0;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DIAG_MANAGER_H
