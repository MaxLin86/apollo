// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_SRS_VERSION_H
#define SRS_HDR_SRS_VERSION_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

const CHAR* get_version_string();

#if !SRS_BOOT_IMAGE
void report_version(void);
#endif

SRS_CLOSE_NAMESPACE()

#endif // ifndef SRS_HDR_SRS_VERSION_H
