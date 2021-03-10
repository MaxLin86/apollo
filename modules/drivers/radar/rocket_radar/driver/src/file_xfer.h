// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#pragma once

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

enum TFTPError
{
    TFTP_ERR_NOT_DEFINED            =  0,
    TFTP_ERR_FILE_NOT_FOUND         =  1,
    TFTP_ERR_ACCESS_VIOLATION       =  2,
    TFTP_ERR_DISK_FULL              =  3,
    TFTP_ERR_ILLEGAL_OPERATION      =  4,
    TFTP_ERR_UNKNOWN_TID            =  5,
    TFTP_ERR_FILE_EXISTS            =  6,
    TFTP_ERR_NO_SUCH_USER           =  7,
    TFTP_ERR_INVALID_OPTION         =  8
};

enum TFTPOpcode
{
    TFTP_OP_READ_REQ                =  1,
    TFTP_OP_WRITE_REQ               =  2,
    TFTP_OP_DATA                    =  3,
    TFTP_OP_ACK                     =  4,
    TFTP_OP_ERROR                   =  5,
    TFTP_OP_OPTIONS_ACK             =  6
};

enum {
    DEFAULT_BLOCK_SIZE = 512,
    MAX_NUM_BLOCKS = 64 * 1024,
    MAX_BLOCK_SIZE = 1464,
};
