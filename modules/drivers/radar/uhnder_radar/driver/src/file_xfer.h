#pragma once

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

enum TFTPError
{
    TFTP_ERR_NOT_DEFINED            =  0,
    TFTP_ERR_FILE_NOT_FOUND         =  1,
    TFTP_ERR_ACCESS_VIOLATION       =  2,
    TFTP_ERR_DISK_FULL              =  3,
    TFTP_ERR_ILLEGAL_OPERATION      =  4,
    TFTP_ERR_UNKNOWN_TID            =  5,
    TFTP_ERR_FILE_EXISTS            =  6,
    TFTP_ERR_NO_SUCH_USER           =  7
};

enum TFTPOpcode
{
    TFTP_OP_READ_REQ                =  1,
    TFTP_OP_WRITE_REQ               =  2,
    TFTP_OP_DATA                    =  3,
    TFTP_OP_ACK                     =  4,
    TFTP_OP_ERROR                   =  5
};

enum { MAX_PAYLOAD_BYTES = 512 };
