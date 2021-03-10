#ifndef SRS_HDR_MEMCMD_STRUCTS_H
#define SRS_HDR_MEMCMD_STRUCTS_H 1

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! command types for memcmds test
enum DiagIndirectMemCommands
{
    MEMCMD_WriteBytes,
    MEMCMD_WriteWord,
    MEMCMD_ReadModifyWrite,
};

//! input structure for test "indirect::memcmds"
struct MemCommandWriteWord
{
    //! must be enum writeword (1)
    uint32_t command_type;

    //! *write_ptr = write_value;
    uint32_t write_ptr;

    //! *write_ptr = write_value;
    uint32_t write_value;

    MemCommandWriteWord() : command_type(MEMCMD_WriteWord) {}
};

//! input structure for test "indirect::memcmds"
struct MemCommandReadModifyWrite
{
    //! must be enum readModifyWrite (2)
    uint32_t command_type;

    //! *write_ptr = ((*write_ptr) & write_and_mask) | write_or_mask;
    uint32_t write_ptr;

    //! *write_ptr = ((*write_ptr) & write_and_mask) | write_or_mask;
    uint32_t write_and_mask;

    //! *write_ptr = ((*write_ptr) & write_and_mask) | write_or_mask;
    uint32_t write_or_mask;

    MemCommandReadModifyWrite() : command_type(MEMCMD_ReadModifyWrite) {}
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_MEMCMD_STRUCTS_H
