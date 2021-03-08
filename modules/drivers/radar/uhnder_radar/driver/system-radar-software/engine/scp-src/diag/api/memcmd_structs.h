#ifndef SRS_HDR_MEMCMD_STRUCTS_H
#define SRS_HDR_MEMCMD_STRUCTS_H 1

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! command types for memcmds test
enum DiagIndirectMemCommands
{
    MEMCMD_WriteBytes,
    MEMCMD_WriteWord,
    MEMCMD_ReadModifyWrite,
    MEMCMD_SetCompletionDelay,
    MEMCMD_Poll,
    MEMCMD_SpinWait,
};

//! input structure for test "indirect::memcmds"
struct MemCommandWriteBytes
{
    //! must be enum writebytes (0)
    uint32_t command_type;

    //! memcpy(write_ptr, data, write_size_bytes);
    uint32_t write_ptr;

    //! memcpy(write_ptr, data, write_size_bytes);
    //!
    //! must be followed by write_size_bytes of data to be written
    uint32_t write_size_bytes;

    MemCommandWriteBytes() : command_type(MEMCMD_WriteBytes) {}
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

//! input structure for test "indirect::memcmds"
struct MemCommandSetCompletionDelay
{
    //! must be enum setCompletionDelay (3)
    uint32_t command_type;

    //! amount of time to wait (after all commands have completed) before
    //! signaling the completion of the memcmds diag test.
    uint32_t microseconds;

    MemCommandSetCompletionDelay() : command_type(MEMCMD_SetCompletionDelay) {}
};

//! input structure for test "indirect::memcmds"
struct MemCommandPoll
{
    //! must be enum poll (4)
    uint32_t command_type;

    //! while ((*poll_ptr & poll_and_mask) != poll_match_value) ;
    uint32_t poll_ptr;

    //! while ((*poll_ptr & poll_and_mask) != poll_match_value) ;
    uint32_t poll_and_mask;

    //! while ((*poll_ptr & poll_and_mask) != poll_match_value) ;
    uint32_t poll_match_value;

    MemCommandPoll() : command_type(MEMCMD_Poll) {}
};

//! input structure for test "indirect::memcmds"
struct MemCommandSpinWait
{
    //! must be enum spinWait (5)
    uint32_t command_type;

    //! block the CPU for this many nanoseconds, before moving on to next
    //! command. Must be less than 10 microseconds.
    uint32_t nanosecond_delay;

    MemCommandSpinWait() : command_type(MEMCMD_SpinWait) {}
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_MEMCMD_STRUCTS_H
