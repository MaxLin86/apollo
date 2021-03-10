#pragma once

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/include/rra.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rdc-structs.h"
#include "modules/drivers/radar/quill_radar/driver/src/uhdp/prothandlerbase.h"

class Connection_Impl;

struct FastCaptureHandler : public ProtHandlerBase
{
public:

    FastCaptureHandler(Connection_Impl& con);
    // Cancel any active blob capture
    void         clear();

    // Check if a retransmission is necessary
    void         poll(uint64_t cur_timestamp);

protected:

    Connection_Impl&    my_con;
    uint32_t            scan_sequence_number;
    uint32_t            cur_blob_segment;
    uint32_t            num_blob_segments;
    uint32_t            extra_ack_sequence;
    uint64_t            rdc3_blob_last_received;
    RDC3BlobHeader      blob_header;

    uint32_t*           completion_bitmap;
    char*               blob_buffer;

    void         send_ack(bool done, uint32_t new_blob_segment);

    virtual void handle_packet(const char* payload, uint32_t len);
};

