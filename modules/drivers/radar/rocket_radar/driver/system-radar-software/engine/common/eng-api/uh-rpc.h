// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_RPC_H
#define SRS_HDR_RPC_H 1
/*! \file */

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/iccq.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/rpc-enums.h"

SRS_DECLARE_NAMESPACE()

struct RPCMessage
{
    enum { RPC_PAYLOAD_BYTES = 56 };

    uint16_t rpc_function_id; /* RPCFunctionEnum */
    uint8_t  rpc_caller_id;   /* active RPC request id */
    int8_t   rpc_return_status;
    int8_t   rpc_payload[RPC_PAYLOAD_BYTES];
};

struct RemoteProcedureResponseHander
{
    virtual void rpc_response(RPCFunctionEnum type, int8_t status, int8_t* rpc_payload) = 0;
};

class RPCClient : public IccqMessageHandler
{
public:

    UHVIRTDESTRUCT(RPCClient)
    static RPCClient& instance();

    void init();

    /* can return false if resource allocation or back-pressure prevents the
     * message from being queued */
    bool call_remote(const ICCQTargetEnum to_cpu, const RPCFunctionEnum type,
                     RemoteProcedureResponseHander& resp_handler,
                     const void* const rpc_payload, const size_t payload_size);

protected:

    virtual void iccq_receive(ICCQMessage&);

    struct ResponseData
    {
        struct ResponseData*           free_next;

        RemoteProcedureResponseHander* handler;
        ICCQTargetEnum                 to_cpu;
        RPCFunctionEnum                type;
        uint8_t                        resp_id;
        /* copy of input payload? */
    };

    enum { MAX_MESSAGE_IDS = 32 };

    ResponseData  resp_table[MAX_MESSAGE_IDS];
    ResponseData* free_list;

#if !SRS_BOOT_IMAGE && __SCP__
    uint16_t    p5_in_flight_rpc[2];
#endif
};

struct RemoteProcedureRequestHandler
{
    virtual void rpc_request(RPCMessage& msg) = 0;
};

class RPCServer : public IccqMessageHandler
{
public:

    UHVIRTDESTRUCT(RPCServer)
    static RPCServer& instance();

    void init();

    void register_handler(RemoteProcedureRequestHandler& cb, RPCFunctionEnum e)
    {
        rpc_table[e] = &cb;
    }


protected:

    virtual void iccq_receive(ICCQMessage&);

    void process(RPCMessage& msg);

    RemoteProcedureRequestHandler* rpc_table[NUM_RPC_FUNCTIONS];
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_RPC_H
