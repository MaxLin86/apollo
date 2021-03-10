#ifndef SRS_HDR_RPC_H
#define SRS_HDR_RPC_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2019 Uhnder, Inc. All rights reserved.
// This Software is the property of Uhnder, Inc. (Uhnder) and is Proprietary and Confidential.  It has been provided
// under license for solely use in evaluating and/or developing code for Uhnder products.  Any use of the Software to
// develop code for a product not manufactured by or for Uhnder is prohibited.  Unauthorized use of this Software is
// strictly prohibited.
// Restricted Rights Legend:  Use, Duplication, or Disclosure by the Government is Subject to Restrictions as Set
// Forth in Paragraph (c)(1)(ii) of the Rights in Technical Data and Computer Software Clause at DFARS 252.227-7013.
// THIS PROGRAM IS PROVIDED UNDER THE TERMS OF THE UHNDER END-USER LICENSE AGREEMENT (EULA). THE PROGRAM MAY ONLY
// BE USED IN A MANNER EXPLICITLY SPECIFIED IN THE EULA, WHICH INCLUDES LIMITATIONS ON COPYING, MODIFYING,
// REDISTRIBUTION AND WARRANTIES. PROVIDING AFFIRMATIVE CLICK-THROUGH CONSENT TO THE EULA IS A REQUIRED PRECONDITION
// TO YOUR USE OF THE PROGRAM. YOU MAY OBTAIN A COPY OF THE EULA FROM WWW.UHNDER.COM. UNAUTHORIZED USE OF THIS
// PROGRAM IS STRICTLY PROHIBITED.
// THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES ARE GIVEN, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING
// WARRANTIES OR MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT AND TITLE.  RECIPIENT SHALL HAVE
// THE SOLE RESPONSIBILITY FOR THE ADEQUATE PROTECTION AND BACK-UP OF ITS DATA USED IN CONNECTION WITH THIS SOFTWARE.
// IN NO EVENT WILL UHNDER BE LIABLE FOR ANY CONSEQUENTIAL DAMAGES WHATSOEVER, INCLUDING LOSS OF DATA OR USE, LOST
// PROFITS OR ANY INCIDENTAL OR SPECIAL DAMAGES, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
// SOFTWARE, WHETHER IN ACTION OF CONTRACT OR TORT, INCLUDING NEGLIGENCE.  UHNDER FURTHER DISCLAIMS ANY LIABILITY
// WHATSOEVER FOR INFRINGEMENT OF ANY INTELLECTUAL PROPERTY RIGHTS OF ANY THIRD PARTY.
// -------------------------------------------------------------------------------------------------------------------
// END_SOFTWARE_LICENSE_NOTICE
/*! \file */

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/iccq-enums.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/iccq.h"
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/rpc-enums.h"

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

#if !SRS_BOOT_IMAGE && __SCP__ && SABINE_B
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
