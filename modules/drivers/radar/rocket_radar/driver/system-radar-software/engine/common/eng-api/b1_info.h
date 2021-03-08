#ifndef SRS_HDR_B1_INFO_H
#define SRS_HDR_B1_INFO_H 1
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/uhnder_part.h"

SRS_DECLARE_NAMESPACE()

//! RPC_B2_START_AUTHENTICATION
// RPC call to SCP from HSM to start authenticating new uhi image
// each image might contain multiple CPU uhim chunks
struct RcpB2AuthenticationData
{
    uint32_t img_length;
    uint32_t signature_length;
    uint32_t key_length;
    uint32_t *authenticated_img_addr;
    uint32_t key[64];
    uint32_t signature[64];
};

struct RcpB2StartAuthentication
{
    int8_t *address;
};

struct RsaUpdateData
{
    uint32_t data[SECTOR_SIZE / 4];
};

struct RpcB2RsaUpdateData
{
    int8_t *address;
    uint32_t length;
};

extern struct RcpB2AuthenticationData *b2_authentication_rpc_data;

extern struct RsaUpdateData *b2_rsa_update_img_data;


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_B1_INFO_H
