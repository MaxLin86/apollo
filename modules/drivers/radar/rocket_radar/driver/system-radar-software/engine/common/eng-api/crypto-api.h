// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_CRYPTO_API_H
#define SRS_HDR_CRYPTO_API_H 1


#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
SRS_DECLARE_NAMESPACE()
enum AESMode
{
    AES_MODE_ECB = 1,
    AES_MODE_CBC = 2,
    AES_MODE_CTR = 3,
    AES_MODE_CFB = 4,
    AES_MODE_OFB = 5,
    AES_MODE_XTS = 6,
    AES_MODE_GCM = 7
};

enum AESOperation
{
    AES_ENCRIPTION = 1,
    AES_DECRYPTION = 2,
    AES_BYPASS
};

enum AESCryptoErr
{
    AES_SUCCESS  = 0,
    AES_FAILURE  = -1,
    AES_BUSY     = -2,
};

enum AESKeyLen
{
   AES_KEY_LEN_128 = 16,
   AES_KEY_LEN_256 = 32
};

enum AESKeyId
{
    AES_KEY_ID_DEVICE_KEY_0 = 0x40U,
    AES_KEY_ID_DEVICE_KEY_1 = 0x80U,
    AES_KEY_ID_SW_KEY       = 0x00U
};

struct AESSetting_t
{
    enum AESMode      mode;
    enum AESOperation op;
    enum AESKeyLen    key_len;
    enum AESKeyId     key_id;
    uint8_t           iv[16];
    uint8_t           sw_key[32];
};

struct RemoteAesRPCRequest
{
    enum ICCQTargetEnum     cpu;
    AESSetting_t            *aes_setting;
    uint32_t                *src;
    uint32_t                *dst;
    uint32_t                size;
    enum EventEnum          completion_event;
    enum AESCryptoErr       status;
};

SRS_CLOSE_NAMESPACE()
#endif
