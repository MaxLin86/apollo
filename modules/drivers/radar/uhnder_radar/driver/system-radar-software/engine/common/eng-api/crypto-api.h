#ifndef SRS_HDR_CRYPTO_API_H
#define SRS_HDR_CRYPTO_API_H 1
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"

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
    AES_FAILURE = -1,
    AES_BUSY     = -2,
};

enum AESKeyLen
{
   AES_KEY_LEN_128 = 16,
   AES_KEY_LEN_256 = 32
};

enum AESKeyId
{
    AES_KEY_ID_DEVICE_KEY_0 = 0x40,
    AES_KEY_ID_DEVICE_KEY_1 = 0x80,
    AES_KEY_ID_SW_KEY = 0x00
};

struct AESSetting_t
{
    enum AESMode mode;
    enum AESOperation op;
    enum AESKeyLen key_len;
    enum AESKeyId key_id;
    uint8_t iv[16];
    uint8_t sw_key[32];
};


struct RemoteAesRPCRequest
{
    
    enum ICCQTargetEnum     cpu;
    AESSetting_t            *aes_setting;
    uint32_t                  *src;
    uint32_t                  *dst;
    uint32_t                  size;
    enum EventEnum          completion_event;
    enum AESCryptoErr     status;
};

SRS_CLOSE_NAMESPACE()


#endif
