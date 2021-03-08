#ifndef SRS_HDR_DIAG_DESC_H
#define SRS_HDR_DIAG_DESC_H 1
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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"

SRS_DECLARE_NAMESPACE()

#define HAVE_DIAG_DESC 1

enum DiagReturnCodeEnum
{
    DIAG_RET_SUCCESS             =  0,
    DIAG_RET_TEST_FAIL           = -1,
    DIAG_RET_INVALID_CLASS_ID    = -2,
    DIAG_RET_CLASS_NOT_PRESENT   = -3,
    DIAG_RET_INVALID_TEST_ID     = -4,
    DIAG_RET_INSUFFICIENT_INPUT  = -5,
    DIAG_RET_INSUFFICIENT_OUTPUT = -6,
    //DIAG_RET_UNSUP_OUTPUT_TYPE   = -7, obsoleted
    DIAG_RET_PRECONDITION_FAIL   = -8,
    DIAG_RET_RUN_STATE_FAIL      = -9,
    DIAG_RET_RPC_FAIL            = -10,
    DIAG_RET_ALLOC_FAIL          = -11,
    DIAG_RET_BAD_PARAMETERS      = -12,
};

/*! the descriptor which must be filled in by a diag agent and posted to
 * DIAG_TRIGGER_EVENT event. */
struct DiagDesc
{
    /*! opaque pointer for use by the diagnostic manager */
    struct DiagDesc* fifo_next;

    /*! This event will be posted when the diagnostic test is complete, the
     * descriptor pointer is returned as the event data pointer */
    EventEnum return_event;

    /*! Each diagnostic class defines its own test case ID enumerations, for
     * independent development */
    uint32_t diag_class;

    /*! The index of the specific diag test to be executed within the specified
     * diag_class */
    uint32_t test_id;

    /*! input data for the diag test, must be allocated by UHDP agent */
    int8_t*  input_data;

    /*! the exact size of the input data, not necessarily the size of the
     * allocated memory */
    uint32_t input_data_size_bytes;

    /*! success or failure indication. The diag tests may use return codes not
     * in the enumeration list of DiagReturnCodeEnum */
    DiagReturnCodeEnum return_code;

    /*! if return_code is DIAG_RET_TEST_FAIL, then this field contains an actual
     * error value */
    INT error_num;

    /*! output data for the diag test, must be allocated by UHDP agent */
    int8_t*  output_data;
    /*! size of allocation for output data */
    uint32_t output_buffer_size_bytes;

    /*! number of bytes written into the output buffer by the DIAG test */
    uint32_t output_data_size_bytes;

    /*! user defined opaque pointer for use by the caller of the diag */
    void *user_ptr;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DIAG_DESC_H
