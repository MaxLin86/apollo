#ifndef SRS_HDR_DIAGCLASS_H
#define SRS_HDR_DIAGCLASS_H 1
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

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/diag-manager.h"

SRS_DECLARE_NAMESPACE()

/* base abstract class for all Diagnostic Class */
struct DiagClass
{
    virtual void        init() = 0;

    virtual void        start_diag(struct DiagDesc& desc) = 0;

    virtual uint32_t    get_num_tests() const = 0;

    virtual const CHAR* get_class_name() const = 0;

    virtual const CHAR* get_test_name(uint32_t test_id) const = 0;

    virtual uint8_t     get_test_api_version(uint32_t test_id) const = 0;

    virtual uint32_t    get_positive_test_attributes(uint32_t test_id) const = 0;

    virtual uint32_t    get_negative_test_attributes(uint32_t test_id) const = 0;

    void                register_self();

    uint32_t            get_class_id() const { return class_id; }

private:

    uint32_t class_id;
};

/*! helper structure for managing the details of the tests in each class, the
 * class can declare a static array of these structs and instantiate them with
 * static initializers. This is less error-prone. */
struct DiagTestDesc
{
    const CHAR* name;
    uint32_t    test_api_version; // could be 8bit
    uint32_t    positive_attr;
    uint32_t    negative_attr;
};

/* Attribute flags that can be applied to each diagnostic test. The flags might
 * be positive attributes, meaning the diag can only run if the particular
 * attribute flag is asserted in the run_state, or it can be a negative
 * attribute, meaning the diag cannot run if the attribute flag is asserted in
 * the run_state */
enum DiagAttributes
{
    DATTR_DESTRUCTIVE = (1 << 15), /*! diag test will leave system in an unstable state */
};

enum { DIAG_NAME_MAX = 128 };

enum { DIAG_TEST_API_VERSION_MAX = 255 };

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DIAGCLASS_H
