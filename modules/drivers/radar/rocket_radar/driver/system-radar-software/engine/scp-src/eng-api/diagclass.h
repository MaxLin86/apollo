// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_DIAGCLASS_H
#define SRS_HDR_DIAGCLASS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/scp-src/eng-api/diag-manager.h"

SRS_DECLARE_NAMESPACE()

/* base abstract class for all Diagnostic Class */
struct DiagClass
{
    UHVIRTDESTRUCT(DiagClass)
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
