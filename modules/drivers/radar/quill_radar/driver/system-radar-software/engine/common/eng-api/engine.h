#ifndef SRS_HDR_ENGINE_H
#define SRS_HDR_ENGINE_H 1
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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "event-handler.h"
#if __SCP__ && !SRS_BOOT_IMAGE
#include "modules/drivers/radar/quill_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#endif

SRS_DECLARE_NAMESPACE()

enum EngineLogSubunits
{
    ENG_EVENT_HANDLER,
    ENG_STATE_MANAGER,
    ENG_LOGGER,
    ENG_RPC,
    ENG_IOVEC,
    ENG_DEVMM,
    ENG_TFS,
    ENG_TFTP,
    NUM_ENG_SUBUNITS
};

class Engine
#if __SCP__ && !SRS_BOOT_IMAGE
    : public LogProducer
#endif
{
public:

    UHVIRTDESTRUCT(Engine)
    static Engine& instance();

    void init();

    void shutdown() { /* TBD */ }

    void mainloop_poll();

#if __SCP__ && !SRS_BOOT_IMAGE
    static const CHAR* subunit_names[NUM_ENG_SUBUNITS];

    virtual const CHAR* get_subunit_name(uint32_t subunit) const;

    virtual uint32_t get_num_subunits() const { return NUM_ENG_SUBUNITS; }

#if WITH_UHDP
    // if the radar receives an ICMP destination unreachable message, this
    // function should be called with the IP address and UDP port of the
    // returned message header (after the ICMP header) in network order These
    // should be the IP and port of the UDP packet that triggered the ICMP
    // response
    void icmp_destination_unreachable(uint32_t dest_ip, uint16_t dest_port);
#endif

#else
    uint32_t data; // this class needs at least one member to declare as a singleton
#endif
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_ENGINE_H
