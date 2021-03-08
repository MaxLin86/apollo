#ifndef SRS_HDR_IP_PROTOCOLS_H
#define SRS_HDR_IP_PROTOCOLS_H 1
// START_SOFTWARE_LICENSE_NOTICE
// -------------------------------------------------------------------------------------------------------------------
// Copyright (C) 2016-2017 Uhnder, Inc. All rights reserved.
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

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

// these structures may be replaced by O/S header includes and macros

struct MACHeader
{
    uint8_t  dest_mac[6];
    uint8_t  source_mac[6];
    uint16_t ether_type;
};

enum
{
    ETHER_TYPE_IP  = 0x0800,
    ETHER_TYPE_ARP = 0x0806,
};

PACK(
    struct IPv4Header
{
    uint8_t  version_ihl;
    uint8_t  dsp_ecn;
    uint16_t total_length;
    uint16_t identification;
    uint16_t fragment_offset_and_flags;
    uint8_t  ttl;
    uint8_t  protocol;
    uint16_t header_checksum;
    uint32_t source_ip;
    uint32_t dest_ip;
});

enum
{
    IP_PROT_ICMP     = 0x01,
    IP_PROT_TCP      = 0x06,
    IP_PROT_UDP      = 0x11
};

struct UDPHeader
{
    uint16_t source_port;
    uint16_t dest_port;
    uint16_t payload_length;
    uint16_t checksum;
};

struct ICMPHeader
{
    uint8_t  type;
    uint8_t  code;
    uint16_t checksum;
    uint32_t data;
};

enum { MAX_ETHERNET_PAYLOAD = 1500 };

enum { MIN_ETHERNET_PAYLOAD = 64 - sizeof(MACHeader) - 4 /* FCS */ };

enum { MAX_ETHERNET_FRAME = MAX_ETHERNET_PAYLOAD + sizeof(MACHeader) + 4 /* FCS */ };

enum { MAX_UDP_DATAGRAM = MAX_ETHERNET_PAYLOAD - sizeof(IPv4Header) - sizeof(UDPHeader) };

enum { UDP_HEADER_SIZE = sizeof(MACHeader) + sizeof(IPv4Header) + sizeof(UDPHeader) };

#endif // SRS_HDR_IP_PROTOCOLS_H
