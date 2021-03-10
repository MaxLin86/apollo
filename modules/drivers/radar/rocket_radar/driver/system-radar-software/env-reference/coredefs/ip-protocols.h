// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_IP_PROTOCOLS_H
#define SRS_HDR_IP_PROTOCOLS_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"

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
