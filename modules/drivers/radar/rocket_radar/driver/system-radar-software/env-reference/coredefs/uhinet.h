// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#ifndef SRS_HDR_UHINET_H
#define SRS_HDR_UHINET_H 1

#if SRS_PLATFORM_SOC

// rely on the fact these operations are equivalent
#define uh_htonl(l)  uh_ntohl(l)
#define uh_htons(l)  uh_ntohs(l)

SRS_DECLARE_NAMESPACE()
uint32_t uh_ntohl(uint32_t netlong);
uint16_t uh_ntohs(uint16_t netshort);
SRS_CLOSE_NAMESPACE()

#else

#include <errno.h>

#if SRS_PLATFORM_POSIX
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/socket.h>
#include <poll.h>
#include <signal.h>
#elif SRS_PLATFORM_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>   // for socketlen_t
#endif

#define uh_htons(s)  htons(s)
#define uh_ntohs(s)  ntohs(s)
#define uh_htonl(l)  htonl(l)
#define uh_ntohl(l)  ntohl(l)

#endif

#endif // if SRS_HDR_UHINET_H
