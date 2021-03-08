#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhinet.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhunistd.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/engine-defs.h"
#include "modules/drivers/radar/rocket_radar/driver/src/uhdp/prothandlerbase.h"
#include "modules/drivers/radar/rocket_radar/driver/src/connection_impl.h"
#include "modules/drivers/radar/rocket_radar/driver/src/file_xfer.h"

#if defined(_MSC_VER)
#include <BaseTsd.h>
typedef SSIZE_T ssize_t;
#endif

enum { MAX_RESENDS = 4 };

static uint16_t get_tftp_port()
{
    uint16_t tftp_port = TFTP_PORT;
    const char* portstr = getenv("SABINE_UHDP_PORT");
    if (portstr)
    {
        int envport = atoi(portstr);
        if (envport > 0 && envport < 0xFFFE)
        {
            tftp_port = (uint16_t)envport + 1;
        }
    }

    return tftp_port;
}


bool send_with_retry(int fd, sockaddr_in raddr, char* message, size_t& len, size_t sz, uint32_t& retry_count)
{
    struct sockaddr_in remaddr;
    socklen_t addrlen = sizeof(remaddr);

    uint32_t rs;
    for (rs = 0; rs < MAX_RESENDS; rs++)
    {
        sendto(fd, message, len, 0, (sockaddr*)&raddr, sizeof(raddr));
        ssize_t recvlen = recvfrom(fd, message, sz, 0, (sockaddr*)&remaddr, &addrlen);
        len = size_t(recvlen);
        uint16_t opcode = ntohs(*(uint16_t*)message);
        if (recvlen <= 0)
        {
            retry_count++;
            continue;
        }
        else if (opcode == TFTP_OP_ACK || opcode == TFTP_OP_OPTIONS_ACK)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
    return false;
}


bool Connection_Impl::tftp_upload(const char* local_fname, const char* radar_fname)
{
    char*  data;
    size_t sizebytes;

    FILE* fp = fopen(local_fname, "rb");
    if (fp)
    {
        fseek(fp, 0, SEEK_END);
        sizebytes = ftell(fp);
        rewind(fp);
    }
    else
    {
        printf("Unable to open local file %s\n", local_fname);
        last_err = CON_UNABLE_TO_OPEN_FILE;
        return false;
    }

    if (sizebytes > MAX_BLOCK_SIZE * MAX_NUM_BLOCKS)
    {
        printf("Local file %s too large for TFTP\n", local_fname);
        fclose(fp);
        last_err = CON_TFTP_INPUT_TOO_LARGE;
        return false;
    }

    data = new char[sizebytes];
    int count = fread(data, sizebytes, 1, fp);
    fclose(fp);
    if (!count)
    {
        printf("Local file %s read failed\n", local_fname);
        delete [] data;
        last_err = CON_UNABLE_TO_READ_FILE;
        return false;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        printf("Cannot create socket, err %d\n", fd);
        delete [] data;
        last_err = CON_SOCKET_FAILURE;
        return false;
    }

    struct sockaddr_in myaddr;
    memset(&myaddr, 0, sizeof(myaddr));

    uint16_t listen_port = 0;
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(listen_port);

    sockaddr_in raddr;
    memcpy(&raddr, &radar_ip4addr, sizeof(sockaddr_in));
    raddr.sin_port = htons(get_tftp_port());

#if _WIN32
    int timeout_ms = 2 * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    timeval tv;
    tv.tv_sec = 2 * timeout_scale;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
#endif

    char msgbuf[MAX_BLOCK_SIZE + 8];
    uint16_t* opcode = (uint16_t*)msgbuf;
    uint16_t* block_id_ptr = opcode + 1;
    char* payload = (char*)(opcode + 1);
    uint32_t remainder = 0;
    uint32_t block_size = DEFAULT_BLOCK_SIZE;
    size_t len;
    bool res = false;

    if (bind(fd, (struct sockaddr*)&myaddr, sizeof(myaddr)) < 0)
    {
        printf("Unable to bind local socket\n");
        last_err = CON_SOCKET_FAILURE;
        goto upfail;
    }

    // initiate TFTP session, return error if fails

    // try to upgrade block size if the file is large enough (RFC-2348)
    *opcode = htons(TFTP_OP_WRITE_REQ);
    len  = sprintf(payload, "%s", radar_fname) + 1;
    len += sprintf(payload + len, "%s", "octet") + 1;
    len += sprintf(payload + len, "%s", "blksize") + 1;
    len += sprintf(payload + len, "%d", MAX_BLOCK_SIZE) + 1;
    len += sizeof(*opcode);

    if ((sizebytes > size_t(DEFAULT_BLOCK_SIZE) * 10) &&
        send_with_retry(fd, raddr, msgbuf, len, sizeof(msgbuf), counters.tftp_retransmissions))
    {
        uint16_t *opcode_ptr = (uint16_t*)&msgbuf[0];
        uint16_t opcode = ntohs(*opcode_ptr);
        if (opcode == TFTP_OP_OPTIONS_ACK && len > 10)
        {
            // See what gifts the server has provided us with
            char* optstr = msgbuf + sizeof(opcode);
            if (!strcmp(optstr, "blksize"))
            {
                char* blksizestr = optstr + sizeof("blksize");
                block_size = atoi(blksizestr);
                if (block_size < 8 || block_size > MAX_BLOCK_SIZE)
                {
                    printf("Returned block size %d is invalid\n", block_size);
                    goto upfail;
                }
                printf("Server is allowing a block size of %d bytes, for a total of %zu\n", block_size,
                       size_t(block_size) * MAX_NUM_BLOCKS);
            }
        }
    }
    else
    {
        // fallback to default block size 512 (RFC-1350)
        *opcode = htons(TFTP_OP_WRITE_REQ);
        len  = sprintf(payload, "%s", radar_fname) + 1;
        len += sprintf(payload + len, "%s", "octet") + 1;
        len += sizeof(*opcode);
        if (!send_with_retry(fd, raddr, msgbuf, len, sizeof(msgbuf), counters.tftp_retransmissions))
        {
            last_err = CON_TFTP_OPEN_ERROR;
            goto upfail;
        }
        block_size = DEFAULT_BLOCK_SIZE;
    }

    // recheck file size with current size restriction
    if (sizebytes > block_size * MAX_NUM_BLOCKS)
    {
        printf("Local file %s too large for TFTP\n", local_fname);
        goto upfail;
    }

    payload = (char*)(block_id_ptr + 1);
    uint16_t block_id;
    for (block_id = 0; block_id < sizebytes / block_size; block_id++)
    {
        *opcode = htons(TFTP_OP_DATA);
        *block_id_ptr = htons(block_id + 1);
        memcpy(payload, data + block_id * block_size, block_size);
        len = block_size + sizeof(*opcode) * 2;
        if (!send_with_retry(fd, raddr, msgbuf, len, sizeof(msgbuf), counters.tftp_retransmissions))
        {
            last_err = CON_TFTP_SEND_ERROR;
            goto upfail;
        }
    }

    remainder = sizebytes - size_t(block_id) * block_size;
    *opcode = htons(TFTP_OP_DATA);
    *block_id_ptr = htons(block_id + 1);
    memcpy(payload, data + block_id * block_size, remainder);
    len = remainder + sizeof(*opcode) * 2;
    if (!send_with_retry(fd, raddr, msgbuf, len, sizeof(msgbuf), counters.tftp_retransmissions))
    {
        last_err = CON_TFTP_SEND_ERROR;
        goto upfail;
    }

    last_err = CON_NO_ERROR;
    res = true;

upfail:

    delete [] data;
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif

    return res;
}

bool Connection_Impl::tftp_download(const char* local_fname, const char* radar_fname)
{
    FILE* fp = fopen(local_fname, "wb");
    if (!fp)
    {
        printf("Unable to open '%s' for write\n", local_fname);
        last_err = CON_UNABLE_TO_OPEN_FILE;
        return false;
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0)
    {
        last_err = CON_SOCKET_FAILURE;
        printf("Cannot create socket, err %d\n", fd);
        fclose(fp);
        return false;
    }

    struct sockaddr_in myaddr;
    memset(&myaddr, 0, sizeof(myaddr));

    uint16_t listen_port = 0;
    myaddr.sin_family = AF_INET;
    myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    myaddr.sin_port = htons(listen_port);

    sockaddr_in raddr;
    memcpy(&raddr, &radar_ip4addr, sizeof(sockaddr_in));
    raddr.sin_port = htons(get_tftp_port());

#if _WIN32
    int timeout_ms = 4 * 1000;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    timeval tv;
    tv.tv_sec = 4 * timeout_scale;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(struct timeval));
#endif

    char msgbuf[DEFAULT_BLOCK_SIZE + 8];
    uint16_t* opcode = (uint16_t*)msgbuf;
    uint16_t* block_id_ptr = opcode + 1;
    char* payload = (char*)(opcode + 1);
    int len, recvlen;
    bool res = false;
    struct sockaddr_in remaddr;
    socklen_t addrlen = sizeof(remaddr);

    if (bind(fd, (struct sockaddr*)&myaddr, sizeof(myaddr)) < 0)
    {
        printf("Unable to bind local socket\n");
        last_err = CON_SOCKET_FAILURE;
        fclose(fp);
        close(fd);
        return false;
    }

    // initiate TFTP session, return error if fails
    *opcode = htons(TFTP_OP_READ_REQ);
    len  = sprintf(payload, "%s", radar_fname) + 1;
    len += sprintf(payload + len, "%s", "binary") + 1;
    len += sizeof(*opcode);

    sendto(fd, msgbuf, len, 0, (struct sockaddr*)&raddr, sizeof(raddr));

    payload = (char*)(block_id_ptr + 1);
    for (;;)
    {
        uint32_t rs;
        for (rs = 0; rs < MAX_RESENDS; rs++)
        {
            recvlen = recvfrom(fd, msgbuf, sizeof(msgbuf), 0, (struct sockaddr*)&remaddr, &addrlen);
            if (recvlen <= 0)
            {
                counters.tftp_retransmissions++;
                sendto(fd, msgbuf, len, 0, (struct sockaddr*)&raddr, sizeof(raddr));
                continue;
            }
            else if (*opcode == htons(TFTP_OP_DATA))
            {
                recvlen -= sizeof(*opcode) * 2;
                fwrite(payload, recvlen, 1, fp);
                break;
            }
            else
            {
                last_err = CON_TFTP_SEND_ERROR;
                goto downfail;
            }
        }
        if (rs >= MAX_RESENDS)
        {
            last_err = CON_TFTP_SEND_ERROR;
            break;
        }

        // change opcode to ACK, leave block ID
        *opcode = htons(TFTP_OP_ACK);
        len = sizeof(*opcode) * 2;
        sendto(fd, msgbuf, len, 0, (struct sockaddr*)&raddr, sizeof(raddr));

        if (recvlen < DEFAULT_BLOCK_SIZE)
        {
            last_err = CON_NO_ERROR;
            res = true;
            break;
        }
    }

downfail:

    fclose(fp);
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif

    return res;
}
