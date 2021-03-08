#ifndef SRS_HDR_OSIOVEC_H
#define SRS_HDR_OSIOVEC_H 1
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
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/engine/common/eng-api/logging.h"
#include "event-handler.h"

SRS_DECLARE_NAMESPACE()

/* Base abstract interface which is passed to ethernet driver to send/receive
 * ethernet packets (and perhaps other datagram interfaces) */
class BaseIOVec
{
public:

    virtual void*     get_data_pointer() = 0;
    virtual uint32_t  get_data_length() const = 0;

    virtual void*     get_buffer_pointer() = 0;
    virtual uint32_t  get_buffer_size() const = 0;
    virtual void      release() = 0;

    bool      is_bare() const        { return get_buffer_size() == 0; }

    bool      is_free() const        { return state == IOVEC_STATE_FREE; }

    bool      is_alloc() const       { return state == IOVEC_STATE_USER_ALLOC; }

    bool      is_eth() const         { return state == IOVEC_STATE_ETH_DRIVER; }

    enum IovecStatesEnum
    {
        IOVEC_STATE_FREE,                   /*! this iovec is in a free list     */
        IOVEC_STATE_USER_ALLOC,             /*! this iovec is owned by user code */
        IOVEC_STATE_ETH_DRIVER,             /*! this iovec is queued for transmit */
    };

    enum { USER_DATA_SIZE = 8 };

    CHAR              user_data[USER_DATA_SIZE];
    uint32_t          src_interface;        // incomimg packets only
    uint32_t          local_ip;             // incomimg packets only
    BaseIOVec*        iovec_next;
    int32_t           tx_complete_handle;
    int32_t           num_iovec_in_packet;
    IovecStatesEnum   state;
    bool              end_of_frame;       /* must be set on last iovec of a packet */
};

class LoadedIOVec : public BaseIOVec
{
public:

    virtual void*    get_data_pointer()      { return payload + offset; }

    virtual uint32_t get_data_length() const { return data_length; }

    virtual void*    get_buffer_pointer()    { return payload; }

    virtual uint32_t get_buffer_size() const { return buffer_size; }

    virtual void     release();

    void dump_info() const
    {
        UHPRINTF("%p payload %p offs:%d len:%d max:%d bufsize:%d\n",
                 this, payload + offset, offset,
                 data_length, max_payload_size, buffer_size);
    }


    /*! only for use by ethernet driver to set the size of the buffer which has
     * been filled in by hardware */
    void set_payload_length(uint32_t recvlen)
    {
        UHASSERT(recvlen <= max_payload_size);
        data_length = recvlen;
    }


    /*! append 'len' bytes of additional data to the end of the current payload
     * data. returns false if there was insufficient room */
    bool payload_append(const void* ptr, uint32_t len)
    {
        if (data_length + len <= max_payload_size)
        {
            uh_memcpy(payload + offset + data_length, ptr, len);
            data_length += len;
            return true;
        }
        else
        {
            return false;
        }
    }


    /*! append any structure to the payload, returns false if there was
     * insufficient room */
    template<class T>
    bool payload_append(T& data)
    {
        if (data_length + sizeof(T) <= max_payload_size)
        {
            uh_memcpy(payload + offset + data_length, &data, sizeof(T));
            data_length += sizeof(T);
            return true;
        }
        else
        {
            return false;
        }
    }


    /*! append room for a struct to the end of the current payload data. returns
     * NULL if there is insufficient room */
    template<class T>
    T* payload_append()
    {
        if (data_length + sizeof(T) <= max_payload_size)
        {
            T* ret = reinterpret_cast<T*>(payload + offset + data_length);
            data_length += sizeof(T);
            return ret;
        }
        else
        {
            return NULL;
        }
    }


    /*! remove last payload byte if it is a NUL byte (\0), a convenience method
     * for then NUL characters are used to separate strings in packets */
    void trim_nul()
    {
        if (data_length && !payload[offset + data_length - 1])
        {
            data_length--;
        }
    }


    /*! prepend a header to the existing data contents of the iovec. We
     * explicitly reserve room at the start of the payload buffer when we
     * allocate new LoadedIOVec to allow typical protocol headers to be
     * prepended */
    template<class T>
    T* push_header()
    {
        if (sizeof(T) <= offset)
        {
            offset -= sizeof(T);
            data_length += sizeof(T);
            return reinterpret_cast<T*>(payload + offset);
        }
        else
        {
            return NULL;
        }
    }


    /*! Return a pointer to the first header, of type T, and advance the
     * data offset pointer past this header. */
    template<class T>
    T* pop_header()
    {
        if (data_length >= sizeof(T))
        {
            offset += sizeof(T);
            data_length -= sizeof(T);
            return reinterpret_cast<T*>(payload + offset - sizeof(T));
        }
        else
        {
            return NULL;
        }
    }


    /*! for use by the application layer (aka: scc agent), this method consumes
     * the content of the message payload, and resets the iovec to its default
     * state in preparation for new payload to be added for a return packet
     * returns false if the payload length did not match the structure size
     * **You must call align_header before appending any payload** */
    template<class T>
    bool consume_payload(T& msg)
    {
        if (data_length == sizeof(T))
        {
            uh_memcpy(&msg, payload + offset, sizeof(T));
            offset = MAX_PREPENDED_HEADER_BYTES;
            data_length = 0;
            return true;
        }
        else
        {
            offset = MAX_PREPENDED_HEADER_BYTES;
            data_length = 0;
            return false;
        }
    }


    /*! this version of consume_payload() presumes that there might be
     * additional payload data after the message (for read or write).
     * Only returns false if the payload was less than the size of the message.
     * The addtl_payload pointer is set to NULL if there was no additional payload
     * (and length will be 0). Note that the additional payload should be
     * consumed or copied prior to overwriting the contents of this iovec for
     * building a return packet. **You must call align_header with the correct
     * number of header bytes you intend to prepend to the iovec before you
     * append any payload, or you will cause an assert or driver crash** */
    template<class T>
    bool consume_payload(T& msg, const CHAR*& addtl_payload, uint32_t& length)
    {
        if (data_length >= sizeof(T))
        {
            uh_memcpy(&msg, payload + offset, sizeof(T));
            data_length -= sizeof(T);
            if (data_length)
            {
                addtl_payload = payload + offset + sizeof(T);
                length = data_length;
            }
            else
            {
                addtl_payload = NULL;
                length = 0;
            }
            offset = MAX_PREPENDED_HEADER_BYTES;
            data_length = 0;
            return true;
        }
        else
        {
            addtl_payload = NULL;
            length = 0;
            offset = MAX_PREPENDED_HEADER_BYTES;
            data_length = 0;
            return false;
        }
    }


    /* this function must be called on empty IOVEC to ensure that once the final
     * header has been added the final packet will be properly aligned for the
     * ethernet hardware to transmit */
    void align_header(uint32_t header_bytes)
    {
        UHASSERT(data_length == 0);
        offset = header_bytes + MIN_BYTE_ALIGN;
        offset -= (uintptr_t)(payload) % MIN_BYTE_ALIGN;
    }


protected:

    enum { MAX_PREPENDED_HEADER_BYTES = 64 };

    enum { MIN_BYTE_ALIGN = 8 };

    void reset(uint32_t payload_bytes, uint32_t header_bytes)
    {
        max_payload_size = payload_bytes;
        data_length = 0;
        align_header(header_bytes);
        end_of_frame = true;
        iovec_next = NULL;
        num_iovec_in_packet = 1;
        tx_complete_handle = -1;
    }


    uint32_t offset;
    uint32_t data_length;
    uint32_t buffer_size;
    uint32_t max_payload_size;

    /*! the size of this element is meaningless, since we allocate additional
     * room after each instance to provide room for payload */
    CHAR     payload[MAX_PREPENDED_HEADER_BYTES];

    friend class IOVecPool;
};

class BareIOVec : public BaseIOVec
{
public:

    void     set_data_length(uint32_t l) { len = l; }

    void     set_data_pointer(void* ptr) { data_ptr = ptr; }

    virtual void*    get_data_pointer()      { return data_ptr; }

    virtual uint32_t get_data_length() const { return len; }

    virtual void*    get_buffer_pointer()    { return NULL; }

    virtual uint32_t get_buffer_size() const { return 0; }

    virtual void     release();

protected:

    void reset()
    {
        end_of_frame = true;
        data_ptr = 0;
        len = 0;
        iovec_next = NULL;
        num_iovec_in_packet = 1;
        tx_complete_handle = -1;
    }


    static BareIOVec* free_list;

    void*    data_ptr;
    uint32_t len;

    friend class IOVecPool;
};

struct PoolStats
{
    uint32_t total_allocations;
    uint32_t total_releases;
    uint32_t failed_allocations;
    uint32_t current_available;
    uint32_t num_iovec_in_pool;
    uint32_t buffer_size;
    uint32_t total_pool_size;
};

class IOVecPool : public EventHandler
{
public:

    static IOVecPool& instance();

    void            init();

    /*! returns a BareIOVec instance, user must call set_data_pointer() and
     * set_data_length() */
    BareIOVec*      allocate_bare_iovec();

    /*! returns a LoadedIOVec instance, user writes payload into pointer
     * get_buffer_pointer() then pass to UHDP or other layer to prepend
     * appropriate headers. Returned buffer will have room for at least
     * payload_bytes bytes of payload data */
    LoadedIOVec*    allocate_iovec(uint32_t max_payload, uint32_t header_size);

    uint32_t        get_num_pools() const;

    const PoolStats& get_pool_stats(uint32_t pool_num) const;

    uint32_t        get_num_max_size_iovec_available() const;

    void            dump_pool_stats() const;

protected:

    virtual void trigger(int8_t*); // IOVEC_ALLOC_FAIL_TIMER

    uint32_t     alloc_failures;

    enum { NUM_BARE_IOVEC = 256 };

    /* loaded iovec pool info is in osiovec.cpp */

    void release_loaded(LoadedIOVec&);

    friend class LoadedIOVec;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_OSIOVEC_H
