#ifndef SRS_HDR_DEVMM_H
#define SRS_HDR_DEVMM_H 1
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

SRS_DECLARE_NAMESPACE()

typedef enum
{
    MEM_SCP_TCMA,    /*! SCP/R5 tightly coupled memory */
    MEM_SCP_TCMB,    /*! SCP/R5 tightly coupled memory */
    MEM_SYSTEM_MEM,  /*! general purpose system SRAM */
    MEM_DSP_SRAM0,   /*! DSP/P5 tightly coupled memory */
    MEM_DSP_SRAM1,   /*! DSP/P5 tightly coupled memory */
    MEM_DRAM,        /*! external DRAM */
    MEM_DCU_SRAM,    /*! radar data memory */
    MEM_NONE,        /* not in device mmeory */
} MemoryTypeEnum;

/*! statcks are allocated in this order at boot time */
enum StackEnum
{
    CODE_DRAM,
    OBJECT_DRAM,     /*! 2MB in reserved area of DRAM */
    ETH_MEM,         /*! iovec pools and gmac buffers and lists */
#if SABINE_B
    RHAL_DRAM_ECC,
#endif
    RHAL_DRAM,       /*! 124MB of DRAM_RSP (remaining)   */

#if PROTIUM_EMULATION
    DDR_SIDE_LOAD,   /*! 100MB reserved for side loading test input on protium. This should accommodate RDC1 for R(256)*Vrx(96)*PRI(1008) */
    DDR_DEBUG,
    DDR_SEQ_CORE_DUMP,
#endif
    RDC_DRAM,        /*! 8MB in reserved area of DRAM_USER */
    TFS_MEM,
    RHAL_DRAM_USER,

    RHAL_DCU,        /*! all 4MB of DCU SRAM             */

    /* The USER_SRAM stack covers all of SRAM, but the lower portion is
     * controlled by the OS. It will reserve the SRAM area consumed by static
     * structures, ARM lib heap, and the SCP execution stack */
    USER_SRAM,
    USER_SRAM_UNCACHED,

    RHAL_TCMA,
    RHAL_TCMB,
    /* define more stacks here */

    NUM_STACKS
};

/*! returns zero on success, negative on error */
INT uh_get_available_memory(MemoryTypeEnum, void**ptr, size_t * size_bytes);

#if __SCP__

struct MallocInstance;

class StackAllocator
{
public:

    MemoryTypeEnum  get_memory_type() const             { return mem_type; }

    const CHAR *    get_name() const;

    size_t          get_available_memory_size() const
    {
        return mem_size - lower_mem_bytes_allocated
               - upper_mem_bytes_allocated;
    }

    size_t          get_in_use_memory_size() const
    {
        return lower_mem_bytes_allocated + upper_mem_bytes_allocated;
    }


    void            dump_allocations() const;

    void*           alloc_upper_stack(size_t size);

    /* you can only allocate lower stack memory if the reserved size is 0,
     * and you can only set the reserved size if there are no lower stack
     * allocations. */
    void*           alloc_lower_stack(size_t size);
    void*           set_lower_reserved_size(size_t size);
    uint32_t        get_lower_reserved_size() const;

    /*! the last lower stack allocation is resizable. You must pass in the
     * allocation pointer, for validation purposes. Returns false if the pointer
     * is wrong or if there is insufficient room for the new size */
    bool            resize_last_lower_alloc(void* ptr, size_t newsize);

    /* release will fail if the pointer does not refer to the last allocation
     * from either the upper or lower stack */
    bool            release(const void* ptr);

protected:

    friend class DeviceMemoryManager;

    void init(MemoryTypeEnum type, uint32_t size, uint32_t maxalloc);

    MemoryTypeEnum  mem_type;
    uint32_t        mem_size;
    uint32_t        max_allocations;

    /* dynamic data tracking allocations */
    CHAR*           alloc_region;
    size_t          lower_mem_bytes_allocated;
    size_t          upper_mem_bytes_allocated;
    MallocInstance* lower_head; /* pointer to lower stack alloc linked list */
    MallocInstance* upper_head; /* pointer to upper stack alloc linked list */
    MallocInstance* free_head;  /* pointer to free list of malloc instances */
};

#endif // if __SCP__

class DeviceMemoryManager
{
public:

    /*! Pointers and byte sizes of all addressable embedded memories */
    struct DevMemDesc
    {
        CHAR*  ptr;
        size_t size;
        size_t bytes_allocated;
    };

    DevMemDesc mem[MEM_NONE];

    /*! returns true if the specified pointer is in the address space of one of
     * the known device memories, false otherwise. When it returns true it also
     * sets 'e' to the memory type and offset to the 'offset' into that memory */
    bool  get_pointer_info(const void* const ptr, MemoryTypeEnum& e, uint32_t& offset) const;

    /*! one-time (boot) initialization, calls uh_get_available_memory() to learn
     * the size of all available memories from platform-hal. On non-embedded
     * systems the memories are simulated by mallocs */
    void init();

#if __SCP__

    /*! called by OSLayer::init() to configure stack objects */
    void     init_stacks();

    /*! allocate available memory from a specific memory type.  The actual
     * allocation might be slightly larger for alignment purposes */
    void*    alloc_memory(MemoryTypeEnum mem_type, size_t size);

    /*! return the amount of memory available in the specified memory */
    uint32_t get_memory_available(MemoryTypeEnum mem_type) const;

    void     dump_allocations() const;

    static StackAllocator stack[NUM_STACKS];

#endif // if __SCP__
};

extern DeviceMemoryManager* devmm;

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_DEVMM_H
