#ifndef SRS_HDR_DMAC_API_H
#define SRS_HDR_DMAC_API_H 1

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/logging-enums.h"
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/engine/common/eng-api/event-enums.h"
#include "dma-enums.h"

SRS_DECLARE_NAMESPACE()

#if SABINE_A
#define DMAC_MASTER_CCP         0
#define DMAC_MASTER_HSM         1
#define DMAC_MASTER_PERI0       2
#define DMAC_MASTER_PER11       3
#define DMAC_MASTER_RSP0        4
#define DMAC_MASTER_RSP1        5
#define DMAC_MASTER_SCP0        6
#define DMAC_MASTER_SCP1        7
#define DMAC_MASTER_RSP0_DMA    8
#define DMAC_MASTER_RSP1_DMA    9
#define DMAC_MASTER_DMAC       10
#define DMAC_MASTER_GMAC       11
#define DMAC_MASTER_H264       12
#define DMAC_MASTER_RSP_SS_MAS 13
#define DMAC_MASTER_FLEXRAY    14
#elif SABINE_B  
#define DMAC_MASTER_CCP                     0
#define DMAC_MASTER_HSM                     1
#define DMAC_MASTER_SCP_R5_0_PERIPHERAL     2
#define DMAC_MASTER_SCP_R5_1_PERIPHERAL     3
#define DMAC_MASTER_RSP_P5_0                4
#define DMAC_MASTER_RSP_P5_1                5
#define DMAC_MASTER_SCP_R5_0_MASTER         6
#define DMAC_MASTER_SCP_R5_1_MASTER         7
#define DMAC_MASTER_RSP_P5_0_IDMA           8
#define DMAC_MASTER_RSP_P5_1_IDMA           9
#define DMAC_MASTER_ETHERNET_CONTROLLER_2   10
#define DMAC_MASTER_ETHERNET_CONTROLLER_0   11
#define DMAC_MASTER_RESERVED                12
#define DMAC_MASTER_RSP_SS                  13
#define DMAC_MASTER_RESERVED1               14
#define DMAC_MASTER_ETHERNET_CONTROLLER_1   15
#endif

#define DMA_MAX_BURST_LENGTH  0x7FU // 0x7FU
#define DMA_MAX_WORD_SIZE     0x03U // 0x3U
#define CHANNEL_ADDR_OFFSET   0x100U
//Changing this from 8 bytes to 4 bytes so that 4 byte aligned data can also be tested.
#define DMA_BUS_WIDTH 4 

//DMAC Error codes 
enum DmacErr 
{
    DMAC_SUCCESS  = 0,
    DMAC_FAILURE = -1,
};

//Enumeration values for DMAC secure mode
enum DmacSecureMode
{
    DMAC_ENABLE_SECURE_MODE,
    DMAC_DISABLE_SECURE_MODE
};

// Structure for initializing the DMAC module 
struct dma_init_param
{
    uint32_t peri_dma_cpu_id; // CPU id for the Peripheral DMA channel(channel 0)
    uint32_t hsm_ch_count;
    uint32_t ccp_ch_count;
    uint32_t scp_0_ch_count;
    uint32_t scp_1_ch_count;// For lockstep use scp_0 count and set this to 0
    uint32_t p5_0_ch_count;
    uint32_t p5_1_ch_count;
};

// Structure for configuring the dMAC module 
struct dma_params
{
    uint32_t channel_num; // DMAC channel number 
    uint32_t* src_addr; // Source address 
    uint32_t* dst_addr; // destination address 
    uint32_t num_bytes; // Number of bytes to transfer 
    uint32_t src_stride;
    uint32_t dest_stride;
    uint32_t x_length;
    uint32_t y_length;
    enum dmaDriver_transfer_loc src_loc_type;
    enum dmaDriver_transfer_loc dst_loc_type;
    enum dmaDriver_transfer_mode trans_mode;
    enum dmaDriver_transfer_type trans_type;
};

SRS_CLOSE_NAMESPACE()

#endif
