#ifndef SRS_HDR_UHNDER_PART_H
#define SRS_HDR_UHNDER_PART_H 1
#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

/*! Uhnder partition manager*/
#define UART_CHUNK_SIZE 512

#define SECTOR_SIZE 4096U
#define PAGE_SIZE 256U
#define FLASH_SIZE (4U * 1024U * 1024U)

// The user partition uses the remaining FLASH after the two system image partitions
// There are 5 sectors of overhead:
//      1 sector for the partition table
//      3 sectors for the partition headers of the 3 partitions
//      1 sector due to a bug
#define USER_SIZE   ((1024 * 1024) - (SECTOR_SIZE * 5))
#define SYSTEM_SIZE ((1024 + 512) * 1024)
#define B1_IMG_SIZE ((128) * 1024)

#define CBC_IV ((uint8_t*)"1234567812345678")
#define PARTITION_TABLE_START_SECTOR 0
/* limited to help with rtl sim runs */
#define MAX_NUM_PART 16U
#if SABINE_B
#define NUM_PART 5
#else
#define NUM_PART 3
#endif
#define FORMAT_MAGIC_BASE 0x78B8737AUL
#define FORMAT_MAGIC_XOR  0x4E23C531UL

#define UHNDER_PARTITION_MAGIC 0x54504855UL //"UHPT"
#define UHNDER_PARTITION_MAGIC_LEN 4
#define UHNDER_PARTITION_HEADER_MAGIC 0x48504855UL // "UHPH"
#define UHNDER_PARTITION_HEADER_MAGIC_LEN 4
#define UHNDER_IMAGE_MAGIC 0x4D494855UL // "UHIM"
#define UHNDER_IMAGE_MAGIC_LEN 4

// Flash driver shared structures between SCP and HSM
/*! Flash parition types */
#define TYPE_NONE   0U 
#define TYPE_SYSTEM TYPE_UHNDER_B2 /*! B2 parition*/
#define TYPE_USER   7U /*! User data parition*/
#define TYPE_UHNDER_B1 1U/*!B1 parition*/
#define TYPE_UHNDER_B2 6U/*!B2 parition*/
#define TYPE_CUSTM_IMG 3U/*!Customer image */
#define TYPE_DATA 4 /*! Customer data, RDC data*/
/*! */
/*!Partition flags */
#define FLAG_EMPTY      1U // (1 << 0)
#define FLAG_VALID      4U // (1 << 2)
#define FLAG_ENCRYPTED  16U// (1 << 4)

SRS_DECLARE_NAMESPACE()

/*!Enum for boot device errors*/
enum
{
    EPARTITIONTABLEMAGIC = 65500, /*! Partition table has wrong magic code*/
    EREADPARTITIONTABLE,        /*! Error in reading parition table */
    EBADPHPTR,                  /*! parition header is incorrect */
    ESEEKPH,                    /*! Error in seeking on boot device */
    EREADPH,                    /*! Error in reading partition header */
    EPHMAGIC,                   /*! parition header has wrong magic code*/
    ENOUI,                      /*! */
    EBADCHECK,                  /*! Bad checksum*/
};

/*! Structure defining the parition entry*/
struct partition_entry
{
    CHAR     name[8]; /*! Name of the parition, should not be more than 7 chars*/
    uint32_t sector_offset; /*! Sector offset on boot device(flash) for the partition*/
    uint32_t size_in_sectors;/*! Size of the parition in sectors */
    uint32_t type; /*! Type of the parition*/
};

/*! Structure defining the partition table
 magic - Magic number
 sector_size   4096
 sector_count    -total sectors on this device
 num_entries     - how many partitions
 warm_boot_count -  how many times can we reboot until we fall back onto the previous boot image
 min_uptime   - how long should a new image have been running until we deem it bad and fall back on previous boot image
 spi_clk      - what clk to run the QSPI/SSPI0 sclk
 wdt_load     - what value to load into the WDT
 qspi_mode    - if 4bit qspi mode is requested, this value is the non-zero read setting, for winbond 25q64 this value would be 0x080220EBUL
 read_delay   - qspi read delay adjustments, reset 0x0000U_0000
 read_capture - qspi read capture (sampling timing) adjustments, reset: 0x0000U_0001
                struct partition_entry partition[MAX_NUM_PART];  num entries partition[] objects

 placed at sector boundary, one sector in size
*/
struct partition_table
{
    uint32_t magic;/*!Magic number*/  
    uint32_t sector_size;   /*! 4096 */
    uint32_t sector_count;  /*! total sectors on this device */
    uint32_t num_entries;   /*! how many partitions */
    uint32_t warm_boot_count; /*! how many times can we reboot until we fall back onto the previous boot image */
    uint32_t min_uptime;    /*! how long should a new image have been running until we deem it bad and fall back on previous boot image */
    uint32_t spi_clk;   /*! what clk to run the QSPI/SSPI0 sclk */
    uint32_t wdt_load;  /*! what value to load into the WDT */
    uint32_t qspi_mode;     /*! if 4bit qspi mode is requested, this value is the non-zero read setting, for winbond 25q64 this value would be 0x080220EBUL */
    uint32_t read_delay;     /*! qspi read delay adjustments, reset 0x0000U_0000 */
    uint32_t read_capture;   /*! qspi read capture (sampling timing) adjustments, reset: 0x0000U_0001 */
/*!    struct partition_entry partition[MAX_NUM_PART];  num entries partition[] objects */
};


/*! Structure defining the partition header*/
/* placed at sector boundary, one sector in size */
struct partition_header
{
    uint32_t magic; /*! "UHPH" */
    uint32_t sector_offset; /*! the absolute sector number where this partition header sits in flash. For convenience */
    uint32_t sector_count; /*! valid sectors in this partition */
    uint32_t image_sequence_number;
    uint32_t flags;
    /*! to be generated by the sabine RNG upon encryption */
    uint32_t iv[4];
};

/*! Enum for Boot cmds*/
enum
{
    CMD_NOP,
    CMD_START_HSM,
/* the follwing are only used by the secondary loader in production mode*/
    CMD_START_CCP,
    CMD_START_R5FL,
    CMD_START_R5FR,
    CMD_START_P5_0,
    CMD_START_P5_1
};

/*! Enum for specifying which CPU the UHI belongs to*/
enum
{
    CPU_NONE,
    CPU_HSM,
    CPU_CCP,
    CPU_R5FL,
    CPU_R5FR,
    CPU_P5_0,
    CPU_P5_1
};

/*! Structure describing the B2 bundle HDR
 * B2 bundle is arranged in following format
 *     -------------------------------
 *     ---------B2 Bundle HDR---------
 *     -------------------------------
 *     -------Customer PubKey---------
 *     -------------------------------
 *     ----Signature for B2 Bundle----
 *     -------------------------------
 *     ----------UHI chunk 0----------
 *     -------------------------------
 *     ----------UHI chunk 1----------
 *     -------------------------------
 *                    .
 *                    .
 *                    .
 *     ---------UHI chunk N-1---------
 *     -------------------------------
 *     ----------UHI chunk N----------
 *     -------------------------------
 */
/*NOTE:Maintain the size of this struct to multiple of 32 bytes*/
struct uhnder_b2_block
{
    uint32_t magic; /*! 'UB2B' */
    uint32_t key_size; /*! Key size in bytes for customer key, 0 for Uhnder key*/
    uint32_t signature_size; /*! Size of signature in bytes*/
    uint32_t reserved0;
    uint32_t reserved1;
    uint32_t reserved2;
    uint32_t reserved3;
    uint32_t reserved4;
};

/*! structure defining the uhnder image HDR format
 * Every image that needs to be loaded and booted by the
 * Uhnder boot code needs to have a UHI HDR
 * A SW image for a CPU can contain multiple UHI chunks
 * each UHI chunk is contiguous in memory location on the
 * target and UHI HDR contains physical address as seen by HSM CPU
 * UHI chunk format is described below
 *
 *                -------------------
 *                ------UHI HDR------
 *                -------------------
 *                -----IMG data------
 *                -------------------
 *
 */
/*NOTE:Maintain the size of this struct to multiple of 16 bytes*/
struct uhnder_image
{
    uint32_t magic; /*! 'UHIM' */
    uint32_t size;  /*! in bytes, pad images to 16 byte boundary  for AES */
    uint32_t dest_addr; /*!Dest address should be physical address as seen by HSM CPU*/
    uint16_t cpu;   /*! CPU -  HSM,SCP,CCP,P5.the target CPU for this image.This is optional for image other than B1*/
    uint16_t command;
    uint32_t checksum;
    uint32_t crypto_id; /*! Depricated*/
    uint32_t key_size; /*! Depricated*/
    uint32_t signature_size; /*! Depricated*/
};


SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_UHNDER_PART_H
