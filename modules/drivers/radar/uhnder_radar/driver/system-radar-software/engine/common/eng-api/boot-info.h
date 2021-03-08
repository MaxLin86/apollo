#ifndef SRS_HDR_BOOT_INFO_H
#define SRS_HDR_BOOT_INFO_H 1

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"


SRS_DECLARE_NAMESPACE()

#define BOOT_INFO_BOOTED_NON_VALIDATED_IMG 0x4E564C44UL
#define BOOT_INFO_IMG_VALIDATED            0x574F524BUL
#define BOOT_INFO_WARM_BOOT_MAGIC          0x5741524DUL
#define BOOT_INFO_COLD_BOOT_MAGIC          0x434F4C44UL

//! B0 boot info
#define BOOT_B0_INFO_STRUCT_LOCATION       0x11FF00U

//! All boot info marker locations
#define BOOT_INFO_STRUCT_LOCATION          0x11FF80U
#define WARM_BOOT_MARKER_ADDR              0x11FFECU
#define PART_NUM_BOOT_IMG_ADDR             0x11FFF0U
#define NON_VALIDATED_IMG_BOOTED_ADDR      0x11FFF4U

#define HSM_BOOT_MARKER_ADDR               0x11FFF8U
#define HSM_RAM_BOOT_ADDR                  0x11FFFCU

#define BOOT_ERR_NONE 0
#define BOOT_ERR_BIST_NOT_STARTED           -1
#define BOOT_ERR_BIST_FAILED                -2
#define BOOT_ERR_SHUTDOWN -3
#define BOOT_ERR_REBOOT -4
#define BOOT_ERR_DEVICE_KEY_WARM_BOOT -5
#define BOOT_ERR_DEVICE_KEY_ERR -6
#define BOOT_ERR_LOAD_B2_ERR -7
#define BOOT_ERR_LOAD_B2_MEM_VIOLATION_ERR -8
#define BOOT_ERR_LOAD_B2_INVALID_PART_ERR -9
#define BOOT_ERR_AUTH_INVALID_KEY_ERR -10
#define BOOT_ERR_TRNG_CONFIG_ERR -11
#define BOOT_ERR_DEVICE_KEY_PROV_ERR -12
#define BOOT_ERR_DEVICE_KEY_LC_ADVANCE_ERR -13
#define BOOT_ERR_TRGN_READ_FIFO_EMPTY_ERR -14
#define BOOT_ERR_TRGN_COND_CHECK_ERR -14
#define BOOT_ERR_TRGN_COND_BUSY_CHECK_ERR -15
#define BOOT_ERR_TRGN_COND_READ_FIFO_ERR -16
#define BOOT_ERR_TRGN_WRITE_TST_FIFO_ERR -17
#define BOOT_ERR_TRGN_ENTROPY_CHECK_ERR -18
#define BOOT_ERR_TRGN_CONFIG_ERR -19
#define BOOT_ERR_BOOT_DEV_FLASH_INIT_ERR -20
#define BOOT_ERR_BOOT_PART_ERR -21
#define BOOT_ERR_LOAD_B1_ERR -22
#define BOOT_ERR_BOOT_B1_ERR -23
#define BOOT_ERR_BOOT_DEV_DCU_INIT_ERR -24
#define BOOT_RSA_AUTH_STARTED 1

#define MAX_ALLOWED_BOOT_COUNT 5

struct b0_boot_info
{
    int32_t boot_mode;
    int32_t lifecycle;
    uint32_t boot_log_wr;
    uint32_t log_wrap;
};

struct sabine_boot_info
{
    uint32_t is_warm_boot : 1;
    uint32_t is_dcu_boot : 1;
    uint32_t fallback_to_valid_img : 1;
    uint32_t wdt_reset : 1;
    uint32_t reserved : 28;
};

typedef enum BootMode
{
    BOOT_MODE_FLASH = 0,
    BOOT_MODE_FTU = 1,
    BOOT_MODE_UART_CONNECTED = 2,
    BOOT_MODE_UART_MONITOR = 3,
    BOOT_MODE_DCU = 4,
}BootMode_t;

int32_t get_hw_boot_modes(uint32_t *uart_con);

uint32_t check_add(uint32_t sum, uint32_t *buf, uint32_t len);

#if !__BARE_METAL__
void set_hw_boot_mode(BootMode_t mode);
#endif

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_BOOT_INFO_H
