#ifndef SRS_HDR_BOOT_INFO_H
#define SRS_HDR_BOOT_INFO_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

#define B1_BOOT_INFO_MAGIC 0x42014D4EUL
#define BOOT_INFO_BOOTED_NON_VALIDATED_IMG 0x4E564C44UL
#define BOOT_INFO_IMG_VALIDATED            0x574F524BUL
#define BOOT_INFO_WARM_BOOT_MAGIC          0x5741524DUL
#define BOOT_INFO_COLD_BOOT_MAGIC          0x434F4C44UL
#define BOOT_INFO_DCU_BOOT_MAGIC           0x44635542UL

//! B0 boot info
#define BOOT_B0_INFO_STRUCT_LOCATION       0x001017E0UL

#if SABINE_A
//! All boot info marker locations
#define BOOT_INFO_STRUCT_LOCATION          0x11FF80U
#define WARM_BOOT_MARKER_ADDR              0x11FFECU
#define PART_NUM_BOOT_IMG_ADDR             0x11FFF0U
#define NON_VALIDATED_IMG_BOOTED_ADDR      0x11FFF4U
#elif SABINE_B
//! All boot info marker locations
#define BOOT_B1_INFO_STRUCT_LOCATION          0x13FF80U
#define WARM_BOOT_MARKER_ADDR              0x13FFECU
#define PART_NUM_BOOT_IMG_ADDR             0x13FFF0U
#define NON_VALIDATED_IMG_BOOTED_ADDR      0x13FFF4U
#endif

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
#define BOOT_END_OF_UHI_DATA -25
#define BOOT_NEW_CRYPTO_HDR 1
#define BOOT_NEW_UHI_HDR 2
#define BOOT_LOAD_B2_COMPLETE 3
#define MAX_ALLOWED_BOOT_COUNT 5

struct sabine_boot_info
{
    uint32_t is_warm_boot : 1;
    uint32_t is_dcu_boot : 1;
    uint32_t fallback_to_valid_img : 1;
    uint32_t wdt_reset : 1;
    uint32_t reserved : 28;
};
struct b0_boot_info
{
    int32_t boot_mode;
    int32_t lifecycle;
    uint32_t boot_log_wr;
    uint32_t log_wrap;
};

struct b1_boot_info
{

    uint32_t magic;
    int32_t boot_mode;
    int32_t is_warm_boot;
    uint32_t warm_boot_magic;
    uint32_t dcu_boot_magic;
    uint32_t dcu_boot_addr;
    int32_t flash_part;
    int32_t validated;
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

class BootInfo
{
public:

    static BootInfo& instance(void);

    void set_b0_info_addr(struct b0_boot_info *ptr){p_boot_info = ptr;}

    void set_b0_info_boot_mode(int32_t bootmode);

    void set_b0_info_lifecycle(int32_t lc);

    void set_b0_info_boot_log_wr(uint32_t log_wr);

    void set_bo_info_boot_log_wrap(uint32_t wrap);

    int32_t get_b0_info_boot_mode();
private:
    struct b0_boot_info *p_boot_info;

};

class B1BootInfo
{
public:
    static B1BootInfo& instance(void);

    void set_b1_info_addr(struct b1_boot_info *ptr){p_b1_boot_info = ptr;}

    void set_b1_info_boot_mode(int32_t bootmode);

    int32_t get_b1_info_boot_mode(void);

    void log(void);

    void set_b1_info_warm_boot(int32_t warmboot);

    int32_t get_b1_info_warm_boot(void);

    void reset_b1_info_warm_boot_magic(void);

    void set_b1_info_warm_boot_magic(uint32_t magic);

    uint32_t get_b1_info_warm_boot_magic(void);

    uint32_t get_b1_info_magic(void);

    void set_b1_dcu_boot(uint32_t magic, uint32_t addr);

    void get_b1_dcu_boot(uint32_t *magic, uint32_t* boot_mode,uint32_t* addr);

    void reset_b1_dcu_boot(void);

    void set_b1_info_magic(uint32_t boot_magic);

    void set_b1_info_flash_part(int32_t flash_part);

    void set_b1_info_validated(int32_t valid);

    int32_t get_b1_info_validated(void);

    int32_t get_b1_info_flash_part(void);
private:
    struct b1_boot_info *p_b1_boot_info;

};
#if !__BARE_METAL__
void set_hw_boot_mode(BootMode_t mode);
#endif
SRS_CLOSE_NAMESPACE()
#endif // SRS_HDR_BOOT_INFO_H
