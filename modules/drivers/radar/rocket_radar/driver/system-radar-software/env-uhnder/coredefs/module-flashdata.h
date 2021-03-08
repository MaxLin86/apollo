#ifndef SRS_HDR_MODULE_FLASHDATA_H
#define SRS_HDR_MODULE_FLASHDATA_H 1

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-uhnder/coredefs/uhstdlib.h"

//! enumeration of the calibration flash files
enum CalFlashEnum
{
    CAL_DC,
    CAL_TCS,
    CAL_VRX,
    CAL_IQ,
    CAL_TEMP,
    CAL_ANGLE,
    CAL_DIAGONAL,
    CAL_STEERING,
    CAL_QILO,
    CAL_LDO,
    CAL_VTR,
    CAL_TRIMS,
    CAL_CDATA,
    CAL_LDO_PARAMS,

    NUM_CAL_FLASH
};

SRS_DECLARE_NAMESPACE()

//! This structure defines the contents of the 'network' file on the flash
//! filesystem. Since it deals with IP networking and other operating-system
//! level details, it is considered an environment structure.

struct ModuleFlashData
{
    static const uint32_t MAGIC_VALUE = 0xD00DBABEUL;

    enum { FILE_VERSION = 13 };

    //! must be MAGIC_VALUE, else file is considered invalid
    uint32_t magic;

    //! file format version, currently FILE_VERSION
    uint32_t version;

    //! The source IPv4 address this radar
    uint32_t radar_ip_address;

    //! The source ethernet (MAC) address of this radar.
    //!
    //! it must have a prefix of 0x70B3D55FULB. The last three nibbles should be
    //! unique for all radars which share a LAN
    uint8_t  radar_ethernet_address[6];

    //! When multiple radars are used simultaneously, they must be given unique
    //! pmcw_channel values, to ensure they each transmit unique codes and
    //! reduce their cross-interference
    uint16_t pmcw_channel;

    //! identifying name for the radar, documentary purposes only
    CHAR     radar_hostname[32];

    //! identifying name for the board on which this flash resides, documentary purposes only
    CHAR     board_name[32];

    //! identifying chip id for the board on which this flash resides, documentary purposes only
    uint32_t chip_id[2];

    //! identifying string of the antenna module (Rhine type waveguides)
    CHAR     antenna_module_name[32];

    enum { BOOT_CONFIG_SIZE_BYTES = 128 };
    //! various string options which can modify the behavior of engine or
    //! environment code (similar to Linux kernel boot options)
    CHAR     boot_config[BOOT_CONFIG_SIZE_BYTES];

    uint32_t boot_mode;                 //!< enum RHAL_BootMode

    float    rear_axle_dist_m;          //!< Dist to rear axle
    float    center_line_dist_m;        //!< Dist from centerline
    float    sensor_rotation_az_deg;    //!< Degrees off boresight in azimuth
    float    sensor_rotation_el_deg;    //!< Degrees off boresight in elevation
    float    sensor_height_m;           //!< Height of radar

    uint32_t ext_lo_enable_flags;       //!< enum RHAL_LOMode

    FLOAT    force_pll_freq;            //!< Force PLL to lock at given freq

    INT      adc_ldo_code;

    int8_t   txu_dll_phase;             //!< TXU DLL Phase

    enum GMAC_PHY_SPEEDS
    {
        PHY_SPEED_AUTODETECT,
        PHY_SPEED_10Mb,
        PHY_SPEED_100Mb,
        PHY_SPEED_1Gb,
    };

    uint8_t  ethernet_speed_gmac0_phy;  //!< enum GMAC_PHY_SPEEDS
    uint8_t  ethernet_speed_gmac1_phy;  //!< enum GMAC_PHY_SPEEDS
    uint8_t  ethernet_speed_gmac2_phy;  //!< enum GMAC_PHY_SPEEDS

    uint8_t  radar_ethernet_address2[6];
    uint8_t  radar_ethernet_address3[6];
    uint32_t radar_ip_address2;
    uint32_t radar_ip_address3;

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));

        magic = MAGIC_VALUE;

        version = FILE_VERSION;

#if PALLADIUM_EMULATION
        radar_ip_address  = 0x7f000001UL; // 127.0.0.1
        radar_ip_address2 = 0x7f000101UL; // 127.0.1.1
        radar_ip_address3 = 0x7f000201UL; // 127.0.2.1

        const uint8_t factory_default_mac[] =
        {
            0xA0U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U
        };
#else
        radar_ip_address  = 0xC0A85F01UL; // 192.168.95.1
        radar_ip_address2 = 0xC0A86001UL; // 192.168.96.1
        radar_ip_address3 = 0xC0A86101UL; // 192.168.97.1

        const uint8_t factory_default_mac[] =
        {   // we are using the prefix 70B3D55FB and range 0x000U to 0xFFFU
            0x70U, 0xB3U, 0xD5U, 0x5FU, 0xBFU, 0xFFU
        };
#endif

        uh_memcpy(radar_ethernet_address, factory_default_mac, sizeof(factory_default_mac));
        uh_memcpy(radar_ethernet_address2, factory_default_mac, sizeof(factory_default_mac));
        uh_memcpy(radar_ethernet_address3, factory_default_mac, sizeof(factory_default_mac));
        radar_ethernet_address2[4] -= 1; // make 2nd and 3rd GMAC addresses unique
        radar_ethernet_address3[4] -= 2;

#if __BARE_METAL__
        strcpy(radar_hostname, "no factory cal");
        ext_lo_enable_flags = 0;
#else
        strcpy(radar_hostname, "localhost");
        const CHAR* env_ant = uh_getenv("SRS_ANTENNA_CONFIG_NAME");
        if (env_ant)
        {
            printf("Using SRS_ANTENNA_CONFIG_NAME environment variable '%s'\n", env_ant);
            strcpy(antenna_module_name, env_ant);
        }
#if SABINE_A
        ext_lo_enable_flags = 1;    // For x86-A, default to OBLO
#elif SABINE_B
        ext_lo_enable_flags = 0;    // FOr x8b-B, default to ILO
#endif
#endif

#if SABINE_B && SRS_PLATFORM_SOC
        boot_mode = 0U; // no analog init yet
#else
        boot_mode = 2U;
#endif

        // Default vehicle mounting geometry
        rear_axle_dist_m = 3.91F;      // Dist to rear axle
        center_line_dist_m = 0.0F;     // Dist from centerline
        sensor_rotation_az_deg = 0.0F; // Degrees off boresight
        sensor_rotation_el_deg = 0.0F;
        sensor_height_m = 0.7F;        // Height of radar

        force_pll_freq = -1.0f;
        adc_ldo_code = -1;
        txu_dll_phase = 4;
    }

    // return true if the file needs to be written back
    uint32_t check_version()
    {
        // C string safety
        radar_hostname[sizeof(radar_hostname) - 1] = 0;
        board_name[sizeof(board_name) - 1] = 0;
        antenna_module_name[sizeof(antenna_module_name) - 1] = 0;
        boot_config[sizeof(boot_config) - 1] = 0;

        if (version < 5)
        {
            // versions older than this are not migratable
            set_defaults();
        }
        else
        {
            // set new fields to factory default values
            ModuleFlashData def;
            def.set_defaults();

            if (version < 6)
            {
                uh_memcpy(boot_config, def.boot_config, sizeof(def.boot_config));
            }
            if (version < 7)
            {
                boot_mode               = def.boot_mode;
            }
            if (version < 8)
            {
                rear_axle_dist_m        = def.rear_axle_dist_m;
                center_line_dist_m      = def.center_line_dist_m;
                sensor_rotation_az_deg  = def.sensor_rotation_az_deg;
                sensor_rotation_el_deg  = def.sensor_rotation_el_deg;
                sensor_height_m         = def.sensor_height_m;
            }
            if (version < 9)
            {
                ext_lo_enable_flags     = def.ext_lo_enable_flags;
                force_pll_freq          = def.force_pll_freq;
            }
            if (version < 10)
            {
                adc_ldo_code            = def.adc_ldo_code;
            }
            if (version < 11)
            {
                txu_dll_phase           = def.txu_dll_phase;
            }
            if (version < 12)
            {
                radar_ip_address2       = def.radar_ip_address2;
                radar_ip_address3       = def.radar_ip_address3;
                uh_memcpy(&radar_ethernet_address2[0], &def.radar_ethernet_address2[0], sizeof(radar_ethernet_address2));
                uh_memcpy(&radar_ethernet_address3[0], &def.radar_ethernet_address3[0], sizeof(radar_ethernet_address3));
            }
            if (version < 13)
            {
#if SABINE_B
                ethernet_speed_gmac0_phy = uint8_t(PHY_SPEED_100Mb);
                ethernet_speed_gmac1_phy = uint8_t(PHY_SPEED_100Mb);
                ethernet_speed_gmac2_phy = uint8_t(PHY_SPEED_100Mb);
#else
                ethernet_speed_gmac0_phy = uint8_t(PHY_SPEED_AUTODETECT);
                ethernet_speed_gmac1_phy = uint8_t(PHY_SPEED_AUTODETECT);
                ethernet_speed_gmac2_phy = uint8_t(PHY_SPEED_AUTODETECT);
#endif
            }
        }

        uint32_t old_version = version;

        version = FILE_VERSION;

        return old_version;
    }

    static ModuleFlashData& instance();

    void init();

    void apply_flash_data(const ModuleFlashData& data);

    void report_network_config() const;

    bool check_boot_config(const CHAR* test_str) const;

    const CHAR* const * find_boot_config(const CHAR* test_str) const;

    int32_t get_network_config(int32_t id, uint32_t *ip_addr, uint8_t *mac_addr);

    void get_ip_addr(const uint32_t gmac_id, uint32_t* ip_addr) const;

    void get_mac_addr(const uint32_t gmac_id, uint8_t* mac_addr) const;
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_MODULE_FLASHDATA_H
