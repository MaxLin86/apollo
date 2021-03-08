#ifndef SRS_HDR_MODULE_FLASHDATA_H
#define SRS_HDR_MODULE_FLASHDATA_H 1

#include "modules/drivers/radar/uhnder_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"

SRS_DECLARE_NAMESPACE()

//! This structure defines the contents of the 'network' file on the flash
//! filesystem. Since it deals with IP networking and other operating-system
//! level details, it is considered an environment structure.

struct ModuleFlashData
{
    static const uint32_t MAGIC_VALUE = 0xD00DBABEUL;

    enum { FILE_VERSION = 12 };

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

    //! various string options which can modify the behavior of engine or
    //! environment code (similar to Linux kernel boot options)
    CHAR     boot_config[128];

    uint32_t boot_mode;                 //!< enum RHAL_BootMode

    float    rear_axle_dist_m;          //!< Dist to rear axle
    float    center_line_dist_m;        //!< Dist from centerline
    float    sensor_rotation_az_deg;    //!< Degrees off boresight in azimuth
    float    sensor_rotation_el_deg;    //!< Degrees off boresight in elevation
    float    sensor_height_m;           //!< Height of radar

    uint32_t ext_lo_enable_flags;       //!< enum RHAL_LOMode

    FLOAT    force_pll_freq;            //!< Force PLL to lock at given freq

    INT      adc_ldo_code;

    int8_t  txu_dll_phase;             //!< TXU DLL Phase 
    uint8_t eth_speed;                 //!< Advertised Ethernet speed - 0 - 10Mbps , 1-100Mbps, 2-1Gbps
    uint8_t reserved1;
    uint8_t reserved2;

    void set_defaults()
    {
        memset(this, 0, sizeof(*this));

        magic = MAGIC_VALUE;

        version = FILE_VERSION;

#if PALLADIUM_EMULATION
        radar_ip_address = 0x7f000001UL; // 127.0.0.1

        const uint8_t factory_default_mac[] =
        {
            0xA0U, 0x01U, 0x02U, 0x03U, 0x04U, 0x05U
        };
#else
        radar_ip_address = 0xC0A85F01UL; // 192.168.95.1

        const uint8_t factory_default_mac[] =
        {   // we are using the prefix 0x70B3D55FULB and range 0x000U to 0xFFFU
            0x70U, 0xB3U, 0xD5U, 0x5FU, 0xBFU, 0xFFU
        };
#endif

        uh_memcpy(radar_ethernet_address, factory_default_mac, sizeof(factory_default_mac));

#if __BARE_METAL__
        strcpy(radar_hostname, "no factory cal");
        ext_lo_enable_flags = 0;
#else
        strcpy(radar_hostname, "localhost");
        ext_lo_enable_flags = 1;
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
        eth_speed = 2; //Advertise 1G by default
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
                eth_speed               = def.eth_speed;
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
};

SRS_CLOSE_NAMESPACE()

#endif // SRS_HDR_MODULE_FLASHDATA_H
