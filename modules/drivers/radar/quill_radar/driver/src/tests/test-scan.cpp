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

#include "uhnder-common.h"
#include "gtest/gtest.h"

#include "scanning.h"
#include "scanobject.h"
#include "detections.h"
#include "rra.h"
#include "connection.h"
#include "serializer.h"
#include "userlogagents.h"
#include "pointcloud.h"

#include <stdlib.h>
#include <assert.h>
uint32_t radar_ip;
Connection* con;
RDC_ThresholdControl thresh;
RDC_ScanControl scanctrl;
UhdpCaptureControl cap_ctrl;
SessionFolderScanSerializer* folder_serializer;
FileLogger* logger;
RDC_ScanDescriptor desc;

TEST (scan, scanTester)
{
    inet_pton(AF_INET, "127.0.0.1", &radar_ip);
    static const float point_cloud_thresh_snr_dB = 27.01f;
    folder_serializer = new SessionFolderScanSerializer("../src/tests/session/");
    logger = new FileLogger("../src/tests/session/");
    if (!logger->is_ok())
    {
        printf("%s/ is not writeable\n", "../src/tests/session/");
        delete folder_serializer;
        delete logger;
    }
    cap_ctrl.set_defaults();
    cap_ctrl.capture_mode = DL_NORMAL;
    cap_ctrl.enable_mask  = DL_POINT_CLOUD;
    cap_ctrl.enable_mask |= DL_CI;
    cap_ctrl.enable_mask |= DL_RDC1;
    cap_ctrl.enable_mask |= DL_RDC2;
    //cap_ctrl.enable_mask |= DL_DET;

    scanctrl.defaults();
    scanctrl.scan_loop_count = 1;

    thresh.apply_preset(LOW_SENSITIVITY);

    thresh.point_cloud_thresh_snr_dB = point_cloud_thresh_snr_dB;
    // since static points are derived from the clutter image, we must use the
    // point cloud threshold for the clutter image as well
    thresh.clutter_image_thresh_snr_dB = point_cloud_thresh_snr_dB;

    // example threshold derived from rate of false alarm
    //thresh.point_cloud_thresh_snr_dB = desc.rfa_to_snr_dB(1.0, MAX_ROUGH_ANGLES);
    con = make_connection(radar_ip, 3, *logger);
    const char* get_rra_version_string();
    const char* get_compiled_srs_path_string();
    enum MakeConnectionError get_last_connection_error();
    const char* get_compiled_srs_version_string(uint32_t* revids);
    EXPECT_STREQ (get_rra_version_string(), "rra-v0.7.5-RC2+22-03f7c480a3a1");
    EXPECT_STREQ (get_compiled_srs_version_string(NULL), "srs-v0.7.5-RC2+30-043956927061");
    EXPECT_STREQ (get_compiled_srs_path_string(), "/home/ndeepak-lap/repos/radar-remote-api/system-radar-software");
    EXPECT_EQ (get_last_connection_error(), 0);
}

TEST (radar_connection, evaluateConnection){
    if (con)
    {
        con->configure_detection_thresholds(&thresh);
        if (con->is_sabine_b())
        {
            desc.set_defaults(VP105);
            printf("sabine_b detected....\n");
        }
        else
        {
            desc.set_defaults(VP1a);
            printf("sabine_a detected....\n");
            desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
        }
        EXPECT_EQ (con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D), (unsigned int)2);
        EXPECT_EQ (con->get_socket_descriptor(), 4);
        EXPECT_EQ (con->get_num_antenna_configs(), (unsigned int)27);
        EXPECT_STREQ (con->get_antenna_config_name(desc.antenna_config_id), "Wanquan 12-Tx Power-ON Azimuth   (96VRx)");
        EXPECT_EQ (con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D), (unsigned int)2);
        EXPECT_EQ (con->get_num_diags(), (unsigned int)26);
        uint32_t diag_id = 0;
        uint32_t test_id = 0;
        for(diag_id = 0; diag_id < con->get_num_diags(); diag_id++){
            EXPECT_STRNE (con->get_diag_name(diag_id), NULL);
            EXPECT_NE (con->get_num_tests_in_diag(diag_id), (unsigned int)0);
            for (test_id = 0; test_id < con->get_num_tests_in_diag(diag_id); test_id++){
                EXPECT_STRNE (con->get_diag_test_name(diag_id, test_id), NULL);
            }
        }
    }
}

TEST (radar_scanning, evaluateScanning){
    if (con){
        Scanning* s = con->setup_scanning(scanctrl, &desc, cap_ctrl);
        if (!s)
        {
            printf("Unable to start scanning\n");
        }
        printf("Running scans..\n");
        ScanObject* rscan;
        int i = 0;
        for (i=0; i<5; i++)
        {
            ScanObject* scan = s->wait_for_scan();
            EXPECT_NE(scan, (ScanObject*)NULL);
            //EXPECT_EQ(s->scan_available(), 0);
            if (scan)
            {
                EXPECT_EQ(s->get_last_error(), 0);
                EXPECT_NE(scan->get_clutter_image(), (ClutterImage*)NULL);
                EXPECT_NE(scan->get_point_cloud(), (PointCloud*)NULL);
                EXPECT_EQ(scan->get_detections(), (Detections*)NULL);
                EXPECT_EQ(scan->get_tracks(), (Tracks*)NULL);
                EXPECT_EQ(scan->get_static_slice(), (StaticSlice*)NULL);
                EXPECT_EQ(scan->get_zero_doppler_rdc2(), (ZeroDoppler*)NULL);
                EXPECT_EQ(scan->get_dynamic_activations(), (Activations*)NULL);
                EXPECT_EQ(scan->get_music_data(), (MUSICData*)NULL);
                EXPECT_NE(scan->get_rdc1(), (RadarDataCube1*)NULL);
                EXPECT_EQ(scan->get_adc(), (ADCCaptureData*)NULL);
                EXPECT_NE(scan->get_scan_thresholds(), (RDC_ThresholdControl*)NULL);
                EXPECT_STREQ(scan->get_radar_software_version(), "srs-v0.7.5-RC2+30-043956927061");
                EXPECT_STREQ(scan->get_module_name(), "");
                EXPECT_STREQ(scan->get_module_type_name(), "Jujitsu");
                EXPECT_STREQ(scan->get_motherboard_type_name(), "PIB89 1.1");
                EXPECT_STREQ(scan->get_antennaboard_type_name(), "Sanya 1.0");
                EXPECT_STREQ(scan->get_antenna_module_type_name(), "Wanquan 1.0");
                scan->serialize(*folder_serializer);
                rscan = ScanObject::deserialize(*folder_serializer, i);
                EXPECT_NE(rscan, (ScanObject*)NULL);
                EXPECT_EQ(rscan->get_scan_info().scan_sequence_number, (unsigned int)i);
                scan->release();
            }
            else
            {
                break;
            }
        }
        s->release();
        con->close_and_release();
    }
    delete folder_serializer;
    delete logger;
}
