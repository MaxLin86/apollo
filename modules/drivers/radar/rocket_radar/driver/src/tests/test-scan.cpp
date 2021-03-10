// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "gtest/gtest.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "modules/drivers/radar/rocket_radar/driver/include/rra.h"
#include "scanning.h"
#include "scanobject.h"
#include "modules/drivers/radar/rocket_radar/driver/include/detections.h"
#include "pointcloud.h"
#include "connection.h"
#include "modules/drivers/radar/rocket_radar/driver/include/serializer.h"
#include "userlogagents.h"

TEST (radar_scanning, evaluateScanning)
{
    FileLogger* logger = new FileLogger("session/");
    EXPECT_EQ(logger->is_ok(), true);
    if (!logger->is_ok())
    {
        delete logger;
        return;
    }

    uint32_t radar_ip;
    inet_pton(AF_INET, "127.0.0.1", &radar_ip);
    Connection* con = make_connection(radar_ip, 3, *logger);
    EXPECT_NE(con, (Connection*)NULL);
    if (con)
    {
        EXPECT_EQ(con->is_sabine_b(), true);

        EXPECT_EQ(con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D), 2U);
        EXPECT_EQ(con->get_num_antenna_configs(), 28U);
        for (uint32_t diag_id = 0; diag_id < con->get_num_diags(); diag_id++)
        {
            EXPECT_STRNE(con->get_diag_name(diag_id), NULL);
            EXPECT_NE(con->get_num_tests_in_diag(diag_id), 0U);
            for (uint32_t test_id = 0; test_id < con->get_num_tests_in_diag(diag_id); test_id++)
            {
                EXPECT_STRNE(con->get_diag_test_name(diag_id, test_id), NULL);
            }
        }

        RDC_ThresholdControl thresh;
        thresh.apply_preset(LOW_SENSITIVITY);
        con->configure_detection_thresholds(&thresh);

        RDC_ScanDescriptor desc;
        desc.set_defaults(VP105);
        desc.antenna_config_id = con->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);

        SessionFolderScanSerializer* folder_serializer;
        folder_serializer = new SessionFolderScanSerializer("session/");

        UhdpCaptureControl cap_ctrl;
        cap_ctrl.set_defaults();
        cap_ctrl.capture_mode = DL_NORMAL;
        cap_ctrl.enable_mask  = DL_POINT_CLOUD | DL_CI | DL_DET;

        RDC_ScanControl scanctrl;
        scanctrl.defaults();
        scanctrl.scan_loop_count = 1;

        Scanning* s = con->setup_scanning(scanctrl, &desc, cap_ctrl);
        EXPECT_NE(s, (Scanning*)NULL);
        if (s)
        {
            for (uint32_t i = 0; i < 5; i++)
            {
                ScanObject* scan = s->wait_for_scan();
                EXPECT_NE(scan, (ScanObject*)NULL);
                EXPECT_EQ(s->get_last_error(), 0);

                if (scan)
                {
                    EXPECT_NE(scan->get_clutter_image(), (ClutterImage*)NULL);
                    EXPECT_NE(scan->get_point_cloud(), (PointCloud*)NULL);
                    EXPECT_NE(scan->get_detections(), (Detections*)NULL);
                    EXPECT_NE(scan->get_scan_thresholds(), (RDC_ThresholdControl*)NULL);

                    EXPECT_STREQ(scan->get_module_type_name(), "Jujitsu");
                    EXPECT_STREQ(scan->get_motherboard_type_name(), "PIB89 1.1");
                    EXPECT_STREQ(scan->get_antennaboard_type_name(), "Sanya 1.0");
                    EXPECT_STREQ(scan->get_antenna_module_type_name(), "Wanquan 1.0");

                    scan->serialize(*folder_serializer);

                    ScanObject* rscan = ScanObject::deserialize(*folder_serializer, i);
                    EXPECT_NE(rscan, (ScanObject*)NULL);
                    if (rscan)
                    {
                        EXPECT_EQ(rscan->get_scan_info().scan_sequence_number, i);
                        rscan->release();
                    }
                    scan->release();
                }
                else
                {
                    break;
                }
            }

            s->release();
        }

        scanctrl.defaults();
        scanctrl.scan_loop_count = 0;
        con->send_scan_control(scanctrl, NULL);

        delete folder_serializer;

        con->close_and_release();
    }

    delete logger;
}
