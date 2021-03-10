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

#include "modules/drivers/radar/quill_radar/driver/system-radar-software/env-uhnder/coredefs/uhnder-common.h"
#include "gtest/gtest.h"

#include "modules/drivers/radar/quill_radar/driver/include/scanning.h"
#include "modules/drivers/radar/quill_radar/driver/include/scanobject.h"
#include "modules/drivers/radar/quill_radar/driver/include/detections.h"
#include "modules/drivers/radar/quill_radar/driver/include/serializer.h"

#include <stdlib.h>
#include <assert.h>

static const char* test_source_path = __FILE__;

TEST(serialize, memtest)
{
    /* derive location of session/ folder in the same location as this
     * source file */
    char* test_folder = strdup(test_source_path);
    char* last_slash = strrchr(test_folder, '/'); assert(last_slash);
    strcpy(last_slash + 1, "session/");
    // relies on 'session' being shorter than the test filename
    assert(strlen(test_folder) >= strlen(test_folder));

    SessionFolderScanSerializer s(test_folder);
    free(test_folder);

    // number of detections in the canned scans
    static uint32_t detcount[] = { 55, 57, 60, 58, 56 };

    ScanObject* scan = ScanObject::deserialize(s, 0);
    EXPECT_NE(scan, (ScanObject*)NULL);
    if (scan)
    {
        EXPECT_NE(scan->get_clutter_image(), (ClutterImage*)NULL);
        EXPECT_NE(scan->get_static_slice(), (StaticSlice*)NULL);
        EXPECT_NE(scan->get_detections(), (Detections*)NULL);
        EXPECT_NE(scan->get_zero_doppler_rdc2(), (ZeroDoppler*)NULL);

        Detections* dets = scan->get_detections();
        if (dets)
        {
            EXPECT_EQ(dets->get_count(), detcount[0]);
        }
    }

    char* buf = new char[1024 * 1024 * 4];
    MemorySerializer ms(buf, 1024 * 1024 * 2);

    // hack so we can change the scan sequence number and pretend we have many
    // scans
    UhdpScanInformation& info = const_cast<UhdpScanInformation&>(scan->get_scan_info());
    for (uint32_t i = 0; i < 4; i++)
    {
        info.scan_sequence_number = i;
        scan->serialize(ms);
    }

    for (uint32_t i = 0; i < 3; i++)
    {
        ScanObject* rscan = ScanObject::deserialize(ms, i);
        EXPECT_NE(rscan, (ScanObject*)NULL);
        if (rscan)
        {
            EXPECT_NE(rscan->get_clutter_image(), (ClutterImage*)NULL);
            EXPECT_NE(rscan->get_static_slice(), (StaticSlice*)NULL);
            EXPECT_NE(rscan->get_detections(), (Detections*)NULL);
            EXPECT_NE(rscan->get_zero_doppler_rdc2(), (ZeroDoppler*)NULL);
        }
    }

    // there should be room to serialize scan 4 now
    info.scan_sequence_number = 4;
    scan->serialize(ms);

    // the fourth serialize call was expected to fail, the buffer is only big
    // enough for three scans
    ScanObject* rscan = ScanObject::deserialize(ms, 3);
    EXPECT_EQ(rscan, (ScanObject*)NULL);

    rscan = ScanObject::deserialize(ms, 4);
    EXPECT_NE(rscan, (ScanObject*)NULL);
    if (rscan)
    {
        EXPECT_NE(rscan->get_clutter_image(), (ClutterImage*)NULL);
        EXPECT_NE(rscan->get_static_slice(), (StaticSlice*)NULL);
        EXPECT_NE(rscan->get_detections(), (Detections*)NULL);
        EXPECT_NE(rscan->get_zero_doppler_rdc2(), (ZeroDoppler*)NULL);
    }

    // steady state operation, many scans with many wraps (wrap scan table)
    for (uint32_t i = 5; i < 64; i++)
    {
        info.scan_sequence_number = i;
        scan->serialize(ms);

        ScanObject* rscan = ScanObject::deserialize(ms, i);
        EXPECT_EQ(rscan->get_scan_info().scan_sequence_number, i);
        EXPECT_NE(rscan, (ScanObject*)NULL);
        if (rscan)
        {
            EXPECT_NE(rscan->get_clutter_image(), (ClutterImage*)NULL);
            EXPECT_NE(rscan->get_static_slice(), (StaticSlice*)NULL);
            EXPECT_NE(rscan->get_detections(), (Detections*)NULL);
            EXPECT_NE(rscan->get_zero_doppler_rdc2(), (ZeroDoppler*)NULL);
        }
    }

    // simulate reconnect
    info.scan_sequence_number = 0;
    scan->serialize(ms);

    uint32_t seq = 1;
    EXPECT_EQ(ms.get_next_read_scan_sequence(seq), true);
    EXPECT_EQ(seq, 0U);
    rscan = ScanObject::deserialize(ms, seq);
    EXPECT_EQ(rscan->get_scan_info().scan_sequence_number, seq);

    delete [] buf;
}
