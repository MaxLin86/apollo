// Copyright (C) Uhnder, Inc. All rights reserved. Confidential and Proprietary - under NDA.
// Refer to SOFTWARE_LICENSE file for details
#include "gtest/gtest.h"

#include "modules/drivers/radar/rocket_radar/driver/system-radar-software/env-reference/coredefs/uhnder-common.h"
#include "rra.h"
#include "sensorgroup.h"
#include "connection.h"
#include "scanobject.h"
#include "modules/drivers/radar/rocket_radar/driver/include/detections.h"

#include "../sensorgroup_impl.h"

TEST(sg, createReplaySensorGroup)
{
    const bool replay_mode = true;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        sg->close_and_release();
    }
}

TEST(sg, createNormalSensorGroup)
{
    const bool replay_mode = false;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        sg->close_and_release();
    }
}

TEST(sg, addSensor)
{
    const bool replay_mode = true;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        ConnectionLogDesc desc;

        uint32_t radar_ip;
        radar_ip = 0; sg->add_sensor(radar_ip, desc);
        radar_ip = 1; sg->add_sensor(radar_ip, desc);
        radar_ip = 2; sg->add_sensor(radar_ip, desc);
        radar_ip = 3; sg->add_sensor(radar_ip, desc);
        EXPECT_EQ(sg->get_sensor_count(), 4U);
        // replay sensor groups do not make Connections
        EXPECT_EQ((Connection*)NULL, sg->get_sensor_connection(0));
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        EXPECT_EQ((Connection*)NULL, sg->get_sensor_connection(4));
        EXPECT_EQ(SensorGroup::SG_INPUT_OUT_OF_RANGE, sg->get_last_error());
        sg->clear_last_error();
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        sg->close_and_release();
    }
}

TEST(sg, linEgoVel)
{
    const bool replay_mode = true;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        SensorGroup_Impl* sgi = static_cast<SensorGroup_Impl*>(sg);
        assert(sgi);

        bool thread = false;
        vec3f_t ins_pos(0, 0, 0);
        uint32_t frame_interval = 100 * 1000;
        sg->initialize(thread, frame_interval, ins_pos);

        uint32_t radar_ip;
        ConnectionLogDesc desc;
        radar_ip = 0; sg->add_sensor(radar_ip, desc);
        EXPECT_EQ(sg->get_sensor_count(), 1U);

        vec3f_t pos(1, 0, 0); // forward radar
        float yaw = 0, pitch = 0, roll = 0;
        sg->set_sensor_position(0, pos, yaw, pitch, roll);

        vec3f_t lin_vel(2, 1, 3);
        vec3f_t ang_vel(0, 0, 0);
        uint32_t age_us = 0;
        sg->update_ego_velocity(lin_vel, ang_vel, age_us);
        EXPECT_EQ(SensorGroup::SG_REPLAY_GROUP_CANNOT_SCAN, sg->get_last_error());
        sg->clear_last_error();

        vec3f_t ego_vel = sgi->sensor[0].last_ego_velocity;
        EXPECT_FLOAT_EQ(ego_vel.x, -lin_vel.x);
        EXPECT_FLOAT_EQ(ego_vel.y, -lin_vel.y);
        EXPECT_FLOAT_EQ(ego_vel.z, -lin_vel.z);

        /* rotate the sensor 180deg around the X axis (upside down) */
        roll = 180;
        sg->set_sensor_position(0, pos, yaw, pitch, roll);
        sg->update_ego_velocity(lin_vel, ang_vel, age_us);

        ego_vel = sgi->sensor[0].last_ego_velocity;
        EXPECT_FLOAT_EQ(ego_vel.x, -lin_vel.x);
        EXPECT_FLOAT_EQ(ego_vel.y, lin_vel.y);
        EXPECT_FLOAT_EQ(ego_vel.z, lin_vel.z);

        /* rotate the sensor 180deg around the Z axis (backwards) */
        roll = 0; yaw = 180;
        sg->set_sensor_position(0, pos, yaw, pitch, roll);
        sg->update_ego_velocity(lin_vel, ang_vel, age_us);

        ego_vel = sgi->sensor[0].last_ego_velocity;
        EXPECT_FLOAT_EQ(ego_vel.x, lin_vel.x);
        EXPECT_FLOAT_EQ(ego_vel.y, lin_vel.y);
        EXPECT_FLOAT_EQ(ego_vel.z, -lin_vel.z);

        /* rotate the sensor 180deg around the Y axis */
        yaw = 0; pitch = 180;
        sg->set_sensor_position(0, pos, yaw, pitch, roll);
        sg->update_ego_velocity(lin_vel, ang_vel, age_us);

        ego_vel = sgi->sensor[0].last_ego_velocity;
        EXPECT_FLOAT_EQ(ego_vel.x, lin_vel.x);
        EXPECT_FLOAT_EQ(ego_vel.y, -lin_vel.y);
        EXPECT_FLOAT_EQ(ego_vel.z, lin_vel.z);

        sg->close_and_release();
    }
}

TEST(sg, angEgoVel)
{
    const bool replay_mode = true;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        const SensorGroup_Impl* sgi = static_cast<const SensorGroup_Impl*>(sg);
        assert(sgi);

        bool thread = false;
        vec3f_t ins_pos(0, 0, 0);
        uint32_t frame_interval = 100 * 1000;
        sg->initialize(thread, frame_interval, ins_pos);

        uint32_t radar_ip;
        ConnectionLogDesc desc;
        radar_ip = 0; sg->add_sensor(radar_ip, desc);
        EXPECT_EQ(sg->get_sensor_count(), 1U);

        vec3f_t sensor_pos(1, 0, 0); // forward radar
        float yaw = 0, pitch = 0, roll = 0;
        sg->set_sensor_position(0, sensor_pos, yaw, pitch, roll);

        vec3f_t lin_vel(2, 0, 3);
        vec3f_t ang_vel(0, 0, 1); // yaw rate 1 radian per sec
        uint32_t age_us = 0;
        sg->update_ego_velocity(lin_vel, ang_vel, age_us);
        EXPECT_EQ(SensorGroup::SG_REPLAY_GROUP_CANNOT_SCAN, sg->get_last_error());
        sg->clear_last_error();

        vec3f_t ego_vel = sgi->sensor[0].last_ego_velocity;
        vec3f_t expected = lin_vel + sensor_pos.cross(ang_vel);
        expected *= -1;
        EXPECT_FLOAT_EQ(ego_vel.x, expected.x);
        EXPECT_FLOAT_EQ(ego_vel.y, expected.y);
        EXPECT_FLOAT_EQ(ego_vel.z, expected.z);

        sg->close_and_release();
    }
}


TEST(sg, sensorPose)
{
    const bool replay_mode = false;
    SensorGroup* sg = SensorGroup::create_new_sensor_group(replay_mode);
    EXPECT_NE(sg, (SensorGroup*)NULL);
    if (sg)
    {
        bool thread = false;
        vec3f_t ins_pos(0, 0, 0);
        uint32_t frame_interval = 100 * 1000;
        sg->initialize(thread, frame_interval, ins_pos);
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());

        ConnectionLogDesc logdesc;
        uint32_t radar_ip;
        inet_pton(AF_INET, "127.0.0.1", &radar_ip);
        sg->add_sensor(radar_ip, logdesc);
        EXPECT_EQ(sg->get_sensor_count(), 1U);
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        EXPECT_NE((Connection*)NULL, sg->get_sensor_connection(0));

        RDC_ThresholdControl thresh;
        thresh.apply_preset(LOW_SENSITIVITY);
        sg->configure_detection_thresholds(&thresh, 1);

        UhdpCaptureControl cap_ctrl;
        cap_ctrl.set_defaults();
        cap_ctrl.enable_mask = DL_DET;
        sg->configure_data_capture(cap_ctrl);

        RDC_ScanDescriptor desc;
        desc.set_defaults(VP105);
        desc.antenna_config_id = sg->get_sensor_connection(0)->get_basic_antenna_config_id(BAC_TRADITIONAL_1D);
        sg->configure_radar_scans(0, 1, &desc);
        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());

        vec3f_t pos(1, 0, 0); // forward radar
        float yaw = 0, pitch = 0, roll = 0;
        sg->set_sensor_position(0, pos, yaw, pitch, roll);

        sg->start_scanning();

        FrameObject* frame1 = NULL;
        while (!frame1)
        {
            frame1 = sg->poll_completed_frame(100);
            if (frame1 && frame1->get_detection_count() == 0)
            {
                // discard early frame with no scan
                frame1->release();
                frame1 = NULL;
            }
            if (sg->get_last_error() != SensorGroup::SG_NO_ERROR)
            {
                break;
            }
        }

        roll = 180; // change the sensor orientation to be upside down
        sg->set_sensor_position(0, pos, yaw, pitch, roll);

        FrameObject* frame2 = NULL;
        while (!frame2)
        {
            frame2 = sg->poll_completed_frame(100);
            if (frame2 && frame2->get_detection_count() == 0)
            {
                // discard early frame with no scan
                frame2->release();
                frame2 = NULL;
            }
            if (sg->get_last_error() != SensorGroup::SG_NO_ERROR)
            {
                break;
            }
        }

        sg->stop_scanning();

        EXPECT_EQ(SensorGroup::SG_NO_ERROR, sg->get_last_error());
        EXPECT_NE(frame1, (FrameObject*)NULL);
        EXPECT_NE(frame2, (FrameObject*)NULL);
        if (frame1 && frame2)
        {
            EXPECT_EQ(frame1->get_detection_count(), frame2->get_detection_count());

            for (uint32_t d = 0; d < frame1->get_detection_count() && d < frame2->get_detection_count(); d++)
            {
                const FrameDetection& d1 = frame1->get_detection(d);
                const FrameDetection& d2 = frame2->get_detection(d);

                // d2 should be d1 as seen by an updside down sensor
                EXPECT_NEAR(d1.range,         d2.range, 0.1);
                EXPECT_NEAR(d1.azimuth,       d2.azimuth, 0.1);
                EXPECT_NEAR(d1.elevation,     d2.elevation, 0.1);
                EXPECT_NEAR(d1.position_m.x,  d2.position_m.x, 0.25);
                EXPECT_NEAR(d1.position_m.y, -d2.position_m.y, 0.25);
                EXPECT_NEAR(d1.position_m.z, -d2.position_m.z, 0.25);
            }

            frame1->release();
            frame2->release();
        }

        while (true)
        {
            FrameObject* frame = sg->poll_completed_frame(frame_interval);
            if (frame)
            {
                frame->release();
            }
            else
            {
                break;
            }
        }

        sg->close_and_release();
    }
}
