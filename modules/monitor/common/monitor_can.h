/*
 * @Author: your name
 * @Date: 2020-04-15 09:22:39
 * @LastEditTime: 2020-04-22 18:20:40
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /apollo-5.5.0/modules/monitor/common/monitor_can.h
 */

//  #include "modules/drivers/canbus/can_comm/can_sender.h"
 

// // #include "modules/canbus/proto/chassis_detail.pb.h"
// #include "modules/common/proto/error_code.pb.h"
// #include "modules/drivers/canbus/can_client/fake/fake_can_client.h"
// #include "modules/drivers/canbus/can_comm/protocol_data.h"

#pragma once

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h" 
#include <vector> 
#include "modules/drivers/canbus/proto/can_card_parameter.pb.h"
namespace apollo {
namespace monitor {
namespace common {
    enum FaultLevel{
        ZERO = 0,
        ONE = 1,
        TWO = 2,
        THREE = 3,
    };
    
    typedef struct fault_info{
        FaultLevel ADU;
        FaultLevel VCU;
    }fault_into_t;

    // template <typename Type>
    using apollo::common::ErrorCode;
    class monitor_can
    {
    public:
        static monitor_can *mc(){
            static monitor_can instance;
            return &instance;
        }
        // monitor_can<Type> &mc(){
        //     static monitor_can<Type> instance;
        //     return instance;
        // }
        // auto mc()-> std::shared_ptr<Reader<MessageT>> { 
        //     static monitor_can<Type> instance;
        //     return instance;
        // }
        // auto mc(const uint32_t message_id)-> std::shared_ptr<monitor_can<Type>>{ 
        //     // static 
        //     // monitor_can<Type> instance;
        //     // return instance;
        //     static auto pMc =  std::make_shared<monitor_can<Type>>(1);
        //     return std::make_shared<monitor_can<Type>>(message_id);
        // }

        
        // void send_msg(uint32_t message_id, drivers::canbus::ProtocolData<Type> *protocol_data){

        // }
        
        // bool send_msg(const drivers::canbus::SenderMessage<Type> &msg){ 
        //     if(!sender_.NeedSend(msg,message_id_)){
        //         AERROR << "NeedSend error";
        //         return false;
        //     }
        // }
        bool init(){ 
            AERROR<<"bkx:monitor_can::init";
            drivers::canbus::CANCardParameter param;
            param.set_brand(drivers::canbus::CANCardParameter::SOCKET_CAN_RAW);
            param.set_channel_id(drivers::canbus::CANCardParameter::CHANNEL_ID_ZERO);
            param.set_interface(drivers::canbus::CANCardParameter::VIRTUAL);

            if(!socket_can_client_.Init(param)){
                AERROR<<"socket can init failed.";
                return false;
            }
            if(socket_can_client_.Start() == ErrorCode::CAN_CLIENT_ERROR_BASE){ 
                AERROR<<"socket can ErrorCode::CAN_CLIENT_ERROR_BASE";
                return false;
            }
            SendFault(FaultLevel::ONE); 
            return true;
        }
        void stop(){ 
            socket_can_client_.Stop();
            AERROR<<"socket_can_client.Stop();";
        }
        bool Receive(const uint32_t message_id,drivers::canbus::CanFrame &can_frame){  
            AERROR<<"socket_can_client_.Receive";          
            std::vector<drivers::canbus::CanFrame> rframes;
            int num = 1;
            if(socket_can_client_.Receive(&rframes, &num) == ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED){
                    AERROR<<"socket can ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED";
                    return false; 
            }
            AERROR<<"rfames.size()"<<rframes.size();
            for (auto cf : rframes)
            {
                AERROR<<"bkx:cf.id"<<cf.id<<";cf.len="<<cf.len<<"cf.data"<<cf.data;
                /* code */
            }
            AERROR<<"socket_can_client_.Receive end";          

        }
        bool SendFault(FaultLevel fl){ 
            AERROR<<"bkx:monitor_can::init";
           // std::vector<drivers::canbus::CanFrame> frames;
            drivers::canbus::CanFrame cf;
            cf.id = 0x0CFF1400;
            if(fl == FaultLevel::ONE){ 
                // cf.data[0] = '1';
                cf.data[0] = 0x01;
                cf.data[1] = 0x00;
                cf.data[2] = 0x00;
                cf.data[3] = 0x00;
                cf.data[4] = 0x65;
                cf.data[5] = 0x00;
                cf.data[6] = 0x00;
                cf.data[7] = 0x00;
                //0x01 0x00 0x00 0x00 0x65 0x00 0x00 0x00
            }else if(fl == FaultLevel::TWO || fl == FaultLevel::THREE){
                cf.data[0] = 0x44; 
            }else{
                AERROR<<"no such fault level";
                return false;
            }
        
           std::vector<drivers::canbus::CanFrame> can_frame;
            // drivers::canbus::CanFrame can_frame;
            can_frame.push_back(cf);
            AERROR <<"socket_can_client_.SendSingleFrame:"<<222<<cf.data[0];
            if(socket_can_client_.SendSingleFrame(can_frame) == ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED){ 
                    AERROR<<"socket can ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED";
                    return false; 
            }
            // frames.emplace_back(cf); 
            // int32_t num = 1;
            // if(socket_can_client_.Send(frames, &num) == ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED){
            //         AERROR<<"socket can ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED";
            //         return false;

            // }
            // AERROR<<"bkx:monitor_can::init";
            // std::vector<drivers::canbus::CanFrame> rframes;
            // if(socket_can_client_.Receive(&rframes, &num) == ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED){
            //         AERROR<<"socket can ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED";
            //         return false; 
            // }
            // AERROR<<"rfames.size()"<<frames.size();
            // for (auto cf : rframes)
            // {
            //     AERROR<<"bkx:cf.id"<<cf.id<<";cf.len="<<cf.len<<"cf.data"<<cf.data;
            //     /* code */
            // }
            
            return true;
/////////////////////////////////////////////////////////////////////////////////////////
            // sender_.Init(&can_client_, true);
            // printf("bkx:monitor_can");

            
            // printf("sm_.message_id=%d,sm_.curr_period()=%d,sm_.CanFrame().id=%d\n",sm_.message_id(),sm_.curr_period(),sm_.CanFrame());
            // int32_t period = sm_.curr_period();
            // msg.UpdateCurrPeriod(-50);
            // EXPECT_FALSE(sender.NeedSend(msg, 1));
            // EXPECT_EQ(msg.message_id(), 1);
            // int32_t period = msg.curr_period();
            // msg.UpdateCurrPeriod(-50);
            // EXPECT_EQ(msg.curr_period(), period + 50);
            // EXPECT_EQ(msg.CanFrame().id, 1);

            // sender.AddMessage(1, &mpd);
            // int a=22;
            // printf("bkx..sender.AddMessadddddgde%d",a);
            // EXPECT_EQ(sender.Start(), common::ErrorCode::OK);
            // EXPECT_TRUE(sender.IsRunning());
            // EXPECT_TRUE(sender.enable_log());

            // sender.Update();
            // sender.Stop();
            // EXPECT_FALSE(sender.IsRunning());
        }
        /* data */
    public:
    private:
        // monitor_can(const uint32_t message_id)
        //     :message_id_(message_id) {

        //     }
        monitor_can(){ }

        ~monitor_can(){} 
    private:
        drivers::canbus::can::SocketCanClientRaw socket_can_client_;
                   //  const uint32_t message_id_;
        // drivers::canbus::CanSender<Type> sender_;
        // drivers::canbus::can::FakeCanClient can_client_;
        // drivers::canbus::ProtocolData<Type> mpd_;
        // drivers::canbus::SenderMessage<Type> sm_(message_id_, &mpd_);
    };

    // monitor_can::monitor_can(/* args */)
    // {
    // }

    // monitor_can::~monitor_can()
    // {
    // }
}
}
}