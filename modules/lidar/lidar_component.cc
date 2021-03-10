 
#include "modules/lidar/lidar_component.h"
#include "modules/common/util/message_util.h"

#include "cyber/common/log.h" 
// #include <iostream> 


namespace apollo {
namespace lidar {
  
bool LidarComponent::Init() { 

led_writer_ =node_->CreateWriter<led::ledmsg>("apollo/led");;
return true;

  if (!GetProtoConfig(&lidar_conf_)) {
    AERROR << "Unable to load lidar conf file: " << ConfigFilePath();
    return false;
  } 
  benewake_ = std::make_shared<benewake>();
  benewake_->init(lidar_conf_.dev_serial_name()); 
  BenewakeLidar_writer_ = node_->CreateWriter<lidar::BenewakeLidar>("apollo/lidar");
  return true;
}

bool LidarComponent::Proc() 
{ 

led::ledmsg led_msg;

    common::util::FillHeader( node_->Name(), &led_msg );
    // led_msg.set_word(led::ledmsg::WORD_ZKYSXXJS);
    led_msg.set_word("zkysNO1");
    led_writer_->Write(led_msg);
    AERROR << "Write(led_msg)="<<led_msg.word();

    return true;
    
        lidar::BenewakeLidar benewake_msg;

        if(!benewake_->GetLiarData(benewake_msg))
        {
                AERROR << "Get lidar data failed!";
                return false;
        } 

	AERROR << "distance = " << std::fixed << benewake_msg.distance() << "  temp = " << std::fixed << benewake_msg.temperature();

        if ( ( benewake_msg.distance() > 10.0 ) && ( benewake_msg.distance() < 1200.0 ) )
        {
                nWithInfoCount_++;

                if( nWithInfoCount_ >= 5 )
                {
                        nWithInfoCount_ = 5;
                        bWithInfo_ = true;
                }
        }
        else
        {
                nWithInfoCount_--;

                if( nWithInfoCount_ <= 0)
                {
                        nWithInfoCount_ = 0;
                        bWithInfo_ = false;
                }
                else
                {
                        benewake_msg.set_distance( info_lidar_.distance() );
                        benewake_msg.set_strength( info_lidar_.strength() );
                        benewake_msg.set_temperature( info_lidar_.temperature() );
                }
        }

        if( bWithInfo_ )
        {
                common::util::FillHeader( node_->Name(), &info_lidar_ );
                info_lidar_.set_distance( benewake_msg.distance() );
                info_lidar_.set_strength( benewake_msg.strength() );
                info_lidar_.set_temperature( benewake_msg.temperature() );
        }
        else
        {
                common::util::FillHeader( node_->Name(), &info_lidar_ );
                info_lidar_.set_distance( 0.0 );
                info_lidar_.set_strength( 0.0 );
                info_lidar_.set_temperature( 0.0 );
        }

        BenewakeLidar_writer_->Write( info_lidar_ );

        return true;
}
 

}  // namespace lidar
}  // namespace apollo
