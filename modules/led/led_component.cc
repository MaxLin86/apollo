 
#include "modules/led/led_component.h"
#include "modules/common/util/message_util.h"

#include "cyber/common/log.h" 
// #include <iostream> 
#include <string>

namespace apollo {
namespace led {
  
bool ledComponent::Init() { 

  if (!GetProtoConfig(&led_conf_)) {
    AERROR << "Unable to load led conf file: " << ConfigFilePath();
    return false;
  }
  serial_ = std::make_shared<serial>(); 
  if(!serial_->init(led_conf_.dev_serial_name())){
    return false;
  }
  // ledmsg_read_ = node_->CreateReader<ledmsg>();
  has_change_ = true;
  WordInit();
  node_->CreateReader<ledmsg>("apollo/led",
                              [this](const std::shared_ptr<ledmsg>& led_msg) {
                                this->OnLedmsg(led_msg);
                              });
  return true;
}

void ledComponent::WordInit(){
  // std::vector<uint8_t> word;
  // word.push_back(0xD6);
  // word.push_back(0xD0);  //中
  // word.push_back(0xBF);
  // word.push_back(0xC6);  //科
  // word.push_back(0xD4);
  // word.push_back(0xC6);  //云
  // word.push_back(0xC9);
  // word.push_back(0xBC);  //杉
  // word.push_back(0xD0);
  // word.push_back(0xC5);  //信
  // word.push_back(0xCF);
  // word.push_back(0xA2);  //息
  // word.push_back(0xBC);
  // word.push_back(0xBC);  //技
  // word.push_back(0xCA);
  // word.push_back(0xF5);  //术

  // // ledmsg::word sss = ledmsg::word::ledmsg_word_WORD_ZKYSXXJS;
  // // int a = sss;
  // // a=ledmsg::word::ledmsg_word_WORD_ZKYSXXJS;
  // // if (a == ledmsg::word::ledmsg_word_WORD_ZKYSXXJS) {
  // //   return;
  // // }
  // word_list_.emplace(ledmsg::word::ledmsg_word_WORD_ZKYSXXJS, word);

  // word.push_back(0xD0);  //中
  // word.push_back(0xBF);
  // word.push_back(0xC6);  //科
  // word.push_back(0xD4);
  // word.push_back(0xC6);  //云
  // word.push_back(0xC9);
  // word.push_back(0xBC);  //杉
  // word_list_.emplace(ledmsg::word::ledmsg_word_WORD_ZKYS, word);
  // // word.swap(std::vector<uint8_t>());
  // word.clear();
  // word.push_back(0xD6);
  // word.push_back(0xD0);  //中
  // word.push_back(0xBF);
  // word.push_back(0xC6);  //科
  // word.push_back(0xD4);
  // word.push_back(0xC6);  //云
  // word.push_back(0xC9);
  // word.push_back(0xBC);  //杉
  // word_list_.emplace(ledmsg::word::ledmsg_word_WORD_ZKYS, word);


}

void ledComponent::OnLedmsg(const std::shared_ptr<ledmsg> &led_msg){
  // if(!led_msg->has_wordid()){
  //   return;
  // }
  // wordid_ = led_msg->wordid(); 
  if(!led_msg->has_word()){
  // AERROR << "!led_msg->has_word";
    return;
  } 
  std::string tmp=led_msg->word();
  {
   
    std::lock_guard<std::mutex> lock(mutex_);
    if(word_ != tmp){ 
      word_ = tmp;
  // AERROR << "has_change_ = false";
      has_change_ = false;
    }
  }
}

bool ledComponent::Proc() {
  if (has_change_) return true;

     LEDCmd cmd; 
    //  auto it = word_list_.find(wordid_);
    //  if(it == word_list_.end()){
    //    return true;
    //  }
    //  cmd.word = it->second;
    { 
     std::lock_guard<std::mutex> lock(mutex_);
     cmd.word = word_; 
  // AERROR << "cmd.word = word_";
    }
     serial_->SetParam(0x03, cmd);
     if(!serial_->SetLEDCmd(cmd)){
  // AERROR << "SetLEDCmd false";
       return false;
     } 
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    if (!serial_->IsSuccess(cmd.id)){
      AERROR << "led feedback error";
      return false;
    }
    has_change_ = true;//成功显示最新内容 
  // AERROR << "return true";
    return true;
}


}  // namespace led
}  // namespace apollo
