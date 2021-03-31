#include "modules/perception/inference/darknet/darknet_api.h"

#include <algorithm>
#include <utility>

// #include "absl/strings/str_cat.h"
#include "cyber/common/log.h"


namespace apollo {
namespace perception {
namespace inference {

DarkNet::DarkNet(const std::string &net_file, const std::string &model_file, const std::vector<std::string> &outputs, const std::vector<std::string> &inputs)
{
  int a = 1;
  AERROR << a;
}


bool DarkNet::Init(const std::map<std::string, std::vector<int>> &shapes) {

  return true;
}

void DarkNet::Infer(){
  int a = 1;
  AERROR << a;
}

std::shared_ptr<apollo::perception::base::Blob<float>> DarkNet::get_blob(const std::string &name) {
  auto iter = blobs_.find(name);
  if (iter == blobs_.end()) {
    return nullptr;
  }
  return iter->second;
}

}
}
}