/******************************************************************************
 * Copyright 2021 MaxLin. All Rights Reserved.
 *****************************************************************************/

#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "modules/perception/inference/inference.h"


namespace apollo {
namespace perception {
namespace inference {

typedef std::shared_ptr<apollo::perception::base::Blob<float>> BlobPtr;

class DarkNet : public Inference {
 public:
  DarkNet(const std::string &net_file, const std::string &model_file,
        const std::vector<std::string> &outputs,
        const std::vector<std::string> &inputs);

  virtual ~DarkNet() {}
  bool Init(const std::map<std::string, std::vector<int>> &shapes) override;
  void Infer() override;
  BlobPtr get_blob(const std::string &name) override;

 private:
  std::string net_file_;
  std::string model_file_;
  std::vector<std::string> output_names_;
  std::vector<std::string> input_names_;
  BlobMap blobs_;
};

}  // namespace inference
}  // namespace perception
}  // namespace apollo