/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "modules/localization/foo/foo_localization.h"

#include "google/protobuf/text_format.h"
#include "gtest/gtest.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/util.h"
#include "modules/localization/msf/common/util/frame_transform.h"

namespace apollo {
namespace localization {

class FooLocalizationTest : public ::testing::Test {
 public:
  virtual void SetUp() { foo_localizatoin_.reset(new FooLocalization()); }

 protected:
  template <class T>
  void load_data(const std::string &filename, T *data) {
    CHECK(cyber::common::GetProtoFromFile(filename, data))
        << "Failed to open file " << filename;
  }


  template <class Type>
  Type stringToNum(const std::string& str)
  {
  std::istringstream iss(str);
  Type num;
  iss >> num;
  return num;
  }
  //删除字符串中空格，制表符tab等无效字符
  std::string Trim(std::string& str)
  {
    //str.find_first_not_of(" \t\r\n"),在字符串str中从索引0开始，返回首次不匹配"\t\r\n"的位置
    str.erase(0,str.find_first_not_of(" \t\r\n"));
    str.erase(str.find_last_not_of(" \t\r\n") + 1);
    return str;
  }

  std::unique_ptr<FooLocalization> foo_localizatoin_;
};

 
TEST_F(FooLocalizationTest, GPStoUTM) { 

  std::ofstream guidepost_utm_out;
  guidepost_utm_out.open("/apollo/data/guidepost_utm_out.txt",std::ios::app);

  static int row = 0;

  std::ifstream fin("/apollo/data/new_csv_090914.csv"); //打开文件流操作
	std::string line; 
	while (getline(fin, line))   //整行读取，换行符“\n”区分，遇到文件尾标志eof终止读取
	{
    row ++;
    if (row <=1) {continue;}
		std::cout <<"原始字符串："<< line << std::endl; //整行输出
    
	  std::istringstream sin(line); //将整行字符串line读入到字符串流istringstream中
		std::vector<std::string> fields; //声明一个字符串向量
		std::string field;
		while (getline(sin, field, ',')) //将字符串流sin中的字符读入到field字符串中，以逗号为分隔符
		{
			fields.push_back(field); //将刚刚读取的字符串添加到向量fields中
		}
		std::string seq_number = Trim(fields[0]); //清除掉向量fields中第一个元素的无效字符，并赋值给变量seq
		std::string str_lat =Trim(fields[5]); //清除掉向量fields中第二个元素的无效字符，并赋值给变量lat
		std::string str_lon =Trim(fields[6]); //清除掉向量fields中第三个元素的无效字符，并赋值给变量lon
		//std::cout <<"处理之后的字符串："<< seq_number << "\t" << lat << "\t" << lon << std::endl; 
    double lat = stringToNum<double>(str_lat);
    double lon = stringToNum<double>(str_lon);
    std::cout <<std::fixed << "处理之后的数值："<< seq_number << "\t" << lat << "\t" << lon << std::endl; 

    msf::UTMCoor utm_xy;
    msf::FrameTransform::LatlonToUtmXY(lon/180.0*M_PI,
                                  lat/180.0*M_PI, &utm_xy);
    std::cout << "utm: " << utm_xy.x << "\t" << utm_xy.y << std::endl;

    guidepost_utm_out << std::fixed << std::atoi(seq_number.c_str()) << "," << utm_xy.x << "," << utm_xy.y << std::endl;

	}
  guidepost_utm_out.close();
}



}  // namespace localization
}  // namespace apollo
