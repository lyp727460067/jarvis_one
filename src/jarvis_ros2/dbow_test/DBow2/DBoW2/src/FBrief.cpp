/**
 * File: FBrief.cpp
 * Date: November 2011
 * Author: Dorian Galvez-Lopez
 * Description: functions for BRIEF descriptors
 * License: see the LICENSE.txt file
 *
 */
 
#include <vector>
#include <string>
#include <sstream>
#include "glog/logging.h"
#include "FBrief.h"
using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FBrief::meanValue(const std::vector<FBrief::pDescriptor> &descriptors, 
  FBrief::TDescriptor &mean)
{
  mean.reset();
  
  if(descriptors.empty()) return;
  
  const int N2 = descriptors.size() / 2;
  const int L = descriptors[0]->size();
  
  vector<int> counters(L, 0);

  vector<FBrief::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
    const FBrief::TDescriptor &desc = **it;
    for(int i = 0; i < L; ++i)
    {
      if(desc[i]) counters[i]++;
    }
  }
  
  for(int i = 0; i < L; ++i)
  {
    if(counters[i] > N2) mean.set(i);
  }
  
}

// --------------------------------------------------------------------------

double FBrief::distance(const FBrief::TDescriptor &a,
                        const FBrief::TDescriptor &b) {
  return static_cast<double>((a ^ b).count());
}

// --------------------------------------------------------------------------
  
std::string FBrief::toString(const FBrief::TDescriptor &a)
{
  // from boost::bitset
  return  a.to_string();
}

// --------------------------------------------------------------------------
  
void FBrief::fromString(FBrief::TDescriptor &a, const std::string &s)
{
  // from boost::bitset
  stringstream ss(s);
  std::vector<uint8_t> breif;
  for (int i = 0; i < 32; i++) {
    int temp;
    ss>>temp;
    breif.push_back(static_cast<uint8_t>(temp));
    // LOG(INFO) <<int(breif.back());
  }

  auto Uint8ToBitSet = [](const std::vector<uint8_t> &bytes) {
    std::bitset<256> result;
    int j = 0;
    for (const auto &byte : bytes) {
      for (int i = 0; i < 8; i++) {
        if (byte & (0x01 << i)) {
          result.set(j);
        } else {
          result.reset(j);
        }
        j++;
      }
    }
    return result;
  };

  a =Uint8ToBitSet(breif);
  // std::cout<<a<<std::endl;
  // CHECK(false);
}

// --------------------------------------------------------------------------

void FBrief::toMat32F(const std::vector<TDescriptor> &descriptors, 
  cv::Mat &mat)
{
  // if(descriptors.empty())
  // {
  //   mat.release();
  //   return;
  // }
  
  // const int N = descriptors.size();
  // const int L = descriptors[0].size();
  
  // mat.create(N, L, CV_32F);
  
  // for(int i = 0; i < N; ++i)
  // {
  //   const TDescriptor& desc = descriptors[i];
  //   float *p = mat.ptr<float>(i);
  //   for(int j = 0; j < L; ++j, ++p)
  //   {
  //     *p = (desc[j] ? 1 : 0);
  //   }
  // } 
}

// --------------------------------------------------------------------------

} // namespace DBoW2

