#include "jarvis/mapping/dbow/vocabulary.h"
//
#include "gtest/gtest.h"
#include "jarvis/common/port.h"
#include "jarvis/mapping/match/des_matcher.h"
#include "random"
#include "to_dbow2_vocabulary_adptor.h"
//
namespace jarvis_ros2 {
using namespace jarvis;
using namespace mapping;

namespace {
BrifBitset GenerateBitSet() {
  // std::mt19937 rng(42);
  // std::uniform_real_distribution<float> bound_distribution(-10.f, 10.f);
  BrifBitset result;
  static std::default_random_engine generator;
  std::bernoulli_distribution distribution(0.5);
  for (int i = 0; i < 256; i++) {
    if (distribution(generator)) {
      result.set(i);
    } else {
      result.reset(i);
    }
  }
  return result;
};
constexpr char vocabulary_filebrif[] =
    "/home/lyp/project/vslam/jarvis/jarvis.dbow";
}  // namespace

TEST(Vocabular, TransformTest) {
  auto dbow2voc = GetVocabulary(vocabulary_filebrif);
  //
  auto dbow_voc = dbow::Vocabulary(dbow::GetVocabulary(0, vocabulary_filebrif));
  Descriptors descriptors;
  MapById<FeatureId, Descriptor> map_descriptors;
  for (int i = 0; i < 1000; i++) {
    auto des = GenerateBitSet();
    descriptors.push_back(des);
    map_descriptors.Insert({0, i}, des);
  }

  DBoW2::BowVector bowvector;
  DBoW2::FeatureVector featurevector;
  dbow2voc->transform(descriptors, bowvector, featurevector, 4);
  auto data = dbow_voc.Transform(map_descriptors, 4);
  //
  CHECK_EQ(bowvector.size(), data.bow_vector.size());

  std::vector<dbow::NodeId> result;
  dbow::MapIntersection(data.index_to_local_features, featurevector,
                        std::back_inserter(result));
  std::stringstream info;
  // for (auto f : data.index_to_local_features) {
  //   info << f.first << " "<<"->: "<<f.second.size();
  // }
  // info << "\n";
  // for (auto f : featurevector) {
  //   info << f.first << " "<<"->: "<<f.second.size();
  // }
  // LOG(INFO) <<"\n"<< info.str();
  CHECK_EQ(result.size(), featurevector.size());
  for (const auto& id : result) {
    CHECK_EQ(featurevector[id].size(), data.index_to_local_features[id].size()

    );
    for (int i = 0; i < featurevector[id].size(); i++) {
      CHECK_EQ(featurevector[id][i], data.index_to_local_features[id][i].index);
    }
  }
  auto b = bowvector.begin();
  auto b1 = data.bow_vector.begin();
  for (int i = 0; i < bowvector.size(); i++) {
    CHECK_EQ(dbow2voc->m_words[b->first]->id, b1->first);
    CHECK_NEAR(b->second, b1->second, 1e-5);
  }
}

//

//
std::vector<std::pair<int, int>> DbowFindMathed(
    const Descriptors& des1, const Descriptors& des2,
    const DBoW2::FeatureVector& feat_vec1,
    const DBoW2::FeatureVector& feat_vec2,
    double describe_distance_threashold) {
  //
  DBoW2::FeatureVector::const_iterator f1it = feat_vec1.begin();
  DBoW2::FeatureVector::const_iterator f2it = feat_vec2.begin();
  DBoW2::FeatureVector::const_iterator f1end = feat_vec1.end();
  DBoW2::FeatureVector::const_iterator f2end = feat_vec2.end();
  std::set<int> matched_index;
  std::vector<std::pair<int, int>> result;
  while (f1it != f1end && f2it != f2end) {
    if (f1it->first == f2it->first) {
      for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++) {
        const size_t idx1 = f1it->second[i1];
        const auto& d1 = des1[idx1];
        int bestDist1 = 256;
        int bestIdx2 = -1;
        int bestDist2 = 256;
        for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++) {
          const size_t idx2 = f2it->second[i2];
          if (matched_index.count(idx2)) continue;
          const auto& d2 = des2[idx2];
          int dist = HammingDis(d1, d2);
          if (dist < bestDist1) {
            bestDist2 = bestDist1;
            bestDist1 = dist;
            bestIdx2 = idx2;
          } else if (dist < bestDist2) {
            bestDist2 = dist;
          }
        }
        if (bestDist1 < describe_distance_threashold) {
          if (static_cast<float>(bestDist1) <
              0.8 * static_cast<float>(bestDist2)) {
            matched_index.insert(bestIdx2);
            result.push_back({idx1, bestIdx2});
          }
        }
      }
      f1it++;
      f2it++;
    } else if (f1it->first < f2it->first) {
      f1it = feat_vec1.lower_bound(f2it->first);
    } else {
      f2it = feat_vec2.lower_bound(f1it->first);
    }
  }
  return result;
}
TEST(Vocabular1, TransformTests) {
  auto dbow2voc = GetVocabulary(vocabulary_filebrif);
  //
  auto dbow_voc = dbow::Vocabulary(dbow::GetVocabulary(0, vocabulary_filebrif));

  ///
  std::vector<Descriptors> descriptors;
  std::vector<MapById<FeatureId, Descriptor>> map_descriptors;
  for (int i = 0; i < 1000; i++) {
    Descriptors descriptor;
    MapById<FeatureId, Descriptor> tem;
    for (int i = 0; i < 100; i++) {
      descriptor.push_back(GenerateBitSet());
      tem.Insert({0, i}, descriptor.back());
    }
    descriptors.push_back(descriptor);
    map_descriptors.push_back(tem);
  }

  //

  {
    std::vector<DBoW2::BowVector> bow_vectors;
    std::vector<DBoW2::FeatureVector> feature_vectors;
    std::vector<dbow::DbowData> dbow_datas;
    int i = 0;
    for (const auto descriptor : descriptors) {
      DBoW2::BowVector bowvector;
      DBoW2::FeatureVector featurevector;
      dbow2voc->transform(descriptor, bowvector, featurevector, 4);
      //
      bow_vectors.push_back(bowvector);
      feature_vectors.push_back(featurevector);
      //
      auto data = dbow_voc.Transform(map_descriptors[i], 4);
      i++;
      dbow_datas.push_back(data);
      //
      CHECK_EQ(bowvector.size(), data.bow_vector.size());

      std::vector<dbow::NodeId> result;
      dbow::MapIntersection(data.index_to_local_features, featurevector,
                            std::back_inserter(result));
      std::stringstream info;
      CHECK_EQ(result.size(), featurevector.size());
      for (const auto& id : result) {
        CHECK_EQ(
            featurevector[id].size(), data.index_to_local_features[id].size()

        );
        for (int i = 0; i < featurevector[id].size(); i++) {
          CHECK_EQ(featurevector[id][i],
                   data.index_to_local_features[id][i].index);
        }
      }
      auto b = bowvector.begin();
      auto b1 = data.bow_vector.begin();
      for (int i = 0; i < bowvector.size(); i++) {
        CHECK_EQ(dbow2voc->m_words[b->first]->id, b1->first);
        CHECK_NEAR(b->second, b1->second, 1e-5);
      }
    }
    for (int i = 0; i < descriptors.size(); i++) {
      for (int j = 0; j < descriptors.size(); j++) {
        CHECK_NEAR(dbow2voc->score(bow_vectors[i], bow_vectors[j]),
                   dbow_datas[i].Score(dbow_datas[j]), 1e-5);
        auto r1 = DbowFindMathed(descriptors[i], descriptors[j],
                                 feature_vectors[i], feature_vectors[j], 200);
        auto r2 = match::DbowFindMathed(map_descriptors[i], map_descriptors[j],
                                        dbow_datas[i], dbow_datas[j], 200);
        CHECK_EQ(r1.size(), r2.size());
        for (int i = 0; i < r1.size(); i++) {
          CHECK_EQ(r1[i].first, r2[i].first.index);
          CHECK_EQ(r1[i].second, r2[i].second.index);
          // LOG(INFO) << r1[i].first << " " << r2[i].first.index;
          // LOG(INFO) << r1[i].second << " " << r2[i].second.index;
        }
      }
    }
  }
}

}  // namespace jarvis_ros2