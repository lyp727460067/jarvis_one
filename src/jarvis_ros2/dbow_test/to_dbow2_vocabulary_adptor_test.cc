#include "to_dbow2_vocabulary_adptor.h"

#include "gtest/gtest.h"
namespace jarvis_ros2 {
using namespace jarvis;
using namespace mapping;
// ./build/jarvis_ros2/dbow_test.to_dbow2_vocabulary_adptor_test  --gtest_filter="ToDbow.Dbow2ToProtoDbow2FromProto2"

TEST(ToDbow, Dbow2ToProtoDbow2FromProto) {
  std::string vocabulary =
      "/home/lyp/project/vslam/ORB_SLAM3/Vocabulary/ORBvoc.txt";

  auto voc = std::make_unique<BriefVocabulary>();
  voc->loadFromTextFile(vocabulary);
  // LOG(INFO) << voc->getScoringType();
  // LOG(INFO) << voc->getWeightingType();
  auto proto = Dbow2ToProto(
      dbow::VocabularyData::Info{"kG2Brief", true, voc->m_k, voc->m_L}, *voc);
  dbow::proto::VocabularyOption option;
  option.set_score_type(voc->getScoringType());
  option.set_weighting_type(voc->getWeightingType());
  //
  LOG(INFO) << option.DebugString();
  //
  auto proto_voc = Dbow2FromProto(option, proto);
  CHECK_EQ(proto_voc->m_k, voc->m_k);
  CHECK_EQ(proto_voc->m_L, voc->m_L);
  CHECK_EQ(proto_voc->m_nodes.size(), voc->m_nodes.size());

  for (int i = 0; i < voc->m_nodes.size(); i++) {
    CHECK_EQ(voc->m_nodes[i].id, proto_voc->m_nodes[i].id) << i;
    CHECK_EQ(voc->m_nodes[i].isLeaf(), proto_voc->m_nodes[i].isLeaf()) << i;
    CHECK_EQ(voc->m_nodes[i].descriptor, proto_voc->m_nodes[i].descriptor) << i;
    CHECK_EQ(voc->m_nodes[i].parent, proto_voc->m_nodes[i].parent) << i;
    CHECK_EQ(voc->m_nodes[i].weight, proto_voc->m_nodes[i].weight) << i;
    CHECK_EQ(voc->m_nodes[i].word_id, proto_voc->m_nodes[i].word_id) << i;
    for (int j = 0; j < voc->m_nodes[i].children.size(); j++) {
      CHECK_EQ(voc->m_nodes[i].children[j], proto_voc->m_nodes[i].children[j]);
    }
  }
  CHECK_EQ(voc->m_words.size(), proto_voc->m_words.size());
  // for (int i = 0; i < voc->m_words.size(); i++) {
  //   CHECK_EQ(voc->m_words[i], proto_voc->m_words[i]);
  // }
}
TEST(ToDbow, Dbow2ToProtoDbow2FromProto1) {
  std::string vocabulary =
        "/home/lyp/project/catkin_ws/src/VINS-Fusion/support_files/"
        "brief_k10L6.bin";

  auto voc = std::make_unique<BriefVocabulary>();
  voc->loadFromTextBin(vocabulary);
  auto proto = Dbow2ToProto(
      dbow::VocabularyData::Info{"kG1Brief", false, voc->m_k, voc->m_L}, *voc);
  dbow::proto::VocabularyOption option;
  option.set_score_type(voc->getScoringType());
  option.set_weighting_type(voc->getWeightingType());
  //
  LOG(INFO) << option.DebugString();
  //
  auto proto_voc = Dbow2FromProto(option, proto);
  CHECK_EQ(proto_voc->m_k, voc->m_k);
  CHECK_EQ(proto_voc->m_L, voc->m_L);
  CHECK_EQ(proto_voc->m_nodes.size(), voc->m_nodes.size());

  for (int i = 0; i < voc->m_nodes.size(); i++) {
    CHECK_EQ(voc->m_nodes[i].id, proto_voc->m_nodes[i].id);
    CHECK_EQ(voc->m_nodes[i].isLeaf(), proto_voc->m_nodes[i].isLeaf());
    CHECK_EQ(voc->m_nodes[i].descriptor, proto_voc->m_nodes[i].descriptor);
    CHECK_EQ(voc->m_nodes[i].parent, proto_voc->m_nodes[i].parent);
    CHECK_EQ(voc->m_nodes[i].weight, proto_voc->m_nodes[i].weight);
    CHECK_EQ(voc->m_nodes[i].word_id, proto_voc->m_nodes[i].word_id);
    for (int j = 0; j < voc->m_nodes[i].children.size(); j++) {
      CHECK_EQ(voc->m_nodes[i].children[j], proto_voc->m_nodes[i].children[j]);
    }
  }
  CHECK_EQ(voc->m_words.size(), proto_voc->m_words.size());
  // for (int i = 0; i < voc->m_words.size(); i++) {
  //   CHECK_EQ(voc->m_words[i], proto_voc->m_words[i]);
  // }
}

TEST(ToDbow, Dbow2ToProtoDbow2FromProto2) {
  dbow::proto::VocabularyDatas protos;

  // {
  //   std::string vocabulary =
  //       "/home/lyp/project/catkin_ws/src/VINS-Fusion/support_files/"
  //       "brief_k10L6.bin";
  //   auto voc = std::make_unique<BriefVocabulary>();
  //   voc->loadFromTextBin(vocabulary);
  //   auto proto = Dbow2ToProto(
  //       dbow::VocabularyData::Info{"kG1Brief", false, voc->m_k, voc->m_L},
  //       *voc);
  //   *protos.add_vocabulary_datas() = proto;
  // }
  {
    std::string vocabulary =
        "/home/lyp/project/vslam/ORB_SLAM3/Vocabulary/ORBvoc.txt";

    auto voc = std::make_unique<BriefVocabulary>();
    voc->loadFromTextFile(vocabulary);
    auto proto = Dbow2ToProto(
        dbow::VocabularyData::Info{"kG2Brief", true, voc->m_k, voc->m_L}, *voc);

    *protos.add_vocabulary_datas() = proto;
  }

  std::string pb_file = "/home/lyp/project/vslam/jarvis/jarvis.dbow";
  std::ofstream os(pb_file, std::ios::out | std::ios::binary);
  //
  CHECK(protos.SerializePartialToOstream(&os)) << "seria failed";
  os.close();
}
}  // namespace jarvis_ros2