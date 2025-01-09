#include "to_dbow2_vocabulary_adptor.h"

#include "jarvis/mapping/dbow/vocabulary.h"
namespace jarvis_ros2 {
using namespace jarvis;
using namespace mapping;
//
std::unique_ptr<BriefVocabulary> Dbow2FromProto(
    const dbow::proto::VocabularyOption& option,
    const dbow::proto::VocabularyData& proto) {
  std::unique_ptr<BriefVocabulary> voc(new BriefVocabulary);
  //
  // LOG(INFO) << proto.info().DebugString();
  //
  voc->m_k = proto.info().branching_factor();
  voc->m_L = proto.info().depth_levels();
  //
  voc->m_scoring = static_cast<DBoW2::ScoringType>(option.score_type());
  voc->m_weighting = static_cast<DBoW2::WeightingType>(option.weighting_type());
  //
  voc->createScoringObject();
  //
  voc->m_nodes.resize(proto.node_datas().size());

  for (int i = 0; i < proto.nodes().size(); i++) {
    auto& node = proto.nodes().at(i);
    auto& m_node = voc->m_nodes[i];
    m_node.id = i;
    m_node.weight = proto.node_datas().at(i).weight();
    m_node.parent = node.parent_id();
    m_node.word_id = 0;
    m_node.descriptor = dbow::FromProto(proto.node_datas().at(i).descriptors());
    for (const auto c_id : node.childrens_ids()) {
      m_node.children.push_back(c_id);
    }
    if (m_node.children.empty()) {
      m_node.word_id = voc->m_words.size();
      voc->m_words.push_back(&m_node);
    }
  }
  return voc;
}
//
dbow::proto::VocabularyData Dbow2ToProto(const dbow::VocabularyData::Info& info,
                                         const BriefVocabulary& voc) {
  dbow::proto::VocabularyData proto;
  //
  proto.mutable_info()->set_descriptor_type(info.descriptor_type);
  proto.mutable_info()->set_descriptor_rotated(info.descriptor_rotated);
  proto.mutable_info()->set_branching_factor(info.branching_factor);
  proto.mutable_info()->set_depth_levels(info.depth_levels);
  //
  LOG(INFO) << proto.info().DebugString();
  for (int i = 0; i < voc.m_nodes.size(); i++) {
    const auto& node = voc.m_nodes[i];
    dbow::proto::VocabularyData::Node proto_node;
    proto_node.set_id(node.id);
    proto_node.set_parent_id(node.parent);
    for (const auto& id : node.children) {
      proto_node.add_childrens_ids(id);
    }

    dbow::proto::VocabularyData::NodeData proto_node_data;
    proto.mutable_nodes()->insert({node.id, proto_node});
    *proto_node_data.mutable_descriptors() = dbow::ToProto(node.descriptor);
    proto_node_data.set_weight(node.weight);
    proto.mutable_node_datas()->insert({node.id, proto_node_data});
  }
  return proto;
}

std::unique_ptr<BriefVocabulary> GetVocabulary(const std::string& pb_file,
                                               int type) {
  std::ifstream is(pb_file, std::ios::in | std::ios::binary);
  LOG(INFO) << "Load :" << pb_file;
  CHECK(is.good()) << "Load   " << pb_file << " Faild";
  dbow::proto::VocabularyDatas proto;
  proto.ParseFromIstream(&is);

  LOG(INFO) << "Load done.";
  std::stringstream info;
  std::unique_ptr<BriefVocabulary> result;
  info << "stream has voc size: " << proto.vocabulary_datas_size() << "\n";
  for (int i = 0; i < proto.vocabulary_datas_size(); i++) {
    const auto& p = proto.vocabulary_datas(i);
    info << "Index : " << i << " \n" << p.info().DebugString() << "\n\n";
  }
  info << "Option choose voc id : ";
  if (type == 0) {
    info << "0";
    result = Dbow2FromProto({}, proto.vocabulary_datas(0));
  } else if (type == 1) {
    info << "1";
    result = Dbow2FromProto({}, proto.vocabulary_datas(1));
  } else {
    LOG(FATAL) << "Not support ." << type;
  }

  LOG(INFO) << info.str() << "\n";
  return std::move(result);
}

}  // namespace jarvis_ros2