#include "jarvis/mapping/dbow/vocabulary.h"
#include <fstream>
namespace jarvis {
namespace mapping {
namespace dbow {


Descriptor FromProto(const proto::Descriptor& proto) {
  std::vector<uint8_t> descrip;
  for (auto const& de : proto.brif_bitset()) {
    descrip.push_back(static_cast<uint8_t>(de));
  }
  return Uint8ToBitSet<Descriptor>(descrip);
}
proto::Descriptor ToProto(const Descriptor& data) {
  proto::Descriptor proto;
  auto de_vecs = BitSetToUint8(data);
  for (auto const& de : de_vecs) {
    proto.mutable_brif_bitset()->Add(de);
  }
  return proto;
}
//
void DbowData::AddBowVectorValue(const NodeId& id, double v) {
  if (!bow_vector.emplace(id, v).second) {
    bow_vector[id] += v;
  }
}
//

DbowData DbowData::NormalizeBowVector() {
  double norm = 0.0;
  for (const auto& v : bow_vector) {
    norm += fabs(v.second);
  }
  if (norm > 0.0) {
    for (auto& v : bow_vector) {
      v.second /= norm;
    }
  }

  return *this;
}
//
double DbowData::Score(const DbowData& v) const {
  std::vector<NodeId> result;
  MapIntersection(v.bow_vector, bow_vector, std::back_inserter(result));

  double score = 0;
  for (const auto& r : result) {
    const auto& vi = v.bow_vector.at(r);
    const auto& wi = this->bow_vector.at(r);
    score += fabs(vi - wi) - fabs(vi) - fabs(wi);
  }
  return -score / 2.0;
}
//
//
Vocabulary::Vocabulary(std::unique_ptr<proto::VocabularyData> voc)
    : vocabulary_data_(std::move(voc)) {
  for (const auto& v : vocabulary_data_->node_datas()) {
    vocabulary_data_des_catch_.emplace(
        v.first,
        FromProto(vocabulary_data_->node_datas().at(v.first).descriptors()));
  }
}
//
//
//
DbowData Vocabulary::Transform(
    const MapById<FeatureId, Descriptor>& descriptors, int sub_level) {
  //
  DbowData result;
  const auto& voc_ = *vocabulary_data_;
  for (const auto& f : descriptors) {
    NodeId nid = 0;
    double w = 0;
    auto [world_id, mid_id] = Transform(f.data, sub_level);
    if (voc_.node_datas().at(world_id).weight() > 0) {
      FeatureId i_feature = f.id;
      result.AddBowVectorValue(world_id,
                               voc_.node_datas().at(world_id).weight());
      result.index_to_local_features[mid_id].push_back(i_feature);
    }
  }
  CHECK(!descriptors.empty());
  return result.NormalizeBowVector();
}

//
std::tuple<NodeId, NodeId> Vocabulary::Transform(
    const Descriptor& descriptor, int sub_level) {
  //
  const auto& voc_ = *vocabulary_data_;
  const int nid_level = voc_.info().depth_levels() - sub_level;
  CHECK(nid_level > 0) << "sub level biger info().depth_levels()";
  NodeId final_id = 0;  // root
  int current_level = 0;
  NodeId nid_id = 0;

  do {
    //
    ++current_level;
    const auto& nodes = voc_.nodes().at(final_id).childrens_ids();
    final_id = nodes[0];
    double best_d = HammingDis(descriptor, VocabularyDes(final_id));
    //
    for (auto nit = nodes.begin() + 1; nit != nodes.end(); ++nit) {
      NodeId id = *nit;
      //
      const double d = HammingDis(descriptor, VocabularyDes(id));
      //
      if (d < best_d) {
        best_d = d;
        final_id = id;
      }
    }
    if (current_level == nid_level) {
      nid_id = final_id;
    }

  } while (!voc_.nodes().at(final_id).childrens_ids().empty());
  return {final_id, nid_id};
}
//
std::unique_ptr<proto::VocabularyData> GetVocabulary(
    const int type, const std::string& pb_file) {
  std::ifstream is(pb_file, std::ios::in | std::ios::binary);
  LOG(INFO) << "Load :" << pb_file;
  CHECK(is.good()) << "Load   " << pb_file << " Faild";
  proto::VocabularyDatas proto;
  proto.ParseFromIstream(&is);
  LOG(INFO) << "Load done.";
  std::stringstream info;
  std::unique_ptr<proto::VocabularyData> result = nullptr;
  info << "Pb has voc size: " << proto.vocabulary_datas_size() << "\n";
  for (int i = 0; i < proto.vocabulary_datas_size(); i++) {
    // const VocabularyData p  ;//proto.vocabulary_datas().at(i);
    // info << "Index : " << i << " \n" << p.info().DebugString() << "\n\n";
  }
  info << "Option choose voc id : ";
  result =
      std::make_unique<proto::VocabularyData>(proto.vocabulary_datas()[type]);

  LOG(INFO) << info.str() << "\n";
  return std::move(result);
}

const Descriptor& Vocabulary::VocabularyDes(const uint64_t& id) {
  return vocabulary_data_des_catch_.at(id);
}
}  // namespace dbow
//
}  // namespace mapping
}  // namespace jarvis
