
#ifndef _JARVIS_MAPPING_DBOW_VOCABULARY_H
#define _JARVIS_MAPPING_DBOW_VOCABULARY_H
#include <map>
#include <memory>
#include <vector>

#include "jarvis/common/id.h"
//
#include "jarvis/mapping/des/des_data_type.h"
#include "jarvis/mapping/dbow/proto/vocabulary_data.pb.h"
#include "sstream"
namespace jarvis {
namespace mapping {
namespace dbow {
//
Descriptor FromProto(const proto::Descriptor& proto);
proto::Descriptor ToProto(const Descriptor& data);
struct NodeData {
  double weight;
  Descriptor descriptor;
};
template <typename A, typename B, typename C, typename Iterator>
void MapIntersection(const std::map<A, B>& a, const std::map<A, C>& b,
                     Iterator result) {
  auto a_it = a.begin();
  auto b_it = b.begin();
  while (a_it != a.end() && b_it != b.end()) {
    if (a_it->first == b_it->first) {
      *result = a_it->first;
      ++a_it;
      ++b_it;
      ++result;
    } else if (a_it->first < b_it->first) {
      a_it = a.lower_bound(b_it->first);
    } else {
      b_it = b.lower_bound(a_it->first);
    }
  }
}
using NodeId = uint64_t;
//
struct Node {
  NodeId id;
  NodeId parent_id;
  std::vector<NodeId> childrens;
  inline bool IsLeaf() const { return childrens.empty(); }
};
//
struct VocabularyData {
  //
  struct Info {
    std::string descriptor_type;
    bool descriptor_rotated = false;
    int branching_factor = 10;
    int depth_levels = 6;
  } info;
  std::map<NodeId, NodeData> node_datas;
  std::map<NodeId, Node> nodes;
  std::vector<NodeId> word_ids;
  //
  //
  std::string DebugInfo() {
    std::stringstream in;
    in << "descriptor_type: " << info.descriptor_type << "\n";
    in << "descriptor_rotated  : " << info.descriptor_rotated << "\n";
    in << "branching_factor : " << info.branching_factor << "\n";
    in << "depth_levels : " << info.depth_levels;
    return in.str();
  }
};
//
//
// todo: need implement
//
struct DbowData {
  using WordId = uint64_t;
  std::map<NodeId, double> bow_vector;
  std::map<NodeId, std::vector<FeatureId>> index_to_local_features;
  enum struct NormType { L1 = 0, L2 };
  void AddBowVectorValue(const NodeId& id, double v);
  DbowData NormalizeBowVector();
  double Score(const DbowData& v) const;
};
//
class GeneralScoring {};
class Vocabulary {
 public:
  //
  explicit Vocabulary(std::unique_ptr<proto::VocabularyData> voc);
  Vocabulary(Vocabulary&& rhs):
      vocabulary_data_(std::move(rhs.vocabulary_data_)),
      vocabulary_data_des_catch_(std::move(rhs.vocabulary_data_des_catch_)) {}

  DbowData Transform(const MapById<FeatureId, Descriptor>& descriptors,
                     int sub_level);
  //
 private:
  //
  std::tuple<NodeId, NodeId> Transform(const Descriptor& descriptor,
                                       int sub_level);
  //
  std::unique_ptr<proto::VocabularyData> vocabulary_data_;
  std::unordered_map<uint64_t, Descriptor> vocabulary_data_des_catch_;
  const Descriptor& VocabularyDes(const uint64_t& id);
};
//
std::unique_ptr<proto::VocabularyData> GetVocabulary(
    const int type, const std::string& pb_file);
//
}  // namespace dbow
}  // namespace mapping
}  // namespace jarvis

#endif