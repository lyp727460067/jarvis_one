
#ifndef __JARVIS_MAPPING_KEY_FRAME_DB_H__
#define __JARVIS_MAPPING_KEY_FRAME_DB_H__
#include <bitset>
#include <list>
#include <unordered_map>
//
#include "jarvis/common/id.h"
#include "jarvis/mapping/mapping_data.h"
#include "memory"
namespace jarvis {
namespace mapping {
//
struct KeyFrameDataBaseOption {
  double min_core=0.8;
  double sharing_words_count_min_is_max_ration=0.8;
  int min_shared_words_num  =10;
  double min_distance_threash_hold=5;

};
namespace dbow{
class Vocabulary;
}
class KeyFrameDataBase {
 public:
  KeyFrameDataBase(const KeyFrameDataBaseOption& option,
                   std::shared_ptr<dbow::Vocabulary> voc);
  ~KeyFrameDataBase();
  //
  void AddData(const KeyFrameId& key_frame_id,
               std::weak_ptr<const KeyFrameData::Data> data);
  //
  void Erase(const KeyFrameId& id);
  void Clear();
  //
  std::unordered_map<KeyFrameId, double> FindSimilarCandidate(
      const std::shared_ptr<const KeyFrameData::Data>& data,
      const std::set<KeyFrameId>& exclude_ids, double min_score = 10) const;
  //
  dbow::Vocabulary* Vocabulary() const { return voc_.get(); }

 protected:
  std::pair<int, int> ComputeMaxMinCommonwords(
      const std::map<KeyFrameId, int>& ids) const;
  //
  std::unordered_map<KeyFrameId, double> ComputeSimilarityScore(
      const std::map<KeyFrameId, int>& shared_ids_with_count,
      const std::shared_ptr<const KeyFrameData::Data>& data,
      double min_score) const;
  //

  KeyFrameDataBaseOption options_;
  std::shared_ptr<dbow::Vocabulary> voc_;
  std::map<uint64_t, std::list<KeyFrameId>> inverted_file_;
  std::map<KeyFrameId, std::weak_ptr<const KeyFrameData::Data>>
      key_frame_datas_;
};
}  // namespace mapping
}  // namespace jarvis

#endif  //