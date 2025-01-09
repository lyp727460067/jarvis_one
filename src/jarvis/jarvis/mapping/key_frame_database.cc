#include "jarvis/mapping/key_frame_database.h"

#include <list>

#include "mapping/covisibility.h"
namespace jarvis {
namespace mapping {
//
KeyFrameDataBase::KeyFrameDataBase(
    const KeyFrameDataBaseOption& option,
    std::shared_ptr<dbow::Vocabulary> voc)
    : voc_(std::move(voc)), options_(option) {}
//
KeyFrameDataBase::~KeyFrameDataBase() {}
//
void KeyFrameDataBase::AddData(const KeyFrameId& key_frame_id,
                               std::weak_ptr<const KeyFrameData::Data> data) {
  for (const auto& bow_vec : data.lock()->dbow_data.bow_vector) {
    inverted_file_[bow_vec.first].push_back(key_frame_id);
  }
  key_frame_datas_.emplace(key_frame_id, data);
}

//
void KeyFrameDataBase::Erase(const KeyFrameId& id) {
  // Erase elements in the Inverse File for the entry
  CHECK(key_frame_datas_[id].lock());
  for (const auto& bow_vec :
       key_frame_datas_[id].lock()->dbow_data.bow_vector) {
    std::list<KeyFrameId>& lKFs = inverted_file_[bow_vec.first];
    for (std::list<KeyFrameId>::iterator lit = lKFs.begin(), lend = lKFs.end();
         lit != lend; lit++) {
      if (id == *lit) {
        lKFs.erase(lit);
        break;
      }
    }
  }
  key_frame_datas_.erase(id);
}
void KeyFrameDataBase::Clear() {
  //   mvInvertedFile.clear();
  //   mvInvertedFile.resize(mpVoc->size());
}
std::unordered_map<KeyFrameId, double> KeyFrameDataBase::FindSimilarCandidate(
    const std::shared_ptr<const KeyFrameData::Data>& data,
    const std::set<KeyFrameId>& exclude_ids, double min_score) const {
  // CHECK(!exclude_ids.empty());
  //
  std::map<KeyFrameId, int> sharing_words_key_frame_ids;
  CHECK(data);
  for (const auto& bow_vec : data->dbow_data.bow_vector) {
    if (inverted_file_.count(bow_vec.first)==0) continue;
    const std::list<KeyFrameId>& key_frame_ids =
        inverted_file_.at(bow_vec.first);
    //

    for (const auto& id : key_frame_ids) {
      bool distance_exclude =
          (key_frame_datas_.at(id).lock()->pose.inverse() * data->pose)
              .translation()
              .norm() > options_.min_distance_threash_hold;

      if (!sharing_words_key_frame_ids.count(id)) {
        if (exclude_ids.count(id) || distance_exclude) continue;
        sharing_words_key_frame_ids.emplace(id, 0);
      }
      sharing_words_key_frame_ids[id]++;
    }
  }
  if (sharing_words_key_frame_ids.empty()) return {};
  return ComputeSimilarityScore(sharing_words_key_frame_ids, data, min_score);
}
//
std::pair<int, int> KeyFrameDataBase::ComputeMaxMinCommonwords(
    const std::map<KeyFrameId, int>& ids) const {
  // Only compare against those keyframes that share enough words
  int max_common_words = 0;
  for (const auto id : ids) {
    if (id.second > max_common_words) {
      max_common_words = id.second;
    }
  }
  if (max_common_words < options_.min_shared_words_num) {
    max_common_words = options_.min_shared_words_num;
  }
  return {max_common_words,
          options_.sharing_words_count_min_is_max_ration * max_common_words};
}
// Compute similarity score.
std::unordered_map<KeyFrameId, double> KeyFrameDataBase::ComputeSimilarityScore(
    const std::map<KeyFrameId, int>& shared_ids_with_count,
    const std::shared_ptr<const KeyFrameData::Data>& data,
    double min_score) const {
  std::unordered_map<KeyFrameId, double> result;
  auto min_common_words = ComputeMaxMinCommonwords(shared_ids_with_count);
  CHECK_GT(min_common_words.first, min_common_words.second);
  for (auto const& id_count : shared_ids_with_count) {
    CHECK(key_frame_datas_.count(id_count.first) != 0);
    if (id_count.second > min_common_words.second) {
      CHECK(key_frame_datas_.at(id_count.first).lock());
      // double score =
      //     voc_->score(data->dbow_bow_vec,
      //                 key_frame_datas_.at(id_count.first).lock()->dbow_bow_vec);
      double score = data->dbow_data.Score(
          key_frame_datas_.at(id_count.first).lock()->dbow_data);
      // voc_->score(data->dbow_bow_vec,
      //             key_frame_datas_.at(id_count.first).lock()->dbow_bow_vec);

      //
      min_score = std::max(options_.min_core, min_score);
      if (score > min_score) {
        result.emplace(id_count.first, score);
      }
    }
  }
  return result;
}
}  // namespace mapping
}  // namespace jarvis