
#ifndef _JARVIS_MAPPING_DBOW_TRAINGING_VOCABULARY_H
#define _JARVIS_MAPPING_DBOW_TRAINGING_VOCABULARY_H
#include <map>
#include <memory>
#include <vector>

#include "jarvis/mapping/dbow/proto/vocabulary_data.pb.h"
#include "jarvis/mapping/dbow/vocabulary.h"
namespace jarvis {
namespace mapping {
namespace dbow {

struct TrainingVocabularyOption {};
//

class Cluster {
 public:
  virtual ~Cluster() = 0;
};
class HKMeans : public Cluster {};

class TrainingVocabulary {
 public:
  //
  TrainingVocabulary(const TrainingVocabularyOption& option,
                     std::vector<Descriptors> descriptors,
                     const proto::VocabularyData* const base = nullptr);
  //
  void Serialization(const std::string& pb_file);

 private:
};
//
}  // namespace dbow
}  // namespace mapping
}  // namespace jarvis

#endif