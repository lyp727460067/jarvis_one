
#ifndef _JARVIS_MAPPING_DOBW2_VOCABULARY_ADPTOR_H
#define _JARVIS_MAPPING_DOBW2_VOCABULARY_ADPTOR_H
#include "DBoW2/DBoW2.h"
#include "jarvis/mapping/dbow/proto/vocabulary_data.pb.h"
#include "jarvis/mapping/dbow/vocabulary.h"
namespace jarvis_ros2 {
//
std::unique_ptr<BriefVocabulary> Dbow2FromProto(
    const jarvis::mapping::dbow::proto::VocabularyOption& option,
    const jarvis::mapping::dbow::proto::VocabularyData& proto);
//

jarvis::mapping::dbow::proto::VocabularyData Dbow2ToProto(
    const jarvis::mapping::dbow::VocabularyData::Info& info,
    const BriefVocabulary& voc);
//
//
// for temp
std::unique_ptr<BriefVocabulary> GetVocabulary(const std::string& pb_file,
                                               int type = 0);

}  // namespace jarvis_ros2

#endif