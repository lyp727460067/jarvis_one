#ifndef _JARVIS_MAPPING_MATCH_PIC_WRITER_H
#define _JARVIS_MAPPING_MATCH_PIC_WRITER_H
#include "jarvis/mapping/mapping_data.h"

namespace jarvis {
namespace mapping {
namespace match {

void WriteImageWithKeyPoint(
    const KeyFrameData::Data& first_data, const KeyFrameData::Data& sencod_data,
    const std::vector<std::pair<FeatureId, FeatureId>>& match_pair);
}
}  // namespace mapping
}  // namespace jarvis

#endif