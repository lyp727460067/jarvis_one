#include "jarvis/mapping/match/pic_writer.h"

#include <random>
namespace jarvis {
namespace mapping {
namespace match {
// constexpr char kTestImageDir[] = "/home/lyp/project/vslam/jarvis/test/image/";
void WriteImageWithKeyPoint(const std::string &path,
    const KeyFrameData::Data& first_data, const KeyFrameData::Data& sencod_data,
    const std::vector<std::pair<FeatureId, FeatureId>>& match_pair) {
//  srand(static_cast<unsigned>(time(0)));
#ifndef __ARM_PLATFORM__
  std::map<int, std::map<int, std::pair<cv::Mat, int>>> catch_iamges;
  for (const auto& p : match_pair) {
    if (catch_iamges.count(p.first.sequence_id) == 0) {
      int width = first_data.image_sizes->at(p.first.sequence_id).sizes().x() +
                  sencod_data.image_sizes->at(p.second.sequence_id).sizes().x();

      int height = std::max(
          first_data.image_sizes->at(p.first.sequence_id).sizes().y(),
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().y());

      cv::Mat image(height,width, CV_8UC1, cv::Scalar::all(0));
      //
      cv::Mat temp1 = image(cv::Rect(
          0, 0, first_data.image_sizes->at(p.first.sequence_id).sizes().x(),
          first_data.image_sizes->at(p.first.sequence_id).sizes().y()));
      first_data.Pyramid(p.first.sequence_id)[0].copyTo(temp1);
      cv::Mat tem = image(cv::Rect(
          first_data.image_sizes->at(p.first.sequence_id).sizes().x(), 0,
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().x(),
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().y()));
      sencod_data.Pyramid(p.second.sequence_id)[0].copyTo(tem);
      cvtColor(image,image, cv::COLOR_GRAY2RGB);
      catch_iamges[p.first.sequence_id].emplace(
          p.second.sequence_id,
          std::make_pair(
              image.clone(),
              first_data.image_sizes->at(p.first.sequence_id).sizes().x())

      );
    }
    if (catch_iamges.at(p.first.sequence_id).count(p.second.sequence_id) == 0) {
      int width = first_data.image_sizes->at(p.first.sequence_id).sizes().x() +
                  sencod_data.image_sizes->at(p.second.sequence_id).sizes().x();

      int height = std::max(
          first_data.image_sizes->at(p.first.sequence_id).sizes().y(),
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().y());
      cv::Mat image(height,width, CV_8UC1, cv::Scalar::all(0));
      first_data.Pyramid(p.first.sequence_id)[0].copyTo(image(cv::Rect(
          0, 0, first_data.image_sizes->at(p.first.sequence_id).sizes().x(),
          first_data.image_sizes->at(p.first.sequence_id).sizes().y())));
      sencod_data.Pyramid(p.second.sequence_id)[0].copyTo(image(cv::Rect(
          first_data.image_sizes->at(p.first.sequence_id).sizes().x(), 0,
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().x(),
          sencod_data.image_sizes->at(p.second.sequence_id).sizes().y())));

      cvtColor(image,image, cv::COLOR_GRAY2RGB);

      catch_iamges[p.first.sequence_id].emplace(
          p.second.sequence_id,
          std::make_pair(
              image.clone(),
              first_data.image_sizes->at(p.first.sequence_id).sizes().x()));
    }
    cv::Mat& image =
        catch_iamges[p.first.sequence_id][p.second.sequence_id].first;
    int gap = catch_iamges[p.first.sequence_id][p.second.sequence_id].second;
    //
    int lower = 1, upper = 255;
    int ranged_random = lower + rand() % (upper - lower + 1);

    cv::Scalar color = cv::Scalar(ranged_random, 0, 255 - ranged_random);
    cv::Point2f point1 = first_data.features.at(p.first).key_point.pt;
    cv::circle(image, point1, 2, color, 2);
    cv::Point2f se_point = sencod_data.features.at(p.second).key_point.pt;
    se_point.x += gap;
    cv::circle(image, se_point, 2, color, 2);
    cv::line(image, point1, se_point, color, 1, 1, 0);
  }

  for (auto& image : catch_iamges) {
    for (auto& image2 : image.second) {
      const std::string file_name =
          path + std::to_string(common::ToUniversal(first_data.time) * 100) + 
          " "+std::to_string(common::ToUniversal(sencod_data.time) * 100) +
          "_" + std::to_string(image.first) + "_" +
          std::to_string(image2.first) + ".png";
      cv::putText(image2.second.first,
                  std::to_string(common::ToUniversal(first_data.time) * 100) +
                  " "+std::to_string(common::ToUniversal(sencod_data.time) * 100) +
                      "_" + std::to_string(image.first) + "_" +
                      std::to_string(image2.first),
                  cv::Point(20, 30), cv::FONT_HERSHEY_SIMPLEX, 1,
                  cv::Scalar(0, 0, 255), 2, 3);
      // cv::imshow("match", image2.second.first);

      // cv::waitKey(0);
      cv::imwrite(file_name, image2.second.first);
    }
  }
#endif
}

}  // namespace match
}  // namespace mapping
}  // namespace jarvis