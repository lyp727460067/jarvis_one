#include "jarvis/mapping/covisibility.h"

#include "glog/logging.h"
#include "gtest/gtest.h"
namespace jarvis {
namespace mapping {
//

class CovisibilityTest : public ::testing::Test {
 public:
  virtual void SetUp() {}
  protected:
  Covisibility covisibility;  
};
//    1 2 3 4 5 6 7 8 9 10
//            5 6 7 8 9 10 11 12 13 14 15
//                7 8 9 10 11 12 13 14 15 16 17
//                                           17 18 19 20 21 22 23 24 25 26  
TEST_F(CovisibilityTest, UpdateConnectTrackingDataTest) {
 
  std::vector<KeyFrameId> key_frame_ids;
  std::vector<MapPointId> map_point_ids;
  for (int i = 0; i < 4; i++) {
    key_frame_ids.push_back({0, i});
  }

  for (int i = 0; i < 100; i++) {
    map_point_ids.push_back({0, i});
  }
  int map_point_index = 0;
  std::map<MapPointId, FeatureId> tracking_data;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[i], FeatureId{0, i});
  }
  map_point_index = 10; 
  covisibility.UpdateWithFrameData(key_frame_ids[0],
                                        std::move(tracking_data));

  //
  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 0}).size(), 0);
  //
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 5 + i],
                          FeatureId(0, i));
  }
  // map_point_index =20;
  covisibility.UpdateWithFrameData(key_frame_ids[1],
                                        std::move(tracking_data));


  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 0}).size(), 1);
  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 1}).size(), 1);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1},{0,0}), 5);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,1}), 5);
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 3+i],FeatureId(0,i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[2],
                                        std::move(tracking_data));
  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 0}).size(), 2);  
  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 1}).size(), 2);  
  CHECK_EQ(covisibility.GetConnectedKeyFrames({0, 1}).size(), 2);  


  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,2}), 3);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1},{0,2}), 8);
  tracking_data.clear();
  map_point_index = 17;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 1+i],FeatureId(0,i) );
  }
  covisibility.UpdateWithFrameData(key_frame_ids[3],
                                        std::move(tracking_data));
  

  auto connect_key_frame =covisibility.GetConnectedKeyFrames({0, 3});
  CHECK_EQ(connect_key_frame.size(), 1);
  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 2));
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 2},{0,3}), 1);

}
//mapid 0 1 2 3 4 5 6 7 8 9 10  11 12 13 14 15 16 17 18 19 20 21 22 23 24 25
//index 1 2 3 4 5 6 7 8 9 10
//              5 6 7 8 9 10 11 12 13 14 15
//                  7 8 9 10 11 12 13 14 15 16 17
//                                             17 18 19 20 21 22 23 24 25 26 
//fuse 25 14                              *                               *
//
TEST_F(CovisibilityTest, UpdateConnectTrackingDataFuseTest) {
  std::vector<KeyFrameId> key_frame_ids;
  std::vector<MapPointId> map_point_ids;
  for (int i = 0; i < 5; i++) {
    key_frame_ids.push_back({0, i});
  }

  for (int i = 0; i < 100; i++) {
    map_point_ids.push_back({0, i});
  }
  int map_point_index = 0;
  std::map<MapPointId,FeatureId> tracking_data;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[i], FeatureId(0, i));
  }
  map_point_index = 10; 
  covisibility.UpdateWithFrameData(key_frame_ids[0],
                                        std::move(tracking_data));
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 5+i],FeatureId(0,i));
  }
  // map_point_index =20;
  covisibility.UpdateWithFrameData(key_frame_ids[1],
                                        std::move(tracking_data));


  //
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 3+i],FeatureId(0,i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[2],
                                        std::move(tracking_data));
  //

  tracking_data.clear();
  map_point_index = 17;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 1+i],FeatureId(0,i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[3],
                                        std::move(tracking_data));
  auto connect_key_frame =covisibility.GetConnectedKeyFrames({0, 3});                                       
 //
  
  CHECK_EQ(connect_key_frame.size(), 1);
  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 2));
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 2},{0,3}), 1);
  //



  covisibility.UpdateWithFuseMapPoint({0, 25}, {0, 14});
  connect_key_frame = covisibility.GetConnectedKeyFrames({0, 3});
  CHECK_EQ(connect_key_frame.size(), 2);
  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 1));
  CHECK_EQ(connect_key_frame[1], KeyFrameId(0, 2));
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1}, {0, 3}), 1);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 2}, {0, 3}), 2);

  //
  connect_key_frame = covisibility.GetConnectedKeyFrames({0, 1});
  for(const auto frame:connect_key_frame ){
    LOG(INFO)<<frame;
  }
  // CHECK_EQ(connect_key_frame.size(), 3);

  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 0));
  CHECK_EQ(connect_key_frame[1], KeyFrameId(0, 2));
  CHECK_EQ(connect_key_frame[2], KeyFrameId(0, 3));


  //
  map_point_index = 25;
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 1 + i],FeatureId(0,i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[4],
                                      std::move(tracking_data));

  
  connect_key_frame = covisibility.GetConnectedKeyFrames({0, 4});
  CHECK_EQ(connect_key_frame.size(),3);


  connect_key_frame = covisibility.GetConnectedKeyFrames({0, 1});
  // CHECK_EQ(connect_key_frame.size(), 3);
  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 0));
  CHECK_EQ(connect_key_frame[1], KeyFrameId(0, 2));
  CHECK_EQ(connect_key_frame[2], KeyFrameId(0, 3));
  CHECK_EQ(connect_key_frame[3], KeyFrameId(0, 4));

}

TEST_F(CovisibilityTest, UpdateConnectTrackingDataFuseKeyFrameTest) {
  std::vector<KeyFrameId> key_frame_ids;
  std::vector<MapPointId> map_point_ids;
  for (int i = 0; i < 5; i++) {
    key_frame_ids.push_back({0, i});
  }

  for (int i = 0; i < 100; i++) {
    map_point_ids.push_back({0, i});
  }
  int map_point_index = 0;
  std::map< MapPointId,FeatureId> tracking_data;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[i], FeatureId(0, i));
  }
  map_point_index = 10; 
  covisibility.UpdateWithFrameData(key_frame_ids[0],
                                        std::move(tracking_data));
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 5 + i],
                          FeatureId(0, i));
  }
  // map_point_index =20;
  covisibility.UpdateWithFrameData(key_frame_ids[1],
                                      std::move(tracking_data));

  //
  tracking_data.clear();
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 3 + i],
                          FeatureId(0, i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[2],
                                        std::move(tracking_data));
  //

  tracking_data.clear();
  map_point_index = 17;
  for (int i = 0; i < 10; i++) {
    tracking_data.emplace(map_point_ids[map_point_index - 1 + i],
                          FeatureId(0, i));
  }
  covisibility.UpdateWithFrameData(key_frame_ids[3],
                                        std::move(tracking_data));
  auto connect_key_frame =covisibility.GetConnectedKeyFrames({0, 3});                                       
 //
  
  CHECK_EQ(connect_key_frame.size(), 1);
  CHECK_EQ(connect_key_frame[0], KeyFrameId(0, 2));
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 0},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 1},{0,3}), 0);
  CHECK_EQ(covisibility.GetConnectedWeigt({0, 2},{0,3}), 1);
  //

 

  }


}  // namespace mapping
}  // namespace jarvis