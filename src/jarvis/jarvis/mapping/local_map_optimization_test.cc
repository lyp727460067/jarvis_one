// #include <fstream>
// #include <iostream>
// #include <iomanip>
// #include <string>
// #include <cstdlib>

// #include "jarvis/mapping/local_map_optimization.h"
// #include "jarvis/mapping/key_point_exract.h"

// using namespace jarvis::mapping;

// namespace jarvis {
// namespace mapping {

// struct DataPath {
//     std::string timestamp;
//     std::string rgb_path;
//     std::string depth_path;
// };

// struct PoseWithTime {
//     std::string timestamp;
//     Eigen::Quaterniond q;
//     Eigen::Vector3d t;
// };

// struct FrameData{
//   PoseWithTime pose;
//   std::vector<cv::KeyPoint> keypoints;
//   Descriptors descriptors;
// };

// class LocalMapOptimizationTest {
// public:
//     LocalMapOptimizationTest() {
//         // des_extractor_ =
//         //     std::make_unique<DescriptorExtract>(DescriptorExtractOption{});
//         // key_extractor_ = std::make_unique<KeyPointExtract>(KeyPointExtract{});
//         // matcher_ = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
//         // LocalMapOptimizationOption option;
//         // option.extric_camera_to_imu.push_back(transform::Rigid3d());
//         // optimizer_ = std::make_unique<LocalMapOptimization>(option);
//         // isInited = false;
//     }

//     void OneStep(const std::string time, const cv::Mat &gray_img, const cv::Mat &depth_img,
//             PoseWithTime &cam_pose, int img_id) {
//         // // 提取keypoint和descriptor
//         // KeyFrameId kf_id(0, img_id);
//         // LocalMapOptimizationData::FrameData frame_data;
//         // frame_data.time = time;
//         // // frame_data.pose = transform::Rigid3d(cam_pose.t, cam_pose.q);
//         // double randomNum1 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//         // double randomNum2 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//         // double randomNum3 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//         // Eigen::Vector3d t_noise(
//         //     cam_pose.t.x() + randomNum1 * 0.02, cam_pose.t.y() + randomNum2 * 0.02, cam_pose.t.z() + randomNum3*0.01);
//         // frame_data.pose = transform::Rigid3d(t_noise, cam_pose.q);
//         // all_data.frame_datas[kf_id] = frame_data;

//         // std::vector<cv::KeyPoint> key_points = key_extractor_->Extract(gray_img);

//         // Descriptors descriptors = des_extractor_->Extract(gray_img, key_points);
//         // cv::Mat descriptors_mat = BriefToCvMat(descriptors);

//         // // std::cout << "show result" << std::endl;
//         // // cv::Mat keypoint_img;
//         // // cv::drawKeypoints(gray_img, key_points, keypoint_img, cv::Scalar::all(-1),
//         // //                   cv::DrawMatchesFlags::DEFAULT);
//         // // cv::namedWindow("keypoints", cv::WINDOW_AUTOSIZE);
//         // // cv::imshow("keypoints", keypoint_img);
//         // // cv::waitKey(0);

//         // // 记录feature
//         // std::vector<FeatureId> feature_ids;
//         // feature_ids.reserve(key_points.size());
//         // for(size_t i = 0; i < key_points.size(); i++){
//         //     FeatureId f_id(0, feature_id);
//         //     feature_id ++;
//         //     feature_ids.push_back(f_id);
//         //     FeatureData* feature_ptr = new FeatureData();
//         //     feature_ptr->key_point = key_points[i];

//         //     // 归一化
//         //     feature_ptr->f.x() = (key_points[i].pt.x - cx) / fx;
//         //     feature_ptr->f.y() = (key_points[i].pt.y - cy) / fy;
//         //     feature_ptr->f.z() = 1.0;

//         //     all_data.feature_datas[f_id] = feature_ptr;
//         // }

//         // std::map<int, MapPointId> kp_to_mp_map;
//         // if (!isInited) { // 第一帧
//         //     isInited = true;
//         // } else {
//         //     // 特征匹配
//         //     std::vector<std::vector<cv::DMatch>> matches;
//         //     std::vector<cv::DMatch> good_matches;
//         //     float ratio_th = 0.7;
//         //     // matcher_->match(descriptors_last, descriptors_mat, good_matches);
//         //     matcher_->knnMatch(descriptors_last, descriptors_mat, matches, 2);
//         //     std::map<int, int> keypoint_id_map; // current keypoints match to last keypoints
//         //     for (auto &match : matches) {
//         //         if (match[0].distance < ratio_th * match[1].distance) {
//         //             good_matches.push_back(match[0]);
//         //             keypoint_id_map[match[0].trainIdx] = match[0].queryIdx;
//         //         }
//         //     }

//         //     // visualization
//         //     // std::vector<cv::KeyPoint> match_points_last, match_points;
//         //     // for (size_t i = 0; i < good_matches.size(); i++) {
//         //     //     match_points_last.push_back(key_points_last[good_matches[i].queryIdx]);
//         //     //     match_points.push_back(key_points[good_matches[i].trainIdx]);
//         //     // }
//         //     // cv::Mat image_matches_bf, image_points_last, image_points;
//         //     // cv::drawKeypoints(img_last, match_points_last, image_points_last, cv::Scalar::all(-1),
//         //     //                   cv::DrawMatchesFlags::DEFAULT);
//         //     // cv::drawKeypoints(gray_img, match_points, image_points, cv::Scalar::all(-1),
//         //     //                   cv::DrawMatchesFlags::DEFAULT);
//         //     // cv::drawMatches(img_last, key_points_last, gray_img, key_points, good_matches, image_matches_bf);
//         //     // cv::imshow("bf_matches", image_matches_bf);
//         //     // cv::imshow("keypoints last", image_points_last);
//         //     // cv::imshow("keypoints now", image_points);
//         //     // cv::waitKey(0);

//         //     // 计算地图点及其共视关系
//         //     for (auto &pair : keypoint_id_map) {
//         //         if (kp_to_mp_map_last.find(pair.second) != kp_to_mp_map_last.end()) {
//         //             // 为地图点增加共视特征
//         //             all_data.con_map_points[kp_to_mp_map_last[pair.second]].con_frame_datas.insert(
//         //                 std::make_pair(kf_id, feature_ids[pair.first]));
//         //             // if(all_data.con_map_points[kp_to_mp_map_last[pair.second]].con_frame_datas.size() > 2){
//         //             //     std::cout << "con map point: " << kp_to_mp_map_last[pair.second].index << ": "
//         //             //               << all_data.con_map_points[kp_to_mp_map_last[pair.second]].con_frame_datas.size() << std::endl;
//         //             // }
//         //             // 记录keypoint id到地图点的映射
//         //             kp_to_mp_map[pair.first] = kp_to_mp_map_last[pair.second];
//         //         } else {
//         //             // 计算地图点坐标
//         //             LocalMapOptimizationData::MapPointData map_point;
//         //             uint16_t depth = depth_img.at<uint16_t>(
//         //                 key_points[pair.first].pt.x, key_points[pair.first].pt.y);
//         //             if (depth == 0) continue;

//         //             // 相机坐标系下
//         //             FeatureData* fd_ptr = all_data.feature_datas[feature_ids[pair.first]];
//         //             Eigen::Vector3d p_cam;
//         //             p_cam.z() = static_cast<double>(depth) / scale_factor;
//         //             p_cam.x() = fd_ptr->f.x() * p_cam.z();
//         //             p_cam.y() = fd_ptr->f.y() * p_cam.z();

//         //             // 投影到世界坐标系
//         //             map_point.pos = cam_pose.q * p_cam + cam_pose.t;

//         //             // 增加共视关系
//         //             map_point.con_frame_datas.insert(std::make_pair(kf_id, feature_ids[pair.first]));

//         //             // 记录地图点
//         //             MapPointId mp_id(0, map_point_id);
//         //             map_point_id++;
//         //             all_data.con_map_points[mp_id] = map_point;
//         //             kp_to_mp_map[pair.first] = mp_id;
//         //         }
//         //     }
//         // }

//         // // std::cout << "all data info: " << std::endl;
//         // // std::cout << "frame size: " << all_data.frame_datas.size() << std::endl;
//         // // std::cout << "feature size: " << all_data.feature_datas.size() << std::endl;
//         // // std::cout << "map point size: " << all_data.con_map_points.size() << std::endl;

//         // // 使用move原有左方內存会首先被释放再转移,gpt说的，不知道对不对
//         // key_points_last = std::move(key_points);
//         // descriptors_last = std::move(descriptors_mat);
//         // img_last = std::move(gray_img);
//         // kp_to_mp_map_last = std::move(kp_to_mp_map);
//     }

//     void Optimize(){
//         // optimizer_->Optimize(&all_data);
//     }

//     void SavePose(std::string save_path){
//         // std::ofstream output_file(save_path);

//         // if(output_file.is_open()){
//         //     for(auto &frame:all_data.frame_datas){
//         //         Eigen::Vector3d t = frame.second.pose.translation();
//         //         Eigen::Quaterniond q = frame.second.pose.rotation();
//         //         output_file << std::fixed << std::setprecision(4) << frame.second.time << " "
//         //                     << t.x() << " " << t.y() <<  " " << t.z() << " " << q.x()
//         //                     << " " << q.y() << " " << q.z() << std::endl;
//         //     }
//         // }else{
//         //     std::cout << "output file open error" << std::endl;
//         // }
//     }

// private:
//     std::unique_ptr<KeyPointExtract> key_extractor_;
//     std::unique_ptr<DescriptorExtract> des_extractor_;
//     cv::Ptr<cv::DescriptorMatcher> matcher_;
//     std::unique_ptr<LocalMapOptimization> optimizer_;
//     LocalMapOptimizationData all_data;
//     int feature_id = 0;
//     int map_point_id = 0;
//     const double fx = 525.0;
//     const double fy = 525.0;
//     const double cx = 319.5;
//     const double cy = 239.5;
//     const double scale_factor = 5000.0;

//     std::vector<cv::KeyPoint> key_points_last;
//     std::map<int, MapPointId> kp_to_mp_map_last;
//     cv::Mat descriptors_last;
//     cv::Mat img_last;
//     bool isInited = false;
// };

// void GetDataPaths(std::ifstream &file, std::vector<DataPath> &data_paths) {
//     std::string line;
//     while (std::getline(file, line)) {
//         std::istringstream iss(line);
//         std::string time1, rgb_path, time2, depth_path;
//         iss >> time1 >> rgb_path >> time2 >> depth_path;

//         DataPath data_path = {time1, rgb_path, depth_path};
//         data_paths.push_back(data_path);
//     }
//     std::cout << "paths size: " << data_paths.size() << std::endl;
// }

// void GetGtPoses(std::ifstream &file, std::vector<PoseWithTime> &gt_poses) {
//     std::string line;
//     int count = 0;
//     while (std::getline(file, line)) {
//         // 前三行是资料不是数据,需要跳过
//         if (count < 3) {
//             count++;
//             continue;
//         }
//         std::istringstream iss(line);
//         std::string time, tx, ty, tz, qx, qy, qz, qw;
//         iss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;

//         Eigen::Quaterniond q(std::stod(qw), std::stod(qx), std::stod(qy), std::stod(qz));
//         Eigen::Vector3d t(std::stod(tx), std::stod(ty), std::stod(tz));
//         PoseWithTime pose = {time, q, t};
//         gt_poses.push_back(pose);
//     }
//     std::cout << "groundtruth size: " << gt_poses.size() << std::endl;
// }

// bool GetCamPose(const std::string time, const std::vector<PoseWithTime> &gt_poses,
//                 PoseWithTime &cam_pose, int &id) {
//     for (; id < gt_poses.size(); id++) {
//         if (time < gt_poses[id].timestamp) {
//             id--;
//             break;
//         }
//     }

//     if (id < 0 || id == gt_poses.size() - 1)
//         return false;

//     // int可以用到2038年
//     int pre_second = std::stoi(gt_poses[id].timestamp.substr(0, 10));
//     int pre_psecond = std::stoi(gt_poses[id].timestamp.substr(11, 4));
//     int next_second = std::stoi(gt_poses[id + 1].timestamp.substr(0, 10)) - pre_second;
//     int next_psecond = std::stoi(gt_poses[id + 1].timestamp.substr(11, 4)) - pre_psecond;
//     int inter_second = std::stoi(time.substr(0, 10)) - pre_second;
//     int inter_psecond = std::stoi(time.substr(11, 4)) - pre_psecond;

//     // std::cout << "pre time: " << gt_poses[id].timestamp << std::endl;
//     // std::cout << "next time: " << gt_poses[id+1].timestamp << std::endl;
//     // std::cout << "inter time: " << time << std::endl;
//     // std::cout << "inter time: " << inter_second << "." << inter_psecond << std::endl;
//     // std::cout << "next time: " << next_second << "." << next_psecond << std::endl;

//     double ratio =
//         (static_cast<double>(inter_second) + static_cast<double>(inter_psecond) / 10e4) /
//         (static_cast<double>(next_second) + static_cast<double>(next_psecond) / 10e4);

//     // std::cout << "inter info" << std::endl;
//     // std::cout << "ratio: " << ratio << std::endl;
//     // std::cout << "pre pose: " << gt_poses[id].t.x() << ", " << gt_poses[id].t.y() << ", "
//     //           << gt_poses[id].t.z() << ", " << gt_poses[id].q.x() << ", " << gt_poses[id].q.y()
//     //           << ", " << gt_poses[id].q.z() << ", " << gt_poses[id].q.w() << std::endl;
//     // std::cout << "next pose: " << gt_poses[id+1].t.x() << ", " << gt_poses[id+1].t.y() << ", "
//     //           << gt_poses[id+1].t.z() << ", " << gt_poses[id+1].q.x() << ", " << gt_poses[id+1].q.y()
//     //           << ", " << gt_poses[id+1].q.z() << ", " << gt_poses[id+1].q.w() << std::endl;

//     Eigen::Quaterniond q_inter = gt_poses[id].q.slerp(ratio, gt_poses[id + 1].q);
//     Eigen::Vector3d t_inter = (1 - ratio) * gt_poses[id].t + ratio * gt_poses[id + 1].t;

//     cam_pose.timestamp = time;
//     cam_pose.q = q_inter;
//     cam_pose.t = t_inter;

//     return true;
// }

// } // namespace mapping
// } // namespace jarvis


// int main(int argc, char *argv[]) {
//     if (argc <= 2) {
//         std::cout << "please input data folder and data size" << std::endl;
//     }

//     const std::string data_folder(argv[1]);
//     const std::string associations_path = data_folder + "/associations.txt";
//     const std::string gt_path = data_folder + "/groundtruth.txt";
//     const std::string output_file = data_folder + "/optimized_pose.txt";

//     srand(0);

//     const std::string data_num_str(argv[2]);
//     int data_num = std::stoi(data_num_str);

//     std::cout << "data_folder: " << data_folder << std::endl;
//     std::cout << "associations_path: " << associations_path << std::endl;
//     std::cout << "gt_path: " << gt_path << std::endl;

//     std::ifstream associations_file(associations_path);
//     if (!associations_file.is_open()) {
//         std::cout << "open associations file failed!" << std::endl;
//         return 0;
//     }

//     std::ifstream gt_file(gt_path);
//     if (!gt_file.is_open()) {
//         std::cout << "open groundtruth file failed!" << std::endl;
//         return 0;
//     }

//     // 读取对应rgb和depth图片名
//     std::vector<DataPath> all_data_paths;
//     GetDataPaths(associations_file, all_data_paths);

//     // 读取真值Pose
//     std::vector<PoseWithTime> all_gt_poses;
//     GetGtPoses(gt_file, all_gt_poses);

//     data_num = (all_data_paths.size() > data_num) ? data_num : all_data_paths.size();
//     LocalMapOptimizationTest test_manager;
//     // 读取图像并计算
//     int gt_id = 0;
//     for (int i = 0; i < data_num; i++) {
//       std::string rgb_path = data_folder + "/" + all_data_paths[i].rgb_path;
//       std::string depth_path = data_folder + "/" + all_data_paths[i].depth_path;

//       cv::Mat rgb_img = cv::imread(rgb_path);
//       cv::Mat depth_img = cv::imread(depth_path, cv::IMREAD_UNCHANGED);

//       //
//       cv::Mat gray_img;
//       cv::extractChannel(rgb_img, gray_img, 1); // 取出绿色通道

//     //   std::cout << "rgb data: " << rgb_img.size() << ", " << rgb_img.channels() << std::endl;
//     //   std::cout << "depth data: " << depth_img.size() << ", " << depth_img.channels() << std::endl;
//     //   std::cout << "rgb type: " << rgb_img.type() << std::endl;
//     //   std::cout << "gray type: " << gray_img.type() << std::endl;
//     //   std::cout << "depth type: " << depth_img.type() << std::endl;
      
//       PoseWithTime cur_pose;
//       if(!GetCamPose(all_data_paths[i].timestamp, all_gt_poses, cur_pose, gt_id))
//         break;
//       // -1到1的隨机数
//     //   double randomNum1 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//     //   double randomNum2 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//     //   double randomNum3 = -1 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX/2));
//     //   std::cout << "before: " << std::endl;
//     //   std::cout << cur_pose.t << std::endl;
//     //   cur_pose.t.x() += randomNum1 * 0.05;
//     //   cur_pose.t.y() += randomNum2 * 0.05;
//     //   cur_pose.t.z() += randomNum3 * 0.01;
//     //   std::cout << "after: " << std::endl;
//     //   std::cout << cur_pose.t << std::endl;
      
//     //   std::cout << "inter pose: " << cur_pose.t << ", " << cur_pose.q << std::endl;
      
//     //   if(i > 10) break;

//       test_manager.OneStep(all_data_paths[i].timestamp, gray_img, depth_img, cur_pose, i);
//     }

//     test_manager.SavePose(data_folder+"/pose_noise.txt");
//     test_manager.Optimize();
//     test_manager.SavePose(output_file);

//     return 0;
// }
