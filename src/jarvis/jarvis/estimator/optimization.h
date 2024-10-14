#ifndef _JARVIS_EISITMATIOR_OPTIMIZATION_H
#define _JARVIS_EISITMATIOR_OPTIMIZATION_H
#include "feature_manager.h"
#include "jarvis/key_frame_data.h"
#include "jarvis/estimator/factor/odometry_factor.h"
#include "parameters.h"
#include "jarvis/estimator/factor/integration_base.h"
//
namespace jarvis {
namespace estimator {

struct OptimizationOption {
  std::vector<std::vector<int>> trace_sequence;
  int use_odom=0;
  double camera_weight = 377 / 1.5;
  int estimate_td=0;
  double init_td=0;
  int max_num_iterations = 1;
  double max_solver_time = 0.5;
  int estimate_extrinsic=1;
  double huber_loss =1.0;
  inline int TrackNum() const { return int(trace_sequence.size()); }
  int CamNum()const {
    int camera_num  =0;
    for (size_t i = 0; i <trace_sequence.size(); i++) {
      camera_num += trace_sequence[i].size();
    }
    return camera_num;
  }
};
class Marginalization;
//
struct OptimizationStateData {
  double** pose;
  double** speed_bias;
  double*** feature;
  double** ex_pose;
  double** ex_pose_odom;
  // double** retrive_pose;
  //后期扩展多个相机有多个dt
  double** td;
};

struct OptimizationData {
  std::vector<OdomFactor*> odom_factors;
  std::vector<IntegrationBase*> imu_factors;
  FeatureManagers* feat_manager_factors;
};
//

class Optimization {
 public:
 
  //
  Optimization(int win_size, const OptimizationOption& option);
  OptimizationStateData* Solve(Marginalization* marg,
                                     OptimizationData* frames_data);
  OptimizationStateData* MutableData() { return &data_; }
  ~Optimization();
  double FinalCost() { return final_cost_; }

 private:
  void AddCameraFactor(int id,ceres::Problem* Problem,
                       ceres::LossFunction* loss_function,
                       ceres::ParameterBlockOrdering* ordering,
                       FeatureManager* feature_managers);
  void AddFrameFactor(ceres::Problem* Problem,
                      ceres::LossFunction* loss_function,
                      ceres::ParameterBlockOrdering* ordering,
                      OptimizationData* feature_managers);

  //
//   void FrameDataToState(SlideWindowLocalData* frames_data);
//   void StateToFrameData(SlideWindowLocalData* frames_data);
  OptimizationStateData data_;

  const int win_size_;
  const OptimizationOption options_;
  double final_cost_=0; 
  int num_= 0;
  // double** para_Pose = data_.pose;
  // double** para_SpeedBias = data_.speed_bias;
  // double** para_Ex_Pose = data_.ex_pose;
  // double **para_Ex_Pose_Odom = data_.ex_pose_odom;
  // double** para_Td = data_.td;
  // double **para_Feature = data_.feature;


};
//
}  // namespace estimator
}  // namespace jarvis

#endif