#ifndef _JARVIS_ESTIMATIOR_MARGINALIZATION_H
#define _JARVIS_ESTIMATIOR_MARGINALIZATION_H
#include "jarvis/estimator/factor/integration_base.h"
#include "jarvis/estimator/factor/marginalization_factor.h"
#include "jarvis/estimator/factor/odometry_factor.h"
#include "jarvis/estimator/feature_manager.h"
#include "jarvis/estimator/optimization.h"
#include "jarvis/key_frame_data.h"
//
namespace jarvis {
namespace estimator {
//
struct MarginalizationOption {
  int win_size = 6;
  std::vector<std::vector<int>> trace_sequence;
  double camera_weight = 200;
  int camera_num=0; 
  int use_odom=0;
  double huber_loss =1.0;
  common::ThreadPool* thread_pool; 
};

//
//
struct MarginalizationFactorData {
  std::vector<OdomFactor*> odom_factors;
  std::vector<IntegrationBase*> imu_factors;
  FeatureManagers* feat_manager_factors;
  std::vector<bool> zero_velocity_factor;
  UpdataZeroVelocity* update_zero_velocity;

  // std::vector<FeatureManager*> feat_manager_factors;
};
//
class Marginalization {
 public:
  Marginalization(const MarginalizationOption&options):options_(options){}
  //
  //
  void Marginalize(const OptimizationStateData* opt_data,
                   MarginalizationFactorData* data,bool flag);
  //
  //
  void AddToProblem(ceres::Problem* problem,
                    ceres::LossFunction* loss_function) const;

 private:
  
  void MergeFrameData(const OptimizationStateData* opt_data,
                      MarginalizationFactorData* sw_data,
                      MarginalizationInfo* margina_info);

  void MergeCameraData(int id,const OptimizationStateData* opt_data,
                        FeatureManager*  feature_manager,
                       MarginalizationInfo* margina_info,
                       ceres::LossFunction*loss_function);

  //
  std::unordered_map<long, double*> ShiftStateAdrrOld(
      const OptimizationStateData* opt_data);
  std::unordered_map<long, double*> ShiftStateAdrrNew(
      const OptimizationStateData* opt_data);
  //
  std::unique_ptr<MarginalizationInfo> last_marginalization_info_;
  std::vector<double*> last_marginalization_parameter_blocks_;
  const MarginalizationOption options_;
};
}  // namespace estimator
}  // namespace jarvis
#endif