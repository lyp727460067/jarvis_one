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
  int camera_num = 2;
  int win_size = 6;
  int convin_used_num = 4;
  double camera_weight = 200;
};

//
//
struct MarginalizationFactorData {
  std::vector<OdomFactor*> odom_factors;
  std::vector<IntegrationBase*> imu_factors;
  FeatureManager* feat_manager_factor;
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

  void MergeCameraData(const OptimizationStateData* opt_data,
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