
#ifndef ESITMATOR_MAPPING_INTERNAL_PNPSOLVER_H
#define ESITMATOR_MAPPING_INTERNAL_PNPSOLVER_H

#include <opencv2/core/core.hpp>

#include "transform/transform.h"
namespace jarvis {
namespace estimator {
//

//
//
struct BetasOptimizationOption {
  int optimize_type = 0;
  int iterations_number = 3;
};
//
class GaussNewton {
 public:
  GaussNewton(Eigen::Vector4d *betas,
              const Eigen::Matrix<double, 6, 10> &l_6x10,
              const Eigen::Matrix<double, 6, 1> &control_points_norm);
  const Eigen::Matrix<double, 6, 4> &Jacobi() const { return Jacobi_; }
  const Eigen::Matrix<double, 6, 1> B() const { return b_; }
  void IterateOnce();
  ~GaussNewton() {}

 private:
  //
  std::pair<Eigen::Matrix<double, 6, 4>, Eigen::Matrix<double, 6, 1>>
  ComputeAAndBGaussNewton(
      const Eigen::Matrix<double, 6, 10> &l_6x10,
      const Eigen::Matrix<double, 6, 1> &control_points_norm,
      const Eigen::Vector4d &betas);
  //
  void QrSolve();
  //
  Eigen::Matrix<double, 6, 4> Jacobi_;
  Eigen::Matrix<double, 6, 1> b_;
  Eigen::Vector4d *betas_;
};

//

class BetasOptimization {
 public:
  BetasOptimization(const BetasOptimizationOption &option,
                    Eigen::Vector4d *betas,
                    const Eigen::Matrix<double, 6, 10> &l_6x10,
                    const Eigen::Matrix<double, 6, 1> control_points_norm);
  void operator()();
  ~BetasOptimization() {}

 private:
  std::unique_ptr<GaussNewton> gauss_newton_ = nullptr;
  const BetasOptimizationOption options_;
  Eigen::Vector4d *betas_ = nullptr;
  const Eigen::Matrix<double, 6, 10> &l_6x10_;
  const Eigen::Matrix<double, 6, 1> &control_points_norm_;
};
//

struct BetasRequiresProcess {
  Eigen::MatrixXd FillM(const std::vector<Eigen::Vector4d> &alphas,
                        const std::vector<Eigen::Vector2d> &key_points) const;
  Eigen::Matrix<double, 6, 10> ComputeL6x10(
      const Eigen::Matrix<double, 12, 4> &eigen_value) const;
  Eigen::Vector4d FindBetasApprox1(
      const Eigen::Matrix<double, 6, 10> &l,
      const Eigen::Matrix<double, 6, 1> &rho) const;
  Eigen::Vector4d FindBetasApprox2(
      const Eigen::Matrix<double, 6, 10> &l,
      const Eigen::Matrix<double, 6, 1> &rho) const;
  Eigen::Vector4d FindBetasApprox3(
      const Eigen::Matrix<double, 6, 10> &l,
      const Eigen::Matrix<double, 6, 1> &rho) const;
};
class HandlePoint;
class BetasProcess {
 public:
  BetasProcess(const std::vector<Eigen::Vector2d> &key_points,
               const HandlePoint *const handle_point);
  const BetasProcess *SetApprox(const int &approx) {
    approx_ = approx;
    return this;
  }
  std::vector<Eigen::Vector3d> CameraPoints() const;
  //
  const std::vector<Eigen::Vector2d> &KeyPoints() { return key_points_; }
  ~BetasProcess() {}

 private:
  const std::vector<Eigen::Vector2d> key_points_;
  std::unique_ptr<const BetasRequiresProcess> betas_requires_process_ = nullptr;
  const HandlePoint *const handle_point_ = nullptr;
  Eigen::Matrix<double, 6, 10> l_6x10_;
  Eigen::Matrix<double, 12, 4> m_svd_u_matrix_;
  int approx_ = 0;
};
//

class HandlePoint {
 public:
  HandlePoint(const std::vector<Eigen::Vector3d> &points);
  //
  const Eigen::Matrix<double, 6, 1> &ControlNormal() const {
    return control_normal_;
  }
  const std::vector<Eigen::Vector4d> &Alphas() const { return alphas_; }
  const std::vector<Eigen::Vector3d> &Points() const { return points_; }

 private:
  Eigen::Matrix<double, 6, 1> ComputeControlNormal(
      const std::array<Eigen::Vector3d, 4> &points);
  std::array<Eigen::Vector3d, 4> ComputeControlPoints();
  std::vector<Eigen::Vector4d> ComputeBarycentricCoordinates(
      const std::vector<Eigen::Vector3d> &points,
      const std::array<Eigen::Vector3d, 4> &cc);
  //
  const std::vector<Eigen::Vector3d> points_;
  std::vector<Eigen::Vector4d> alphas_;
  Eigen::Matrix<double, 6, 1> control_normal_;
  static int constexpr kMinPointsSize_ = 4;
};

//

class PnpSolverInterface;
struct PnPsolverResult {
  transform::Rigid3d pose;
  std::vector<bool> inliers;
  std::unique_ptr<PnpSolverInterface> extend;
};
struct PnpSolverOption {
  int min_inliers = 10;
  double probability = 0.99;
  int max_iterations = 100;
  int iterations = 10;
  int min_set = 8;
  float epsilon = 0.7;
  float th2 = 0.017570423;
};
class PnpSolverInterface {
 public:
  virtual std::unique_ptr<PnPsolverResult> Solve(
      const std::vector<Eigen::Vector3d>& points,
      const std::vector<Eigen::Vector2d>& key_points,
      const transform::Rigid3d& init_pose) = 0;
 virtual  std::unique_ptr<PnPsolverResult> Find()=0;
  virtual ~PnpSolverInterface() {}
};



class PnpSolver : public PnpSolverInterface {
 public:
  PnpSolver(const PnpSolverOption &option,
            const std::vector<Eigen::Vector3d> &points =
                std::vector<Eigen::Vector3d>(),
            const std::vector<Eigen::Vector2d> &key_points =
                std::vector<Eigen::Vector2d>());
  //
  std::unique_ptr<PnPsolverResult> Find();
  std::unique_ptr<PnPsolverResult> Solve(
      const std::vector<Eigen::Vector3d> &points,
      const std::vector<Eigen::Vector2d> &key_points,
      const transform::Rigid3d &init_pose) override;
  //
  std::unique_ptr<PnPsolverResult> operator()(
      const transform::Rigid3d &init_pose);

  double ReprojectionError(const transform::Rigid3d &pose);

  std::vector<bool> CheckInliers(const transform::Rigid3d &pose);

  //
  transform::Rigid3d IcpComputePose(const std::vector<Eigen::Vector3d> &p1,
                                    const std::vector<Eigen::Vector3d> &p2);

 private:
  //
  std::pair<transform::Rigid3d, std::vector<bool>> CoarseComputePose();
  void RestMinPoints(const std::vector<bool> &best_inliers);
  std::unique_ptr<PnPsolverResult> Refine(
      const std::vector<bool> &best_inliers);
  transform::Rigid3d ComputePose();
  //
  PnpSolverOption options_;
  const std::vector<Eigen::Vector3d> &points_;
  const std::vector<Eigen::Vector2d> &key_points_;
  //
  std::unique_ptr<HandlePoint> handle_points_ = nullptr;
  std::unique_ptr<BetasProcess> betas_process_ = nullptr;
  //
  std::vector<int> points_index_;
  int max_iterations = 0;
};
}  // namespace estimator
}  // namespace jarvis

#endif
