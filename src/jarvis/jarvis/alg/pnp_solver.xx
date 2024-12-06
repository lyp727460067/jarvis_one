#include "pnp_solver.h"

#include <opencv2/core/types_c.h>
#include <opencv2/highgui/highgui_c.h>

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <random>
#include <vector>

#include "DUtils/Random.h"
#include "glog/logging.h"
namespace jarvis {
namespace alg {
//
namespace {
//
Eigen::MatrixXd StdVectorToEigen(const std::vector<Eigen::Vector3d> &vectors) {
  CHECK(!vectors.empty());
  Eigen::MatrixXd result(vectors.size(), 3);
  for (size_t i = 0; i < vectors.size(); i++) {
    result.block<1, 3>(i, 0) = vectors[i];
  }
  return result;
}
//
std::vector<Eigen::Vector3d> MinusPoints(
    const std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &point) {
  std::vector<Eigen::Vector3d> result(points.size());
  std::transform(points.begin(), points.end(), result.begin(),
                 [&](const Eigen::Vector3d &p) { return p - point; });
  return result;
}
//
Eigen::Vector3d MeanPoints(const std::vector<Eigen::Vector3d> &points) {
  Eigen::Vector3d points_sum = Eigen::Vector3d::Zero();
  for (const auto &point : points) {
    points_sum += point;
  }

  return points_sum / points.size();
}

// 利用Eigen库，采用SVD分解的方法求解矩阵伪逆，默认误差er为0
Eigen::MatrixXd PinvEigenBased(const Eigen::MatrixXd &origin,
                               const float er = 0) {
  // 进行svd分解
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(
      origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // 构建SVD分解结果
  Eigen::MatrixXd U = svd_holder.matrixU();
  Eigen::MatrixXd V = svd_holder.matrixV();
  Eigen::MatrixXd D = svd_holder.singularValues();

  // 构建S矩阵
  Eigen::MatrixXd S(V.cols(), U.cols());
  S.setZero();
  for (unsigned int i = 0; i < D.size(); ++i) {
    if (D(i, 0) > er) {
      S(i, i) = 1 / D(i, 0);
    } else {
      S(i, i) = 0;
    }
  }

  // pinv_matrix = V * S * U^T
  return V * S * U.transpose();
}

//
}  // namespace
//

std::pair<Eigen::Matrix<double, 6, 4>, Eigen::Matrix<double, 6, 1>>
GaussNewton::ComputeAAndBGaussNewton(
    const Eigen::Matrix<double, 6, 10> &l_6x10_1,
    const Eigen::Matrix<double, 6, 1> &control_points_norm,
    const Eigen::Vector4d &betas) {
  //
  const Eigen::Matrix<double, 6, 10, Eigen::RowMajor> &l_6x10 = l_6x10_1;
  Eigen::Matrix<double, 6, 4> A = Eigen::Matrix<double, 6, 4>::Zero();
  Eigen::Matrix<double, 6, 1> b = Eigen::Matrix<double, 6, 1>::Zero();
  for (int i = 0; i < 6; i++) {
    //
    const double *rowL = l_6x10.data() + i * 10;
    //

    Eigen::Vector4d row;
    row << 2 * rowL[0] * betas[0] + rowL[1] * betas[1] + rowL[3] * betas[2] +
               rowL[6] * betas[3],
        rowL[1] * betas[0] + 2 * rowL[2] * betas[1] + rowL[4] * betas[2] +
            rowL[7] * betas[3],
        rowL[3] * betas[0] + rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +
            rowL[8] * betas[3],
        rowL[6] * betas[0] + rowL[7] * betas[1] + rowL[8] * betas[2] +
            2 * rowL[9] * betas[3];
    A.row(i) = row;

    b[i] = rowL[0] * betas[0] * betas[0] + rowL[1] * betas[0] * betas[1] +
           rowL[2] * betas[1] * betas[1] + rowL[3] * betas[0] * betas[2] +
           rowL[4] * betas[1] * betas[2] + rowL[5] * betas[2] * betas[2] +
           rowL[6] * betas[0] * betas[3] + rowL[7] * betas[1] * betas[3] +
           rowL[8] * betas[2] * betas[3] + rowL[9] * betas[3] * betas[3];
    b[i] = control_points_norm[i] - b[i];
  }
  return {A, b};
}
//
GaussNewton::GaussNewton(Eigen::Vector4d *betas,
                         const Eigen::Matrix<double, 6, 10> &l_6x10,
                         const Eigen::Matrix<double, 6, 1> &control_points_norm)
    : betas_(betas) {
  CHECK_NOTNULL(betas_);
  std::tie(Jacobi_, b_) =
      ComputeAAndBGaussNewton(l_6x10, control_points_norm, *betas_);
}

void GaussNewton::IterateOnce() { QrSolve(); }

void GaussNewton::QrSolve() {
  //
  int max_nr = 0;
  double *A1, *A2;
  const int nr = Jacobi_.rows();
  const int nc = Jacobi_.cols();
  A1 = new double[nr];
  A2 = new double[nr];
  if (A1 == 0 || A2 == 0) {
    if (A1) {
      delete[] A1;
    }
    if (A2) {
      delete[] A2;
    }
    return;
  }

  // double *pA = A->data.db, *ppAkk = pA;

  const Eigen::Matrix<double, 6, 4, Eigen::RowMajor> &jacobi_temp = Jacobi_;
  double *pA = const_cast<double *>(jacobi_temp.data()), *ppAkk = pA;
  for (int k = 0; k < nc; k++) {
    double *ppAik = ppAkk, eta = fabs(*ppAik);
    for (int i = k + 1; i < nr; i++) {
      double elt = fabs(*ppAik);
      if (eta < elt) eta = elt;
      ppAik += nc;
    }

    if (eta == 0) {
      A1[k] = A2[k] = 0.0;
      LOG(ERROR) << "God damnit, A is singular, this shouldn't happen.";
      return;
    } else {
      double *ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for (int i = k; i < nr; i++) {
        *ppAik *= inv_eta;
        sum += *ppAik * *ppAik;
        ppAik += nc;
      }
      double sigma = sqrt(sum);
      if (*ppAkk < 0) sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;
      for (int j = k + 1; j < nc; j++) {
        double *ppAik = ppAkk, sum = 0;
        for (int i = k; i < nr; i++) {
          sum += *ppAik * ppAik[j - k];
          ppAik += nc;
        }
        double tau = sum / A1[k];
        ppAik = ppAkk;
        for (int i = k; i < nr; i++) {
          ppAik[j - k] -= tau * *ppAik;
          ppAik += nc;
        }
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b

  double *ppAjj = pA, *pb = b_.data();
  for (int j = 0; j < nc; j++) {
    double *ppAij = ppAjj, tau = 0;
    for (int i = j; i < nr; i++) {
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for (int i = j; i < nr; i++) {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  Eigen::Vector4d X = Eigen::Vector4d::Zero();
  double *pX = X.data();
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for (int i = nc - 2; i >= 0; i--) {
    double *ppAij = pA + i * nc + (i + 1), sum = 0;

    for (int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }

  *betas_ += X;
  delete[] A1;
  delete[] A2;
}

//
BetasOptimization::BetasOptimization(
    const BetasOptimizationOption &option, Eigen::Vector4d *betas,
    const Eigen::Matrix<double, 6, 10> &l_6x10,
    const Eigen::Matrix<double, 6, 1> control_points_norm)
    : options_(option),
      betas_(betas),
      l_6x10_(l_6x10),
      control_points_norm_(control_points_norm) {
  CHECK_NOTNULL(betas_);
}

//
void BetasOptimization::operator()() {
  for (int i = 0; i < options_.iterations_number; i++) {
    if (options_.optimize_type == 0) {
      gauss_newton_ =
          std::make_unique<GaussNewton>(betas_, l_6x10_, control_points_norm_);
      gauss_newton_->IterateOnce();
    }
  }
}
//

Eigen::MatrixXd BetasRequiresProcess::FillM(
    const std::vector<Eigen::Vector4d> &alphas,
    const std::vector<Eigen::Vector2d> &key_points) const {
  //
  CHECK_EQ(alphas.size(), key_points.size());
  //
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(key_points.size() * 2, 12);
  for (size_t i = 0; i < key_points.size(); i++) {
    const auto u = key_points[i].x();
    const auto v = key_points[i].y();
    std::vector<Eigen::Matrix<double, 2, 3>> m_portion(4);
    for (int j = 0; j < 4; j++) {
      m_portion[j] << alphas[i][j], 0.0f, -alphas[i][j] * u, 0.0f, alphas[i][j],
          -alphas[i][j] * v;
    }

    Eigen::Matrix<double, 2, 12> tmp;
    tmp << m_portion[0], m_portion[1], m_portion[2], m_portion[3];
    matrix.block<2, 12>(i * 2, 0) = tmp;
  }
  return matrix;
}
//

//

Eigen::Matrix<double, 6, 10> BetasRequiresProcess::ComputeL6x10(
    const Eigen::Matrix<double, 12, 4> &eigen_value) const {
  //
  std::array<Eigen::Matrix<double, 3, 4>, 6> v;
  v[0] = eigen_value.block<3, 4>(0, 0) - eigen_value.block<3, 4>(3, 0);
  //
  v[1] = eigen_value.block<3, 4>(0, 0) - eigen_value.block<3, 4>(6, 0);
  v[2] = eigen_value.block<3, 4>(0, 0) - eigen_value.block<3, 4>(9, 0);
  v[3] = eigen_value.block<3, 4>(3, 0) - eigen_value.block<3, 4>(6, 0);
  v[4] = eigen_value.block<3, 4>(3, 0) - eigen_value.block<3, 4>(9, 0);
  v[5] = eigen_value.block<3, 4>(6, 0) - eigen_value.block<3, 4>(9, 0);
  //
  Eigen::Matrix<double, 6, 10> result;
  for (int i = 0; i < 6; i++) {
    result(i, 0) = v[i].block<3, 1>(0, 0).squaredNorm();
    result(i, 1) = 2.0f * v[i].block<3, 1>(0, 0).dot(v[i].block<3, 1>(0, 1));
    result(i, 2) = v[i].block<3, 1>(0, 1).squaredNorm();
    result(i, 3) = 2.0f * v[i].block<3, 1>(0, 0).dot(v[i].block<3, 1>(0, 2));
    result(i, 4) = 2.0f * v[i].block<3, 1>(0, 1).dot(v[i].block<3, 1>(0, 2));
    //
    result(i, 5) = v[i].block<3, 1>(0, 2).squaredNorm();
    //
    result(i, 6) = 2.0f * v[i].block<3, 1>(0, 0).dot(v[i].block<3, 1>(0, 3));
    result(i, 7) = 2.0f * v[i].block<3, 1>(0, 1).dot(v[i].block<3, 1>(0, 3));
    result(i, 8) = 2.0f * v[i].block<3, 1>(0, 2).dot(v[i].block<3, 1>(0, 3));
    result(i, 9) = v[i].block<3, 1>(0, 3).squaredNorm();
  }
  return result;
}
// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]
Eigen::Vector4d BetasRequiresProcess::FindBetasApprox1(
    const Eigen::Matrix<double, 6, 10> &l,
    const Eigen::Matrix<double, 6, 1> &rho) const {
  Eigen::Matrix<double, 6, 4> l_partion;
  l_partion << l.col(0), l.col(1), l.col(3), l.col(6);
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> svd(
      l_partion, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto betas = svd.solve(rho);
  CHECK(betas.norm() != 0.0);
  Eigen::Vector4d result;

  result[0] = sqrt(fabs(betas[0]));
  result[1] = betas[1] / betas[0];
  result[2] = betas[2] / betas[0];
  result[3] = betas[3] / betas[0];
  if (betas[0] < 0) {
    result.tail(3) = -result.tail(3);
  }
  return result;
}
// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22

Eigen::Vector4d BetasRequiresProcess::FindBetasApprox2(
    const Eigen::Matrix<double, 6, 10> &l,
    const Eigen::Matrix<double, 6, 1> &rho) const {
  Eigen::Matrix<double, 6, 3> l_partion;
  l_partion << l.col(0), l.col(1), l.col(2);

  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 3>> svd(
      l_partion, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto betas = svd.solve(rho);

  Eigen::Vector4d result;
  if (betas[0] < 0) {
    result[0] = sqrt(-betas[0]);
    result[1] = (betas[2] < 0) ? sqrt(-betas[2]) : 0.0;
  } else {
    result[0] = sqrt(betas[0]);
    result[1] = (betas[2] > 0) ? sqrt(betas[2]) : 0.0;
  }
  if (betas[1] < 0) {
    result[0] = -result[0];
  }

  result[2] = 0.0;
  result[3] = 0.0;
  return result;
}
//
// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

Eigen::Vector4d BetasRequiresProcess::FindBetasApprox3(
    const Eigen::Matrix<double, 6, 10> &l,
    const Eigen::Matrix<double, 6, 1> &rho) const {
  Eigen::Matrix<double, 6, 5> l_partion;
  l_partion << l.col(0), l.col(1), l.col(2), l.col(3), l.col(4);
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 5>> svd(
      l_partion, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto betas = svd.solve(rho);
  Eigen::Vector4d result;
  if (betas[0] < 0) {
    result[0] = sqrt(-betas[0]);
    result[1] = (betas[2] < 0) ? sqrt(-betas[2]) : 0.0;
  } else {
    result[0] = sqrt(betas[0]);
    result[1] = (betas[2] > 0) ? sqrt(betas[2]) : 0.0;
  }
  if (betas[1] < 0) result[0] = -result[0];
  result[2] = betas[3] / result[0];
  result[3] = 0.0;
  return result;
}

//
BetasProcess::BetasProcess(const std::vector<Eigen::Vector2d> &key_points,
                           const HandlePoint *const handle_point)
    : key_points_(key_points),
      handle_point_(handle_point),
      betas_requires_process_(std::make_unique<const BetasRequiresProcess>()) {
  CHECK_NOTNULL(handle_point);
  CHECK_EQ(handle_point_->Alphas().size(), key_points_.size());
  //
  const Eigen::MatrixXd m =
      betas_requires_process_->FillM(handle_point_->Alphas(), key_points_);
  //

  Eigen::Matrix<double, 12, 12, Eigen::RowMajor> mtm_eigen = m.transpose() * m;
  Eigen::JacobiSVD<Eigen::Matrix<double, 12, 12, Eigen::RowMajor>> svd_holder(
      mtm_eigen, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // LOG(INFO)<<m.transpose() * m;
  // LOG(INFO) << svd_holder.matrixU();

  // double mtm[12 * 12], d[12], ut[12 * 12];
  // CvMat MtM = cvMat(12, 12, CV_64F, mtm_eigen.data());
  // CvMat D = cvMat(12, 1, CV_64F, d);
  // CvMat Ut = cvMat(12, 12, CV_64F, ut);
  // cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A);
  // {
  //   std::stringstream info;

  //   for (int i = 0; i < 12; i++) {
  //     for (int j = 0; j < 12; j++) {
  //       info << ut[i * 12 + j] << " ";
  //     }
  //     info << std::endl;
  //   }
  //   info << std::endl;
  //   // LOG(INFO) << info.str();
  // }
  // for (int i = 0; i < 12; i++) {
  //   for (int j = 0; j < 4; j++) {
  //     m_svd_u_matrix_(i, j) =ut[i * 12 + (11-j)];
  //   }
  // }
  // LOG(INFO)<<svd_holder.matrixU();
  // m_svd_u_matrix_  = svd_holder.matrixV().rightCols(4);
  m_svd_u_matrix_ << svd_holder.matrixU().col(11), svd_holder.matrixU().col(10),
      svd_holder.matrixV().col(9), svd_holder.matrixV().col(8);
  l_6x10_ = betas_requires_process_->ComputeL6x10(m_svd_u_matrix_);
  // LOG(INFO)<<l_6x10_;
}
//

std::vector<Eigen::Vector3d> BetasProcess::CameraPoints() const {
  CHECK_EQ(key_points_.size(), handle_point_->Alphas().size());
  const Eigen::Matrix<double, 6, 1> &control_points_norm =
      handle_point_->ControlNormal();
  Eigen::Vector4d betas;

  if (approx_ == 0) {
    betas =
        betas_requires_process_->FindBetasApprox1(l_6x10_, control_points_norm);
  } else if (approx_ == 1) {
    betas =
        betas_requires_process_->FindBetasApprox2(l_6x10_, control_points_norm);
  } else if (approx_ == 2) {
    betas =
        betas_requires_process_->FindBetasApprox3(l_6x10_, control_points_norm);
  } else {
    LOG(FATAL) << " Apporx err.";
  }
  // LOG(INFO)<<betas.transpose();
  //
  BetasOptimization betas_optimization(BetasOptimizationOption{}, &betas,
                                       l_6x10_, control_points_norm);
  betas_optimization();
  // LOG(INFO)<<betas.transpose();
  //
  const Eigen::Matrix<double, 12, 1> c_control = m_svd_u_matrix_ * betas;
  //
  //
  const auto &alphas = handle_point_->Alphas();

  std::vector<Eigen::Vector3d> result(key_points_.size());
  int z = 1;
  for (size_t i = 0; i < key_points_.size(); i++) {
    Eigen::Vector3d point = Eigen::Vector3d::Zero();
    for (int j = 0; j < 4; j++) {
      point += alphas[i](j) * c_control.block(j * 3, 0, 3, 1);
    }
    result[i] = point;
    if (i == 0) {
      z = result[0].z();
    }
    if (z < 0.0) {
      result[i] = -result[i];
    }
    // LOG(INFO) << result[i].transpose();
  }
  return result;
}
//
HandlePoint::HandlePoint(const std::vector<Eigen::Vector3d> &points)
    : points_(points) {
  //
  CHECK_GE(points.size(), kMinPointsSize_);
  //
  const auto control_points = ComputeControlPoints();
  control_normal_ = ComputeControlNormal(control_points);
  alphas_ = ComputeBarycentricCoordinates(points_, control_points);
}
//
std::array<Eigen::Vector3d, 4> HandlePoint::ComputeControlPoints() {
  //
  const Eigen::Vector3d mean_points = MeanPoints(points_);
  const std::vector<Eigen::Vector3d> decentroid_points =
      MinusPoints(points_, mean_points);
  const Eigen::MatrixXd centroid_matrix = StdVectorToEigen(decentroid_points);
  //
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      centroid_matrix.transpose() * centroid_matrix,
      Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::Vector3d lamda = svd.singularValues() / points_.size();

  CHECK(svd.rank() == 3);
  //
  const int d = 4;
  std::array<Eigen::Vector3d, 4> cw;
  cw[0] = mean_points;
  for (int i = 1; i < d; i++) {
    cw[i] = cw[0] + sqrt(lamda[i - 1]) * svd.matrixU().col(i - 1);
  }
  return cw;
  //
}
//

std::vector<Eigen::Vector4d> HandlePoint::ComputeBarycentricCoordinates(
    const std::vector<Eigen::Vector3d> &points,
    const std::array<Eigen::Vector3d, 4> &cc) {
  //
  Eigen::MatrixXd cc_matrix = Eigen::Matrix3d::Zero();
  cc_matrix << cc[1], cc[2], cc[3];
  //
  const Eigen::MatrixXd mean_cc_matrix = cc_matrix.colwise() - cc[0];
  //
  const std::vector<Eigen::Vector3d> decentroid_points =
      MinusPoints(points, cc[0]);
  std::vector<Eigen::Vector4d> result;

  auto cc_matrix_inverse = PinvEigenBased(mean_cc_matrix);
  // auto cc_matrix_inverse =mean_cc_matrix.inverse();
  for (auto const &point : decentroid_points) {
    const Eigen::Vector3d a_p = cc_matrix_inverse * point;
    Eigen::Vector4d a;
    a << (1 - a_p.sum()), a_p;
    result.push_back(a);
  }

  return result;
}
//
Eigen::Matrix<double, 6, 1> HandlePoint::ComputeControlNormal(
    const std::array<Eigen::Vector3d, 4> &points) {
  Eigen::Matrix<double, 6, 1> result;
  result[0] = (points[0] - points[1]).squaredNorm();
  result[1] = (points[0] - points[2]).squaredNorm();
  result[2] = (points[0] - points[3]).squaredNorm();
  result[3] = (points[1] - points[2]).squaredNorm();
  result[4] = (points[1] - points[3]).squaredNorm();
  result[5] = (points[2] - points[3]).squaredNorm();
  return result;
}
//

PnpSolver::PnpSolver(const PnpSolverOption &option,
                     const std::vector<Eigen::Vector3d> &points,
                     const std::vector<Eigen::Vector2d> &key_points)
    : options_(option), points_(points), key_points_(key_points) {
  for (size_t i = 0; i < points_.size(); i++) {
    points_index_.push_back(i);
  }
  this->handle_points_ = nullptr;
  this->betas_process_ = nullptr;
  this->max_iterations = 0;
}
//
void PnpSolver::RestMinPoints(const std::vector<bool> &best_inliers) {
  //
  CHECK(!points_.empty());
  CHECK(!key_points_.empty());
  std::vector<Eigen::Vector3d> min_set_points;
  std::vector<Eigen::Vector2d> min_set_key_points;

  static std::mt19937 rng(42);
  if (best_inliers.empty()) {
    std::vector<int> points_index = points_index_;
    for (int i = 0; i < options_.min_set; i++) {
      std::uniform_int_distribution<int> ran(0, points_.size() - 1);
      const int randi = DUtils::Random::RandomInt(0, points_index.size() - 1);
      // LOG(INFO)<<points_index.size();
      // const int randi = ran(rng);
      min_set_points.push_back(points_[points_index[randi]]);
      min_set_key_points.push_back(key_points_[points_index[randi]]);
      points_index[randi] = points_index.back();
      points_index.pop_back();
    }
  } else {
    // LOG(INFO) << "best inliers "
    //           << std::count(best_inliers.begin(), best_inliers.end(), true);
    for (size_t i = 0; i < best_inliers.size(); i++) {
      if (best_inliers[i]) {
        min_set_points.push_back(points_[i]);
        min_set_key_points.push_back(key_points_[i]);
      }
    }
  }
  LOG(INFO) << "!";
  handle_points_ = std::make_unique<HandlePoint>(min_set_points);
  betas_process_ =
      std::make_unique<BetasProcess>(min_set_key_points, handle_points_.get());
}
transform::Rigid3d PnpSolver::IcpComputePose(
    const std::vector<Eigen::Vector3d> &p1,
    const std::vector<Eigen::Vector3d> &p2) {
  //
  CHECK_GE(p1.size(), 4);
  CHECK_EQ(p1.size(), p2.size());
  const std::vector<Eigen::Vector3d> &points = p1;  //
  const auto mean_points = MeanPoints(points);
  //
  const std::vector<Eigen::Vector3d> &camera_points = p2;  // =
  CHECK(!camera_points.empty());
  const auto mean_camera_points = MeanPoints(camera_points);

  CHECK(!points.empty());
  CHECK(!camera_points.empty());
  const std::vector<Eigen::Vector3d> decentroid_points =
      MinusPoints(points, mean_points);

  const std::vector<Eigen::Vector3d> decentroid_camera_points =
      MinusPoints(camera_points, mean_camera_points);
  //
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(
      StdVectorToEigen(decentroid_camera_points).transpose() *
          StdVectorToEigen(decentroid_points),
      Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::Matrix3d R = svd_holder.matrixU() * svd_holder.matrixV().transpose();
  if (R.determinant() < 0) {
    R(2, 0) = -R(2, 0);
    R(2, 1) = -R(2, 1);
    R(2, 2) = -R(2, 2);
  }
  Eigen::Vector3d t = mean_camera_points - R * mean_points;
  return transform::Rigid3d(t, Eigen::Quaterniond(R));
}
//

std::vector<Eigen::Vector3d> TrasformPoints(
    const transform::Rigid3d &pose,
    const std::vector<Eigen::Vector3d> &points) {
  std::vector<Eigen::Vector3d> result;
  for (const auto &point : points) {
    result.push_back(pose * point);
  }
  return result;
}

transform::Rigid3d PnpSolver::ComputePose() {
  //
  std::multimap<double, transform::Rigid3d> errs;
  for (int i = 0; i < 3; i++) {
    //
    auto key_points = betas_process_->SetApprox(i)->CameraPoints();
    auto pose = IcpComputePose(handle_points_->Points(), key_points);
    // LOG(INFO) << pose;
    errs.emplace(ReprojectionError(pose), pose);
  }
  return errs.begin()->second;
}
//
double PnpSolver::ReprojectionError(const transform::Rigid3d &pose) {
  //
  CHECK_NOTNULL(handle_points_);
  CHECK_NOTNULL(betas_process_);
  const auto &points = handle_points_->Points();
  const auto &key_points = betas_process_->KeyPoints();
  //
  CHECK(!points.empty());
  CHECK(!key_points.empty());
  CHECK_EQ(key_points.size(), points.size());
  //
  double err = 0.0;
  for (int i = 0; i < points.size(); i++) {
    const Eigen::Vector3d transform_point = pose * points[i];
    const Eigen::Vector3d norm_transform_point =
        transform_point / transform_point.z();
    err += (norm_transform_point.head<2>() - key_points[i]).norm();
  }
  return err / points.size();
}
//

//
std::vector<bool> PnpSolver::CheckInliers(const transform::Rigid3d &pose) {
  std::vector<bool> inliers(points_.size(), false);
  for (int i = 0; i < points_.size(); i++) {
    const Eigen::Vector3d transform_point = pose * points_[i];
    if (transform_point.z() < 0) continue;
    const Eigen::Vector3d norm_transform_point =
        transform_point / transform_point.z();
    double err =
        (norm_transform_point.head<2>() - key_points_[i]).squaredNorm();
    // LOG(INFO)<<err;
    if (err < options_.th2) {
      inliers[i] = true;
    }
  }
  return std::move(inliers);
}
//

std::pair<transform::Rigid3d, std::vector<bool>>
PnpSolver::CoarseComputePose() {
  RestMinPoints(std::vector<bool>{});

  auto pose = ComputePose();

  return {pose, CheckInliers(pose)};
}
//
//
//

std::unique_ptr<PnPsolverResult> PnpSolver::Refine(
    const std::vector<bool> &best_inliers) {
  RestMinPoints(best_inliers);
  auto pose = ComputePose();
  // LOG(INFO)<<"refine "<<pose;
  auto inliers = CheckInliers(pose);
  //
  // LOG(INFO) << std::count(inliers.begin(), inliers.end(), true);
  //
  if (std::count(inliers.begin(), inliers.end(), true) > options_.min_inliers) {
    //
    //
    return std::make_unique<PnPsolverResult>(
        PnPsolverResult{pose, std::move(inliers)});
  }
  return nullptr;
}

//
std::unique_ptr<PnPsolverResult> PnpSolver::operator()(
    const transform::Rigid3d &init_pose) {
  //
  if (key_points_.size() < options_.min_inliers) return nullptr;
  //
  int best_inliers_size = 0;
  transform::Rigid3d best_pose;
  std::vector<bool> best_inliers;
  int iterations = 0;
  while (max_iterations++ < options_.max_iterations ||
         iterations++ < options_.iterations) {
    //

    auto [pose, inliers] = CoarseComputePose();
    // LOG(INFO)<<pose;
    const int inliers_size = std::count(inliers.begin(), inliers.end(), true);
    // LOG(INFO)<<inliers_size ;

    if (inliers_size > options_.min_inliers) {
      if (inliers_size > best_inliers_size) {
        best_inliers_size = inliers_size;
        best_pose = pose;
        best_inliers = inliers;
      }

      auto refine_result = Refine(best_inliers);
      if (refine_result) {
        return refine_result;
      }
    }
  }
  //
  if (max_iterations > options_.max_iterations) {
    if (std::count(best_inliers.begin(), best_inliers.end(), true) >
        options_.min_inliers) {
      return std::make_unique<PnPsolverResult>(
          PnPsolverResult{best_pose, std::move(best_inliers)});
    }
  }
  return nullptr;
}

std::unique_ptr<PnPsolverResult> PnpSolver::Solve(
    const std::vector<Eigen::Vector3d> &points,
    const std::vector<Eigen::Vector2d> &key_points,
    const transform::Rigid3d &init_pose) {
  //
  CHECK(!points.empty());
  CHECK(!key_points.empty());
  if (points.size() < options_.min_set) return nullptr;
  const float n = points.size();
  int min_inliers = options_.epsilon * key_points.size();
  //
  if (min_inliers < options_.min_inliers) min_inliers = options_.min_inliers;
  if (min_inliers < options_.min_set) min_inliers = options_.min_set;
  PnpSolverOption option = options_;
  option.min_inliers = min_inliers;
  //
  if (options_.epsilon < min_inliers / n) {
    options_.epsilon = min_inliers / n;
  }
  //
  int iterations = 0;
  if (option.min_inliers == n) {
    iterations = 1;
  } else {
    iterations =
        ceil(log(1 - option.probability) / log(1 - pow(option.epsilon, 3)));
  }
  option.max_iterations =
      std::max(1, std::min(iterations, option.max_iterations));
  //

  auto pnp_ptr = std::make_unique<PnpSolver>(option, points, key_points);
  auto result = pnp_ptr->operator()(init_pose);

  if (!result) {
    result = std::make_unique<PnPsolverResult>();
    result->extend = std::move(pnp_ptr);
  }

  return std::move(result);
}

std::unique_ptr<PnPsolverResult> PnpSolver::Find() {
  options_.iterations = options_.max_iterations;
  return this->operator()(transform::Rigid3d::Identity());
}
}  // namespace alg
}  // namespace jarvis