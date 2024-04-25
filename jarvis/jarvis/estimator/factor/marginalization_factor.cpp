/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science
 *and Technology
 *
 * This file is part of VINS.
 *
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "marginalization_factor.h"

#include <fstream>
namespace jarvis {
namespace estimator {
void ResidualBlockInfo::Evaluate() {
  residuals.resize(cost_function->num_residuals());

  std::vector<int> block_sizes = cost_function->parameter_block_sizes();
  raw_jacobians = new double *[block_sizes.size()];
  jacobians.resize(block_sizes.size());

  for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
    jacobians[i].resize(cost_function->num_residuals(), block_sizes[i]);
    raw_jacobians[i] = jacobians[i].data();
    // dim += block_sizes[i] == 7 ? 6 : block_sizes[i];
  }
  cost_function->Evaluate(parameter_blocks.data(), residuals.data(),
                          raw_jacobians);

  // std::vector<int> tmp_idx(block_sizes.size());
  // Eigen::MatrixXd tmp(dim, dim);
  // for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++)
  //{
  //     int size_i = localSize(block_sizes[i]);
  //     Eigen::MatrixXd jacobian_i = jacobians[i].leftCols(size_i);
  //     for (int j = 0, sub_idx = 0; j <
  //     static_cast<int>(parameter_blocks.size()); sub_idx += block_sizes[j] ==
  //     7 ? 6 : block_sizes[j], j++)
  //     {
  //         int size_j = localSize(block_sizes[j]);
  //         Eigen::MatrixXd jacobian_j = jacobians[j].leftCols(size_j);
  //         tmp_idx[j] = sub_idx;
  //         tmp.block(tmp_idx[i], tmp_idx[j], size_i, size_j) =
  //         jacobian_i.transpose() * jacobian_j;
  //     }
  // }
  // Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(tmp);
  // std::cout << saes.eigenvalues() << std::endl;
  // ROS_ASSERT(saes.eigenvalues().minCoeff() >= -1e-6);

  if (loss_function) {
    double residual_scaling_, alpha_sq_norm_;

    double sq_norm, rho[3];

    sq_norm = residuals.squaredNorm();
    loss_function->Evaluate(sq_norm, rho);
    // printf("sq_norm: %f, rho[0]: %f, rho[1]: %f, rho[2]: %f\n", sq_norm,
    // rho[0], rho[1], rho[2]);

    double sqrt_rho1_ = sqrt(rho[1]);

    if ((sq_norm == 0.0) || (rho[2] <= 0.0)) {
      residual_scaling_ = sqrt_rho1_;
      alpha_sq_norm_ = 0.0;
    } else {
      const double D = 1.0 + 2.0 * sq_norm * rho[2] / rho[1];
      const double alpha = 1.0 - sqrt(D);
      residual_scaling_ = sqrt_rho1_ / (1 - alpha);
      alpha_sq_norm_ = alpha / sq_norm;
    }

    for (int i = 0; i < static_cast<int>(parameter_blocks.size()); i++) {
      jacobians[i] = sqrt_rho1_ * (jacobians[i] -
                                   alpha_sq_norm_ * residuals *
                                       (residuals.transpose() * jacobians[i]));
    }

    residuals *= residual_scaling_;
  }
}

MarginalizationInfo::~MarginalizationInfo() {
  // ROS_WARN("release marginlizationinfo");

  for (auto it = parameter_block_data.begin(); it != parameter_block_data.end();
       ++it)
    delete it->second;

  for (int i = 0; i < (int)factors.size(); i++) {
    delete[] factors[i]->raw_jacobians;

    delete factors[i]->cost_function;

    delete factors[i];
  }
}

void MarginalizationInfo::addResidualBlockInfo(
    ResidualBlockInfo *residual_block_info) {
  factors.emplace_back(residual_block_info);

  std::vector<double *> &parameter_blocks =
      residual_block_info->parameter_blocks;
  std::vector<int> parameter_block_sizes =
      residual_block_info->cost_function->parameter_block_sizes();

  for (int i = 0;
       i < static_cast<int>(residual_block_info->parameter_blocks.size());
       i++) {
    double *addr = parameter_blocks[i];
    int size = parameter_block_sizes[i];
    parameter_block_size[reinterpret_cast<long>(addr)] = size;
  }

  for (int i = 0; i < static_cast<int>(residual_block_info->drop_set.size());
       i++) {
    double *addr = parameter_blocks[residual_block_info->drop_set[i]];
    parameter_block_idx[reinterpret_cast<long>(addr)] = 0;
  }
}

void MarginalizationInfo::preMarginalize() {
  for (auto it : factors) {
    it->Evaluate();

    std::vector<int> block_sizes = it->cost_function->parameter_block_sizes();
    for (int i = 0; i < static_cast<int>(block_sizes.size()); i++) {
      long addr = reinterpret_cast<long>(it->parameter_blocks[i]);
      int size = block_sizes[i];
      if (parameter_block_data.find(addr) == parameter_block_data.end()) {
        double *data = new double[size];
        memcpy(data, it->parameter_blocks[i], sizeof(double) * size);
        parameter_block_data[addr] = data;
      }
    }
  }
}

int MarginalizationInfo::localSize(int size) const {
  return size == 7 ? 6 : size;
}

int MarginalizationInfo::globalSize(int size) const {
  return size == 6 ? 7 : size;
}

void *ThreadsConstructA(void *threadsstruct) {
  ThreadsStruct *p = ((ThreadsStruct *)threadsstruct);
  for (auto it : p->sub_factors) {
    for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
      int idx_i = p->parameter_block_idx[reinterpret_cast<long>(
          it->parameter_blocks[i])];
      int size_i = p->parameter_block_size[reinterpret_cast<long>(
          it->parameter_blocks[i])];
      if (size_i == 7) size_i = 6;
      Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
      for (int j = i; j < static_cast<int>(it->parameter_blocks.size()); j++) {
        int idx_j = p->parameter_block_idx[reinterpret_cast<long>(
            it->parameter_blocks[j])];
        int size_j = p->parameter_block_size[reinterpret_cast<long>(
            it->parameter_blocks[j])];
        if (size_j == 7) size_j = 6;
        Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
        if (i == j)
          p->A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
        else {
          p->A.block(idx_i, idx_j, size_i, size_j) +=
              jacobian_i.transpose() * jacobian_j;
          p->A.block(idx_j, idx_i, size_j, size_i) =
              p->A.block(idx_i, idx_j, size_i, size_j).transpose();
        }
      }
      p->b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
    }
  }
  return threadsstruct;
}

#if 1
void MarginalizationInfo::marginalize() {
  int pos = 0;
  for (auto &it : parameter_block_idx) {
    it.second = pos;
    pos += localSize(parameter_block_size[it.first]);
  }

  m = pos;

  for (const auto &it : parameter_block_size) {
    if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
      parameter_block_idx[it.first] = pos;
      pos += localSize(it.second);
    }
  }

  n = pos - m;
  // ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n,
  // (int)parameter_block_idx.size());
  if (m == 0) {
    valid = false;
    printf("unstable tracking...\n");
    return;
  }

  TicToc t_summing;
  Eigen::MatrixXd A(pos, pos);
  Eigen::VectorXd b(pos);
  A.setZero();
  b.setZero();
  /*
  for (auto it : factors)
  {
      for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++)
      {
          int idx_i =
  parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])]; int
  size_i =
  localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[i])]);
          Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);
          for (int j = i; j < static_cast<int>(it->parameter_blocks.size());
  j++)
          {
              int idx_j =
  parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[j])]; int
  size_j =
  localSize(parameter_block_size[reinterpret_cast<long>(it->parameter_blocks[j])]);
              Eigen::MatrixXd jacobian_j = it->jacobians[j].leftCols(size_j);
              if (i == j)
                  A.block(idx_i, idx_j, size_i, size_j) +=
  jacobian_i.transpose() * jacobian_j; else
              {
                  A.block(idx_i, idx_j, size_i, size_j) +=
  jacobian_i.transpose() * jacobian_j; A.block(idx_j, idx_i, size_j, size_i) =
  A.block(idx_i, idx_j, size_i, size_j).transpose();
              }
          }
          b.segment(idx_i, size_i) += jacobian_i.transpose() * it->residuals;
      }
  }
  ROS_INFO("summing up costs %f ms", t_summing.toc());
  */
  // multi thread

  TicToc t_thread_summing;
  pthread_t tids[NUM_THREADS];
  ThreadsStruct threadsstruct[NUM_THREADS];
  int i = 0;
  for (auto it : factors) {
    threadsstruct[i].sub_factors.push_back(it);
    i++;
    i = i % NUM_THREADS;
  }
  for (int i = 0; i < NUM_THREADS; i++) {
    TicToc zero_matrix;
    threadsstruct[i].A = Eigen::MatrixXd::Zero(pos, pos);
    threadsstruct[i].b = Eigen::VectorXd::Zero(pos);
    threadsstruct[i].parameter_block_size = parameter_block_size;
    threadsstruct[i].parameter_block_idx = parameter_block_idx;
    int ret = pthread_create(&tids[i], NULL, ThreadsConstructA,
                             (void *)&(threadsstruct[i]));
    CHECK(ret == 0) << "pthread_create error";
  }
  for (int i = NUM_THREADS - 1; i >= 0; i--) {
    pthread_join(tids[i], NULL);
    A += threadsstruct[i].A;
    b += threadsstruct[i].b;
  }
  // ROS_DEBUG("thread summing up costs %f ms", t_thread_summing.toc());
  // ROS_INFO("A diff %f , b diff %f ", (A - tmp_A).sum(), (b - tmp_b).sum());

  // TODO
  Eigen::MatrixXd Amm =
      0.5 * (A.block(0, 0, m, m) + A.block(0, 0, m, m).transpose());
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(Amm);

  // ROS_ASSERT_MSG(saes.eigenvalues().minCoeff() >= -1e-4, "min eigenvalue %f",
  // saes.eigenvalues().minCoeff());

  Eigen::MatrixXd Amm_inv =
      saes.eigenvectors() *
      Eigen::VectorXd((saes.eigenvalues().array() > eps)
                          .select(saes.eigenvalues().array().inverse(), 0))
          .asDiagonal() *
      saes.eigenvectors().transpose();
  // printf("error1: %f\n", (Amm * Amm_inv - Eigen::MatrixXd::Identity(m,
  // m)).sum());

  Eigen::VectorXd bmm = b.segment(0, m);
  Eigen::MatrixXd Amr = A.block(0, m, m, n);
  Eigen::MatrixXd Arm = A.block(m, 0, n, m);
  Eigen::MatrixXd Arr = A.block(m, m, n, n);
  Eigen::VectorXd brr = b.segment(m, n);
  A = Arr - Arm * Amm_inv * Amr;
  b = brr - Arm * Amm_inv * bmm;

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(A);
  Eigen::VectorXd S =
      Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array(), 0));
  Eigen::VectorXd S_inv =
      Eigen::VectorXd((saes2.eigenvalues().array() > eps)
                          .select(saes2.eigenvalues().array().inverse(), 0));

  Eigen::VectorXd S_sqrt = S.cwiseSqrt();
  Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

  linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();
  linearized_residuals =
      S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * b;
  // std::cout << A << std::endl
  //           << std::endl;
  // std::cout << linearized_jacobians << std::endl;
  // printf("error2: %f %f\n", (linearized_jacobians.transpose() *
  // linearized_jacobians - A).sum(),
  //       (linearized_jacobians.transpose() * linearized_residuals - b).sum());
  //  ROS_INFO("hessian summing up costs %f ms", t_summing.toc());
}
#else
void ReadData(std::istream &fin, Eigen::MatrixXd &m_matrix) {
  int numRow = m_matrix.rows();
  int numCol = m_matrix.cols();

  Eigen::VectorXd vecPerRow(numRow);
  for (int j = 0; j < numRow; j++)  // 共numRow行
  {
    for (int i = 0; i < numCol; i++)  // 共numCol列组成一行
    {
      fin >> m_matrix(j, i);
    }
  }
}
void WriteMatrix(Eigen::MatrixXd &m_matrix) {
  //   static bool is_head_write= false;
  std::ofstream fout("/tmp/matrixTest.bin", std::ios::app);
  fout << "header " << m_matrix.rows() << " " << m_matrix.cols() << std::endl;
  fout << m_matrix << std::endl;
}

void MarginalizationInfo::marginalize() {
  int pos = 0;
  for (auto &it : parameter_block_idx) {
    it.second = pos;
    pos += localSize(parameter_block_size[it.first]);
  }

  m = pos;

  for (const auto &it : parameter_block_size) {
    if (parameter_block_idx.find(it.first) == parameter_block_idx.end()) {
      parameter_block_idx[it.first] = pos;
      pos += localSize(it.second);
    }
  }

  n = pos - m;
  // ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n,
  // (int)parameter_block_idx.size());
  if (m == 0) {
    valid = false;
    printf("unstable tracking...\n");
    return;
  }

  int szf = 0;
  for (auto it : factors) {
    szf += it->residuals.rows();
  }

  // ROS_INFO("marginalization, pos: %d, m: %d, n: %d, size: %d", pos, m, n,
  // (int)parameter_block_idx.size());

  Eigen::MatrixXd jac_marg(szf, m), jac_rem(szf, n);
  Eigen::VectorXd res(szf);
  jac_marg.setZero();
  jac_rem.setZero();
  res.setZero();

  //   ROS_INFO("marginalization, szf: %d, m: %d, n: %d", szf, m, n);

  szf = 0;
  for (auto it : factors) {
    for (int i = 0; i < static_cast<int>(it->parameter_blocks.size()); i++) {
      int idx_i =
          parameter_block_idx[reinterpret_cast<long>(it->parameter_blocks[i])];
      int size_i = localSize(parameter_block_size[reinterpret_cast<long>(
          it->parameter_blocks[i])]);
      Eigen::MatrixXd jacobian_i = it->jacobians[i].leftCols(size_i);

      if (idx_i < m) {
        jac_marg.block(szf, idx_i, jacobian_i.rows(), jacobian_i.cols()) =
            jacobian_i;
      } else {
        // if(idx_i - m >= n)
        // ROS_INFO("marginalization, idx_i - m: %d, n: %d", idx_i - m, n);
        jac_rem.block(szf, idx_i - m, jacobian_i.rows(), jacobian_i.cols()) =
            jacobian_i;
      }
    }
    res.segment(szf, it->residuals.rows()) = it->residuals;
    szf += it->residuals.rows();
  }
  TicToc t_summing;
  // conduct givens rotation first time
  Eigen::JacobiRotation<double> temp_gr;
  for (int i = 0; i < jac_marg.cols(); ++i) {
    for (int j = (int)jac_marg.rows() - 1; j > i; j--) {
      temp_gr.makeGivens(jac_marg(j - 1, i), jac_marg(j, i));
      // jac_marg
      (jac_marg.block(j - 1, i, 2, jac_marg.cols() - i))
          .applyOnTheLeft(0, 1, temp_gr.adjoint());
      (jac_rem.block(j - 1, 0, 2, jac_rem.cols()))
          .applyOnTheLeft(0, 1, temp_gr.adjoint());
      (res.block(j - 1, 0, 2, 1)).applyOnTheLeft(0, 1, temp_gr.adjoint());
    }
  }
  //   std::cout<<jac_marg<<std::endl;
  LOG(INFO) << jac_marg.rows() << " " << jac_marg.cols();
  ROS_DEBUG("jac_marg time %f", t_summing.toc());
  //   WriteMatrix(jac_marg);
  jac_rem = jac_rem
                .block(jac_marg.cols(), 0, jac_rem.rows() - jac_marg.cols(),
                       jac_rem.cols())
                .eval();
  res = res.block(jac_marg.cols(), 0, res.rows() - jac_marg.cols(), res.cols())
            .eval();

  TicToc t_rotaton_sencod;
  // conduct givens rotation second time
  for (int i = 0; i < jac_rem.cols(); ++i) {
    for (int j = (int)jac_rem.rows() - 1; j > i; j--) {
      temp_gr.makeGivens(jac_rem(j - 1, i), jac_rem(j, i));

      (jac_rem.block(j - 1, 0, 2, jac_rem.cols()))
          .applyOnTheLeft(0, 1, temp_gr.adjoint());
      (res.block(j - 1, 0, 2, 1)).applyOnTheLeft(0, 1, temp_gr.adjoint());
    }
  }

  ROS_DEBUG(" rotaion sencod %f", t_rotaton_sencod.toc());
  linearized_jacobians = jac_rem.block(0, 0, jac_rem.cols(), jac_rem.cols());
  linearized_residuals = res.block(0, 0, jac_rem.cols(), res.cols());

  //   ROS_INFO("linearized_jacobians size:(%d, %d)",
  //   linearized_jacobians.rows(), linearized_jacobians.cols());
  //   ROS_INFO("linearized_residuals size:(%d, %d)",
  //   linearized_residuals.rows(), linearized_residuals.cols());
  //   ROS_INFO("givens summing up costs %f ms", t_summing.toc());
}
#endif

std::vector<double *> MarginalizationInfo::getParameterBlocks(
    std::unordered_map<long, double *> &addr_shift) {
  std::vector<double *> keep_block_addr;
  keep_block_size.clear();
  keep_block_idx.clear();
  keep_block_data.clear();
  std::ostringstream pra_block_data_str;
  std::ostringstream addr_shift_data_str;
  for (const auto &it : parameter_block_idx) {
    if (it.second >= m) {
      // std::cout<<it.first<<std::endl;
      keep_block_size.push_back(parameter_block_size[it.first]);
      keep_block_idx.push_back(parameter_block_idx[it.first]);
      keep_block_data.push_back(parameter_block_data[it.first]);
      // pra_block_data_str
      //     << "parameter_block_data[it.first]"
      //     << reinterpret_cast<long>(parameter_block_data[it.first])
      //     << std::endl;
      // addr_shift_data_str << "addr_shirf[it.first]"
      //                     << reinterpret_cast<long>(addr_shift[it.first])
      //                     << std::endl;

      keep_block_addr.push_back(addr_shift[it.first]);
    }
  }
  // std::cout<<pra_block_data_str.str();
  // std::cout<<addr_shift_data_str.str();
  sum_block_size = std::accumulate(std::begin(keep_block_size),
                                   std::end(keep_block_size), 0);

  return keep_block_addr;
}

MarginalizationFactor::MarginalizationFactor(
    MarginalizationInfo *_marginalization_info)
    : marginalization_info(_marginalization_info) {
  int cnt = 0;
  for (auto it : marginalization_info->keep_block_size) {
    mutable_parameter_block_sizes()->push_back(it);
    cnt += it;
  }
  // printf("residual size: %d, %d\n", cnt, n);
  set_num_residuals(marginalization_info->n);
};

bool MarginalizationFactor::Evaluate(double const *const *parameters,
                                     double *residuals,
                                     double **jacobians) const {
  // printf("internal addr,%d, %d\n", (int)parameter_block_sizes().size(),
  // num_residuals()); for (int i = 0; i <
  // static_cast<int>(keep_block_size.size()); i++)
  //{
  //     //printf("unsigned %x\n", reinterpret_cast<unsigned
  //     long>(parameters[i]));
  //     //printf("signed %x\n", reinterpret_cast<long>(parameters[i]));
  // printf("jacobian %x\n", reinterpret_cast<long>(jacobians));
  // printf("residual %x\n", reinterpret_cast<long>(residuals));
  // }
  int n = marginalization_info->n;
  int m = marginalization_info->m;
  Eigen::VectorXd dx(n);
  for (int i = 0;
       i < static_cast<int>(marginalization_info->keep_block_size.size());
       i++) {
    int size = marginalization_info->keep_block_size[i];
    int idx = marginalization_info->keep_block_idx[i] - m;
    Eigen::VectorXd x = Eigen::Map<const Eigen::VectorXd>(parameters[i], size);
    Eigen::VectorXd x0 = Eigen::Map<const Eigen::VectorXd>(
        marginalization_info->keep_block_data[i], size);
    if (size != 7)
      dx.segment(idx, size) = x - x0;
    else {
      dx.segment<3>(idx + 0) = x.head<3>() - x0.head<3>();
      dx.segment<3>(idx + 3) =
          2.0 * Utility::positify(
                    Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                    Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                    .vec();
      if (!((Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
             Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                .w() >= 0)) {
        dx.segment<3>(idx + 3) =
            2.0 *
            -Utility::positify(
                 Eigen::Quaterniond(x0(6), x0(3), x0(4), x0(5)).inverse() *
                 Eigen::Quaterniond(x(6), x(3), x(4), x(5)))
                 .vec();
      }
    }
  }
  Eigen::Map<Eigen::VectorXd>(residuals, n) =
      marginalization_info->linearized_residuals +
      marginalization_info->linearized_jacobians * dx;
  if (jacobians) {
    std::ostringstream print_str;
    for (int i = 0;
         i < static_cast<int>(marginalization_info->keep_block_size.size());
         i++) {
      // print_str<<"i =" <<i<<" ";
      if (jacobians[i]) {
        int size = marginalization_info->keep_block_size[i],
            local_size = marginalization_info->localSize(size);
        int idx = marginalization_info->keep_block_idx[i] - m;
        Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                                 Eigen::RowMajor>>
            jacobian(jacobians[i], n, size);
        jacobian.setZero();
        jacobian.leftCols(local_size) =
            marginalization_info->linearized_jacobians.middleCols(idx,
                                                                  local_size);
        // print_str<<"idx  = "<<idx;
        // print_str<<"size  = "<<size;
        // print_str<<"jacobian"<<jacobian.rows()<<"
        // "<<jacobian.cols()<<std::endl;
      }
    }
    // std::cout<<print_str.str();
  }
  return true;
}
}  // namespace estimator
}  // namespace jarvis
