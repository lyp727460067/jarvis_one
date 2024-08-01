#include "eigen_quaterniond_from_two_vectors.h"

Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b) {
  return Eigen::Quaterniond::FromTwoVectors(a, b);
}
