#ifndef _JARVIS_VIO__EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_
#define _JARVIS_VIO__EIGEN_QUATERNIOND_FROM_TWO_VECTORS_H_

#include "Eigen/Geometry"

// Calls Eigen::Quaterniond::FromTwoVectors(). This is in its own compilation
// unit since it can take more than 10 s to build while using more than 1 GB of
// memory causing slow build times and high peak memory usage.
Eigen::Quaterniond FromTwoVectors(const Eigen::Vector3d& a,
                                  const Eigen::Vector3d& b);

#endif
