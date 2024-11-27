// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// This file is subject to the terms and conditions defined in the file
// 'LICENSE', which is part of this source code package.

#ifndef JARVIS_MAPPING_SVO_PATH_WARP_H
#define JARVIS_MAPPING_SVO_PATH_WARP_H
#include "Eigen/Eigen"
#include "jarvis/common/id.h"
#include "jarvis/mapping/match/data_type.h"
namespace jarvis {
namespace mapping {
namespace match {
using namespace camera_models;
namespace svo {
// Forward declarations.
/// Warp a patch from the reference view to the current view.
namespace warp {

void getWarpMatrixAffine(const CameraPtr& cam_ref, const CameraPtr& cam_cur,
                         const Eigen::Vector2d& px_ref,
                         const Eigen::Vector3d& f_ref, const double depth_ref,
                         const transform::Rigid3d& T_cur_ref,
                         const int level_ref, AffineTransformation2* A_cur_ref);

void getWarpMatrixAffineHomography(
    const CameraPtr& cam_ref, const CameraPtr& cam_cur,
    const Eigen::Vector2d& px_ref, const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& normal_ref, const double depth_ref,
    const transform::Rigid3d& T_cur_ref, const int level_ref,
    AffineTransformation2& A_cur_ref);

int getBestSearchLevel(const AffineTransformation2& A_cur_ref,
                       const int max_level);

bool warpAffine(const AffineTransformation2& A_cur_ref, const cv::Mat& img_ref,
                const Eigen::Vector2d& px_ref, const int level_ref,
                const int level_cur, const int halfpatch_size, uint8_t* patch);

bool warpPixelwise(const Frame& cur_frame, const Frame& ref_frame,
                   const FeatureWrapper& ref_ftr, const int level_ref,
                   const int level_cur, const int halfpatch_size,
                   uint8_t* patch);

void createPatchNoWarp(const cv::Mat& img, const Eigen::Vector2i& px,
                       const int halfpatch_size, uint8_t* patch);

void createPatchNoWarpInterpolated(const cv::Mat& img,
                                   const Eigen::Vector2d& px,
                                   const int halfpatch_size, uint8_t* patch);
}  // namespace warp
}  // namespace svo
}  // namespace match
}  // namespace mapping
}  // namespace jarvis

#endif