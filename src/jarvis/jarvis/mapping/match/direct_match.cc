#include "jarvis/mapping/match/direct_match.h"

#include "jarvis/mapping/match/feature_alignment.h"
#include "jarvis/mapping/match/patch_utils.h"
namespace jarvis {
namespace mapping {
namespace match {
//
using namespace svo;
using BearingVector = Eigen::Vector3d;
bool Frame::IsVisible(const Eigen::Vector3d& xyz_w, Eigen::Vector2d* pt) {
  Eigen::Vector3d xyz_f = pose.inverse() * xyz_w;
  //
  if(xyz_f.z()<0)return false;
  // Eigen::Vector2d px_top_left(0.01, 0.01);
  // Eigen::Vector3d f_top_left;
  // cam->liftProjective(px_top_left, f_top_left);  // 注意这里找对应的相机
  const Eigen::Vector3d z(0.0, 0.0, 1.0);
  const double min_cos = f_top_left->dot(z);
  const double cur_cos = xyz_f.normalized().dot(z);
  if (cur_cos < min_cos) {
    return false;
  }
  if (pt) {
    cam->spaceToPlane(xyz_f, *pt);
  }
  return true;
}

bool Frame::IsKeypointVisibleWithMargin(const Eigen::Vector2d& keypoint,
                                        int margin) {

  int image_with = image_size.x();
  int image_height = image_size.y();
  return keypoint[0] >= margin && keypoint[1] >= margin &&
         keypoint[0] < (image_with - margin) &&
         keypoint[1] < (image_height - margin);
}

//
//
//
MatchResult DirectMatch::FindMatch(const Frame& ref_frame,
                                   const Frame& cur_frame,
                                   const FeatureWrapper& ref_ftr,
                                   const double& ref_depth,
                                   const Keypoint& pr) {
  Eigen::Vector2i pxi = ref_ftr.px.cast<int>() / (1 << ref_ftr.level);
  int boundary = kHalfPatchSize + 2;
  //
  if (pxi[0] < boundary || pxi[1] < boundary ||
      pxi[0] >=
          static_cast<int>(ref_frame.image_size.x() / (1 << ref_ftr.level)) -
              boundary ||
      pxi[1] >=
          static_cast<int>(ref_frame.image_size.y() / (1 << ref_ftr.level)) -
              boundary) {
    return {MatchResultState::kFailVisibility};
  }
  // warp affine
  //

  AffineTransformation2 A_cur_ref;
  warp::getWarpMatrixAffine(
      ref_frame.cam, cur_frame.cam, ref_ftr.px, ref_ftr.f, ref_depth,
      cur_frame.pose.inverse() * ref_frame.pose, ref_ftr.level, &A_cur_ref);
  //
  //
  // LOG(INFO)<<ref_frame.img_pyr.size();
  int search_level =
      warp::getBestSearchLevel(A_cur_ref, ref_frame.img_pyr.size() - 1);
  // LOG(INFO)<<search_level ;
  //
  //

  if (options_.use_affine_warp) {
    if (!warp::warpAffine(A_cur_ref, ref_frame.img_pyr[ref_ftr.level],
                          ref_ftr.px, ref_ftr.level, search_level,
                          kHalfPatchSize + 1, patch_with_border_)) {
      return {MatchResultState::kFailWarp};
    }

  } else {
    // pixelwise warp:
    // TODO(zzc): currently using the search level from affine, good enough?
    if (!warp::warpPixelwise(cur_frame, ref_frame, ref_ftr, ref_ftr.level,
                             search_level, kHalfPatchSize + 1,
                             patch_with_border_)) {
      return {MatchResultState::kFailWarp};
    }
  }

  patch_utils::createPatchFromPatchWithBorder(patch_with_border_, kPatchSize,
                                              patch_);
   const Keypoint& px_cur = pr;
  // px_cur should be set
  Keypoint px_scaled(px_cur / (1 << search_level));
  Keypoint px_scaled_start(px_scaled);
  // cv::imshow("cur_frame.img_pyr",cur_frame.img_pyr[search_level]);
  // cv::Mat patch_image(kPatchSize, kPatchSize, CV_8UC1, patch_);
  // cv::imshow("pach_image", patch_image);
  // cv::waitKey(0);
  std::vector<Eigen::Vector2f>* last_fail_steps = nullptr;
  bool res = feature_alignment::align2D(
      cur_frame.img_pyr[search_level], patch_with_border_, patch_,
      options_.align_max_iter, options_.affine_est_offset,
      options_.affine_est_gain, px_scaled, options_.min_update_squared, false,
      last_fail_steps);

  if (res) {
    if ((px_scaled - px_scaled_start).norm() >
        options_.max_patch_diff_ratio * kPatchSize) {
      VLOG(2) << "Proejct -esitimator distance  "
                << (px_scaled - px_scaled_start).norm() << " > "
                << options_.max_patch_diff_ratio * kPatchSize;
      // return {MatchResultState::kFailTooFar};
    }
    // LOG(INFO)<<(px_scaled - px_scaled_start).norm();
     const Keypoint px_cur = px_scaled * (1 << search_level);
    // set member variables with results (used in reprojector)
    Eigen::Vector3d f_cur;
    cur_frame.cam->backProject3(px_cur, &f_cur);
    // f_cur.normalize();
    return {MatchResultState::kSuccess, px_cur, f_cur, search_level};
  } else {
    // LOG(WARNING) << "NOT CONVERGED: search level " << search_level;
  }
  return {MatchResultState::kFailAlignment};
}

//
MatchResult DirectMatch::FindEpipolarMatchDirect(
    const Frame& ref_frame, const Frame& cur_frame,
    const FeatureWrapper& ref_ftr, const double d_estimate_inv,
    const double d_min_inv, const double d_max_inv, double& depth) {
  transform::Rigid3d T_cur_ref = cur_frame.f_pose.inverse() * ref_frame.f_pose;
  return FindEpipolarMatchDirect(ref_frame, cur_frame, T_cur_ref, ref_ftr,
                                 d_estimate_inv, d_min_inv, d_max_inv, depth);
}

MatchResult DirectMatch::FindEpipolarMatchDirect(
    const Frame& ref_frame, const Frame& cur_frame,
    const transform::Rigid3d& T_cur_ref, const FeatureWrapper& ref_ftr,
    const double d_estimate_inv, const double d_min_inv, const double d_max_inv,
    double& depth) {
  int zmssd_best = PatchScore::threshold();

  // Compute start and end of epipolar line in old_kf for match search, on image
  // plane
  const BearingVector A =
      T_cur_ref.rotation() * ref_ftr.f + T_cur_ref.translation() * d_min_inv;
  const BearingVector B =
      T_cur_ref.rotation() * ref_ftr.f + T_cur_ref.translation() * d_max_inv;
  Eigen::Vector2d px_A, px_B;
  cur_frame.cam->project3(A, &px_A);
  cur_frame.cam->project3(B, &px_B);
  Eigen::Vector2d epi_image = px_A - px_B;
  AffineTransformation2 A_cur_ref;
  // Compute affine warp matrix
  warp::getWarpMatrixAffine(ref_frame.cam, cur_frame.cam, ref_ftr.px, ref_ftr.f,
                            1.0 / std::max(0.000001, d_estimate_inv), T_cur_ref,
                            ref_ftr.level, &A_cur_ref);

  // feature pre-selection
  bool reject = false;

  // prepare for match
  //    - find best search level
  //    - warp the reference patch
  int search_level =
      warp::getBestSearchLevel(A_cur_ref, ref_frame.img_pyr.size() - 1);
  // length and direction on SEARCH LEVEL
  double epi_length_pyramid = epi_image.norm() / (1 << search_level);
  //
  GradientVector epi_dir_image = epi_image.normalized();
  if (!warp::warpAffine(A_cur_ref, ref_frame.img_pyr[ref_ftr.level], ref_ftr.px,
                        ref_ftr.level, search_level, kHalfPatchSize + 1,
                        patch_with_border_))
    return {MatchResultState::kFailWarp};
  //
  patch_utils::createPatchFromPatchWithBorder(patch_with_border_, kPatchSize,
                                              patch_);
  // Case 1: direct search locally if the epipolar line is too short
  Eigen::Vector2d px_cur;
  if (epi_length_pyramid < 2.0) {
    px_cur = (px_A + px_B) / 2.0;
    MatchResultState res =
        FindLocalMatch(cur_frame, epi_dir_image, search_level, px_cur);
    if (res != MatchResultState::kSuccess) return {res};
    Eigen::Vector3d f_cur;
    cur_frame.cam->backProject3(px_cur, &f_cur);
    f_cur.normalize();
    return {DepthFromTriangulation(T_cur_ref, ref_ftr.f, f_cur, &depth), px_cur,
            f_cur, search_level};
  }

  // Case 2: search along the epipolar line for the best match
  PatchScore patch_score(patch_);  // precompute for reference patch
  BearingVector C = T_cur_ref.rotation() * ref_ftr.f +
                    T_cur_ref.translation() * d_estimate_inv;
  ScanEpipolarLine(cur_frame, A, B, C, patch_score, search_level, &px_cur,
                   &zmssd_best);

  // check if the best match is good enough
  if (zmssd_best < PatchScore::threshold()) {
    if (options_.subpix_refinement) {
      MatchResultState res =
          FindLocalMatch(cur_frame, epi_dir_image, search_level, px_cur);
      if (res != MatchResultState::kSuccess) return {res};
    }

    Eigen::Vector3d f_cur;
    cur_frame.cam->backProject3(px_cur, &f_cur);
    f_cur.normalize();
    return {DepthFromTriangulation(T_cur_ref, ref_ftr.f, f_cur, &depth), px_cur,
            f_cur, search_level};
  } else
    return {MatchResultState::kFailScore};
}

std::string DirectMatch::ToDebugString(const MatchResultState& result) {
  std::string result_str = "success";
  switch (result) {
    case MatchResultState::kFailScore:
      result_str = "fail score";
      break;
    case MatchResultState::kFailTriangulation:
      result_str = "fail triangulation";
      break;
    case MatchResultState::kFailVisibility:
      result_str = "fail visibility";
      break;
    case MatchResultState::kFailWarp:
      result_str = "fail warp";
      break;
    case MatchResultState::kFailAlignment:
      result_str = "fail alignment";
      break;
    case MatchResultState::kFailRange:
      result_str = "fail range";
      break;
    case MatchResultState::kFailAngle:
      result_str = "fail angle";
      break;
    case MatchResultState::kFailCloseView:
      result_str = "fail close view";
      break;
    case MatchResultState::kFailLock:
      result_str = "fail lock";
      break;
    default:
      result_str = "unknown";
  }
  return result_str;
}

MatchResultState DirectMatch::FindLocalMatch(
    const Frame& frame, const GradientVector& direction,
    const int patch_level, Keypoint& px_cur) {
  Keypoint px_scaled(px_cur / (1 << patch_level));
  bool res;
  if (options_.align_1d) {
    double h_inv_;
    res = feature_alignment::align1D(
        frame.img_pyr[patch_level], direction, patch_with_border_, patch_,
        options_.align_max_iter, options_.affine_est_offset,
        options_.affine_est_gain, &px_scaled, &h_inv_);
  } else {
    res = feature_alignment::align2D(
        frame.img_pyr[patch_level], patch_with_border_, patch_,
        options_.align_max_iter, options_.affine_est_offset,
        options_.affine_est_gain, px_scaled, options_.min_update_squared);
  };
  if (!res) return MatchResultState::kFailAlignment;

  px_cur = px_scaled * (1 << patch_level);
  return MatchResultState::kSuccess;
}

bool DirectMatch::UpdateZMSSD(const Frame& frame, const Eigen::Vector2i& pxi,
                              const int patch_level,
                              const PatchScore& patch_score, int* zmssd_best) {
  // TODO interpolation would probably be a good idea
  uint8_t* cur_patch_ptr =
      frame.img_pyr[patch_level].data +
      (pxi[1] - kHalfPatchSize) * frame.img_pyr[patch_level].step +
      (pxi[0] - kHalfPatchSize);
  int zmssd =
      patch_score.computeScore(cur_patch_ptr, frame.img_pyr[patch_level].step);

  if (zmssd < *zmssd_best) {
    *zmssd_best = zmssd;
    return true;
  } else
    return false;
}

bool DirectMatch::IsPatchWithinImage(const Frame& frame,
                                     const Eigen::Vector2i& pxi,
                                     const int patch_level) {
  return !(
      pxi[0] < kPatchSize || pxi[1] < kPatchSize ||
      pxi[0] >= (static_cast<int>(frame.image_size.x() / (1 << patch_level)) -
                 kPatchSize) ||
      pxi[1] >= (static_cast<int>(frame.image_size.y() / (1 << patch_level)) -
                 kPatchSize));
}

void DirectMatch::ScanEpipolarLine(const Frame& frame, const Eigen::Vector3d& A,
                                   const Eigen::Vector3d& B,
                                   const Eigen::Vector3d& C,
                                   const PatchScore& patch_score,
                                   const int patch_level, Keypoint* image_best,
                                   int* zmssd_best) {
  if (options_.scan_on_unit_sphere)
    ScanEpipolarUnitSphere(frame, A, B, C, patch_score, patch_level, image_best,
                           zmssd_best);
  else
    ScanEpipolarUnitPlane(frame, A, B, C, patch_score, patch_level, image_best,
                          zmssd_best);
}

void DirectMatch::ScanEpipolarUnitPlane(
    const Frame& frame, const Eigen::Vector3d& A, const Eigen::Vector3d& B,
    const Eigen::Vector3d& C, const PatchScore& patch_score,
    const int patch_level, Keypoint* image_best, int* zmssd_best) {
  CHECK(false);
#if 0
  
  // if there're too many steps, we only search for a limited range around the
  // center
  //    while keeping the step size small enough to check each pixel on the
  //    image plane
  size_t n_steps = epi_length_pyramid_ / 0.7;  // one step per pixel
  Eigen::Vector2d step = (vk::project2(A) - vk::project2(B)) / n_steps;
  if (n_steps > options_.max_epi_search_steps) {
    /* TODO
    printf("WARNING: skip epipolar search: %d evaluations, px_lenght=%f,
    d_min=%f, d_max=%f.\n", n_steps, epi_length_, d_min_inv, d_max_inv);
    */
    n_steps = options_.max_epi_search_steps;
  }
  // now we sample along the epipolar line
  Eigen::Vector2d uv_C = vk::project2(C);
  Eigen::Vector2d uv = uv_C;
  Eigen::Vector2d uv_best = uv;
  bool forward = true;
  Eigen::Vector2i last_checked_pxi(0, 0);

  for (size_t i = 0; i < n_steps; ++i, uv += step) {
    Eigen::Vector2d px;
    frame.cam()->project3(vk::unproject2d(uv), &px);
    Eigen::Vector2i pxi(
        px[0] / (1 << patch_level) + 0.5,
        px[1] / (1 << patch_level) + 0.5);  // +0.5 to round to closest int

    if (pxi == last_checked_pxi) continue;
    last_checked_pxi = pxi;

    // check if the patch is full within the new frame
    if (!IsPatchWithinImage(frame, pxi, patch_level)) {
      // change search direction if pixel is out of field of view
      if (forward) {
        // reverse search direction
        i = n_steps * 0.5;
        step = -step;
        uv = uv_C;
        forward = false;
        continue;
      } else
        break;
    }

    if (UpdateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
      uv_best = uv;

    if (forward && i > n_steps * 0.5) {
      // reverse search direction
      step = -step;
      uv = uv_C;
      forward = false;
    }
  }

  // convert uv_best to image coordinates
  Eigen::Vector2d projected;
  //
  //
  CHECK(false) << "Need check vk::unproject2d...";
  // frame.cam->project3(vk::unproject2d(uv_best), &projected);
  *image_best = projected.cast<double>();
#endif
}

void DirectMatch::ScanEpipolarUnitSphere(
    const Frame& frame, const Eigen::Vector3d& A, const Eigen::Vector3d& B,
    const Eigen::Vector3d& C, const PatchScore& patch_score,
    const int patch_level, Keypoint* image_best, int* zmssd_best) {
#if 0
  size_t n_steps =
      epi_length_pyramid_ / 0.7;  // TODO(zzc): better way of doing this?
  n_steps = n_steps > options_.max_epi_search_steps
                ? options_.max_epi_search_steps
                : n_steps;
  size_t half_steps = n_steps / 2;

  // calculate the step in angle
  Eigen::Vector3d f_A = A.normalized();
  Eigen::Vector3d f_B = B.normalized();
  double step = std::acos(f_A.dot(f_B)) / n_steps;

  // calculate the rotation axis: positive angle -> toward A
  kindr::minimal::AngleAxis rotation_B_to_A;
  rotation_B_to_A.setAxis((f_B.cross(f_A)).normalized());

  // search around center
  Eigen::Vector3d f_C = C.normalized();
  Eigen::Vector3d f = f_C;
  Eigen::Vector3d f_best = f_C;
  Eigen::Vector2i last_checked_pxi(0, 0);
  for (size_t i = 0; i < n_steps; i++) {
    // TODO(zzc): more compact
    // rotation angle w.r.t. f_C
    double angle = 0.0;
    if (i < half_steps)  // f_A <-- f_C
      angle = i * step;
    else
      angle = (i - half_steps) * (-step);  // f_C --> f_B
    rotation_B_to_A.setAngle(angle);

    // current sample on unit sphere
    f = rotation_B_to_A.rotate(f_C);

    // back project to image plane
    Eigen::Vector2d px;
    frame.cam()->project3(f, &px);
    Eigen::Vector2i pxi(
        px[0] / (1 << patch_level) + 0.5,
        px[1] / (1 << patch_level) + 0.5);  // +0.5 to round to closest int
    if (pxi == last_checked_pxi) continue;
    last_checked_pxi = pxi;

    // is within image?
    // TODO(zzc): FIX use visibility check in camera model
    if (!isPatchWithinImage(frame, pxi, patch_level)) {
      if (i < half_steps)  // f_A <-- f_C to f_C --> f_B
      {
        i = half_steps;
        continue;
      } else  // end of search
        break;
    }

    // update ZMSSD
    if (updateZMSSD(frame, pxi, patch_level, patch_score, zmssd_best))
      f_best = f;
  }

  // backproject to image plane
  Eigen::Vector2d projected;
  frame.cam()->project3(f_best, &projected);
  *image_best = projected.cast<svo::FloatType>();
#endif
}
//
MatchResultState DirectMatch::DepthFromTriangulation(
    const transform::Rigid3d& T_search_ref, const Eigen::Vector3d& f_ref,
    const Eigen::Vector3d& f_cur, double* depth) {
  Eigen::Matrix<double, 3, 2> A;
  A << T_search_ref.rotation() * f_ref, f_cur;
  const Eigen::Matrix2d AtA = A.transpose() * A;
  if (AtA.determinant() < 0.000001) return MatchResultState::kFailTriangulation;
  const Eigen::Vector2d depth2 =
      -AtA.inverse() * A.transpose() * T_search_ref.translation();
  (*depth) = std::fabs(depth2[0]);
  return MatchResultState::kSuccess;
}

}  // namespace match
}  // namespace mapping
}  // namespace jarvis
