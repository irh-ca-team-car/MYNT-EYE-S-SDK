// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "mynteye/api/processor/rectify_processor.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <utility>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

cv::Mat RectifyProcessor4::rectifyrad(const cv::Mat &R) {
  cv::Mat r_vec;
  cv::Rodrigues(R, r_vec);
  //  pi/180 = x/179 ==> x = 3.1241
  double rad = cv::norm(r_vec);
  if (rad >= 3.1241) {
    cv::Mat r_dir;
    cv::normalize(r_vec, r_dir);
    cv::Mat r = r_dir * (3.1415926 - rad);
    cv::Mat r_r;
    cv::Rodrigues(r, r_r);
    return r_r.clone();
  }
  return R.clone();
}
void setO(cv::Mat& m)
{
  for (int c = 0; c < m.cols; c++)
    for (int r = 0; r < m.rows; r++)
      m.at<double>(c, r) = 0;
}
#include <execinfo.h>
void RectifyProcessor4::stereoRectify(
    models::CameraPtr leftOdo, models::CameraPtr rightOdo, const cv::Mat &K1,
    const cv::Mat &K2, const cv::Mat &D1, const cv::Mat &D2, cv::Size imageSize,
    const cv::Mat &matR, const cv::Mat &matT, cv::Mat &_R1, cv::Mat &_R2,
    cv::Mat &_P1, cv::Mat &_P2, double &T_mul_f, double &cx1_min_cx2, int flags,
    double alpha, cv::Size newImgSize) {
  alpha = _alpha;
  cv::Rect_<float> inner1, inner2, outer1, outer2;

  cv::Mat om = cv::Mat(3, 1, CV_64FC1);
  cv::Mat t = cv::Mat(3, 1, CV_64FC1);
  cv::Mat uu = cv::Mat(3, 1, CV_64FC1);
  cv::Mat r_r = cv::Mat(3, 3, CV_64FC1);
  cv::Mat pp = cv::Mat(3, 4, CV_64FC1);
  cv::Mat ww = cv::Mat(3, 1, CV_64FC1);  // temps
  cv::Mat w3 = cv::Mat(3, 1, CV_64FC1);  // temps
  cv::Mat wR = cv::Mat(3, 3, CV_64FC1);
  cv::Mat Z = cv::Mat(3, 1, CV_64FC1);
  cv::Mat Ri = cv::Mat(3, 3, CV_64FC1);
  double nx = imageSize.width, ny = imageSize.height;
  int i, k;
  double nt, nw;

  setO(om);
  setO(t);
  setO(uu);
  setO(r_r);
  setO(pp);
  setO(ww);
  setO(w3);
  setO(wR);
  setO(Z);
  setO(Ri);

  if (matR.rows == 3 && matR.cols == 3)
    cv::Rodrigues(matR, om);
  else
    matR.convertTo(om, CV_64FC1);
  // cv::convertTo(matR,om, 1,0);// it's already a rotation vector
  om.convertTo(om, CV_64FC1, -0.5);
  // cv::convertScaleAbs(om, om, -0.5);  // get average rotation
  cv::Rodrigues(om, r_r);  // rotate cameras to same orientation by averaging
  // cv::multiply(matT,r_r,t);
  // cvMatMul(&r_r, matT, &t)
  // cvMatMulAdd(&r_r, matT,NULL, &t);

  cv::gemm(r_r, matT, 1, __null, 0, t, 0);

  int idx = fabs(t.at<double>(0)) > fabs(t.at<double>(1)) ? 0 : 1;
  // if idx == 0
  //   e1 = T / ||T||
  //   e2 = e1 x [0,0,1]

  // if idx == 1
  //   e2 = T / ||T||
  //   e1 = e2 x [0,0,1]

  // e3 = e1 x e2

  uu.at<double>(2) = 1;

#define cvCrossProduct(A, B, C) C = A.cross(B)

  cvCrossProduct(uu, t, ww);
  nt = cv::norm(t, cv::NORM_L2);
  nw = cv::norm(ww, cv::NORM_L2);
  ww.convertTo(ww, CV_64F, 1 / nw);
  cvCrossProduct(t, ww, w3);
  nw = cv::norm(w3, cv::NORM_L2);
  w3.convertTo(w3, CV_64F, 1 / nw);
  uu.at<double>(2)=0;
 
  // ERROR:The output of this loop does not fit the desired result
  for (i = 0; i < 3; ++i) {
    wR.at<double>(idx,i) = (-(t.at<double>(i)) / (nt));
    wR.at<double>(idx ^ 1,i) = (-ww.at<double>(i));
    wR.at<double>(2,i) = (w3.at<double>(i) * (1 - 2 * idx));  // if idx == 1 -> opposite direction
  }

  // apply to both views

  // THIS GEMM DOES NOT BEHAVE CORRECTLY
  cv::gemm(wR, r_r.t(), 1, NULL, 0, Ri);
  // cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);

  Ri.convertTo(_R1, CV_64F, 1, 0);

  cv::gemm(wR, r_r, 1, NULL, 0, Ri);
  // cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
  Ri.convertTo(_R2, CV_64F, 1, 0);
  cv::gemm(Ri, matT, 1, NULL, 0, t, 0);
  // cv::MatMul(&Ri, matT, &t);
  // calculate projection/camera matrices
  // these contain the relevant rectified image internal params (fx, fy=fx, cx,
  // cy)
  double fc_new = DBL_MAX;
  cv::Point2d cc_new[2] = {{0, 0}, {0, 0}};
 
  newImgSize =
      newImgSize.width * newImgSize.height != 0 ? newImgSize : imageSize;
  const double ratio_x =
      static_cast<double>(newImgSize.width) / imageSize.width / 2;
  const double ratio_y =
      static_cast<double>(newImgSize.height) / imageSize.height / 2;
  const double ratio = idx == 1 ? ratio_x : ratio_y;
  fc_new = (K1.at<double>(idx ^ 1, idx ^ 1) + K2.at<double>(idx ^ 1, idx ^ 1)) * ratio;
  for (k = 0; k < 2; k++) {
    cv::Mat pts = cv::Mat(1, 4, CV_32FC2);
    cv::Mat pts_3 = cv::Mat(1, 4, CV_32FC3);
    // Eigen::Vector2d a;
    // Eigen::Vector3d b;
    models::Vector2d a(2, 1);
    models::Vector3d b(3, 1);

    for (i = 0; i < 4; i++) {
      int j = (i < 2) ? 0 : 1;
      a(0) = static_cast<float>((i % 2) * (nx));
      a(1) = static_cast<float>(j * (ny));
      if (0 == k) {
        leftOdo->liftProjective(a, b);
      } else {
        rightOdo->liftProjective(a, b);
      }
      auto pt = pts.at<cv::Point2f>(i);
      auto pt3 = pts_3.at<cv::Point3f>(i);

      pt.x = pt3.x = b(0) / b(2);
      pt.y = pt3.y = pt3.z = b(1) / b(2);

      pts.at<cv::Point2f>(i)=pt;
      pts_3.at<cv::Point3f>(i)=pt3;

    }
    cv::convertPointsToHomogeneous(pts, pts_3);
    // cvConvertPointsHomogeneous(&pts, &pts_3);

    // Change camera matrix to have cc=[0,0] and fc = fc_new
    cv::Mat A_tmp = cv::Mat(3, 3, CV_64F);
    setO(A_tmp);
    A_tmp.at<double>(0, 0) = fc_new;
    A_tmp.at<double>(1, 1) = fc_new;
    A_tmp.at<double>(0, 2) = 0.0;
    A_tmp.at<double>(1, 2) = 0.0;

    cv::projectPoints(
        pts_3, k == 0 ? _R1 : _R2, Z, A_tmp, cv::Mat(1, 4, CV_64F), pts);
    // cvProjectPoints2(&pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts);

    cv::Scalar avg = cv::mean(pts);
    cc_new[k].x = (nx) / 2 - avg.val[0];
    cc_new[k].y = (ny) / 2 - avg.val[1];
  }
  if (flags & cv::CALIB_ZERO_DISPARITY) {
    cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x) * 0.5;
    cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y) * 0.5;
  } else if (idx == 0) {
    // horizontal stereo
    cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y) * 0.5;
  } else {
    // vertical stereo
    cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x) * 0.5;
  }
  setO(pp);

  pp.at<double>(0,0) = pp.at<double>(1,1) = fc_new;
  pp.at<double>(0,2) = cc_new[0].x;
  pp.at<double>(1,2) = cc_new[0].y;
  pp.at<double>(2,2) = 1;
  pp.convertTo(_P1, _P1.type());
  // cv::convertScaleAbs(pp, _P1);
  pp.at<double>(0,2) = cc_new[1].x;
  pp.at<double>(1,2) = cc_new[1].y;
  pp.at<double>(idx,3) = t.at<double>(idx) * fc_new;  // baseline * focal length
  T_mul_f = 0. - t.at<double>(idx) * fc_new;
  pp.convertTo(_P2, _P2.type());
  // cv::convertScaleAbs(pp, _P2);

  _alpha = MIN(alpha, 1.);
  {
    newImgSize =
        newImgSize.width * newImgSize.height != 0 ? newImgSize : imageSize;
    double cx1_0 = cc_new[0].x;
    double cy1_0 = cc_new[0].y;
    double cx2_0 = cc_new[1].x;
    double cy2_0 = cc_new[1].y;
    double cx1 = newImgSize.width * cx1_0 / imageSize.width;
    double cy1 = newImgSize.height * cy1_0 / imageSize.height;
    double cx2 = newImgSize.width * cx2_0 / imageSize.width;
    double cy2 = newImgSize.height * cy2_0 / imageSize.height;
    double s = 1.;

    if (_alpha >= 0) {
      double s0 = std::max(
          std::max(
              std::max(
                  (double)cx1 / (cx1_0 - inner1.x),
                  (double)cy1 / (cy1_0 - inner1.y)),
              (double)(newImgSize.width - cx1) /
                  (inner1.x + inner1.width - cx1_0)),
          (double)(newImgSize.height - cy1) /
              (inner1.y + inner1.height - cy1_0));
      s0 = std::max(
          std::max(
              std::max(
                  std::max(
                      (double)cx2 / (cx2_0 - inner2.x),
                      (double)cy2 / (cy2_0 - inner2.y)),
                  (double)(newImgSize.width - cx2) /
                      (inner2.x + inner2.width - cx2_0)),
              (double)(newImgSize.height - cy2) /
                  (inner2.y + inner2.height - cy2_0)),
          s0);

      double s1 = std::min(
          std::min(
              std::min(
                  (double)cx1 / (cx1_0 - outer1.x),
                  (double)cy1 / (cy1_0 - outer1.y)),
              (double)(newImgSize.width - cx1) /
                  (outer1.x + outer1.width - cx1_0)),
          (double)(newImgSize.height - cy1) /
              (outer1.y + outer1.height - cy1_0));
      s1 = std::min(
          std::min(
              std::min(
                  std::min(
                      (double)cx2 / (cx2_0 - outer2.x),
                      (double)cy2 / (cy2_0 - outer2.y)),
                  (double)(newImgSize.width - cx2) /
                      (outer2.x + outer2.width - cx2_0)),
              (double)(newImgSize.height - cy2) /
                  (outer2.y + outer2.height - cy2_0)),
          s1);
      s = s0 * (1 - alpha) + s1 * alpha;
    }

    fc_new *= s;
    cc_new[0] = cv::Point2d(cx1, cy1);
    cc_new[1] = cv::Point2d(cx2, cy2);
#define cvmSet(A, B, C, D) A.at<double>(B, C) = D
    cvmSet(_P1, 0, 0, fc_new);
    cvmSet(_P1, 1, 1, fc_new);
    cvmSet(_P1, 0, 2, cx1);
    cvmSet(_P1, 1, 2, cy1);

    cvmSet(_P2, 0, 0, fc_new);
    cvmSet(_P2, 1, 1, fc_new);
    cvmSet(_P2, 0, 2, cx2);
    cvmSet(_P2, 1, 2, cy2);

    cvmSet(_P2, idx, 3, s * _P2.at<double>(idx, 3));
    cvmSet(_P2, idx, 3, s * _P2.at<double>(idx, 3));

    cx1_min_cx2 = -(cx1 - cx2);
    // std::cout << "info_pair.T_mul_f :" << *T_mul_f << std::endl;
    // std::cout << "info_pair.cx1_minus_cx2 :" << *cx1_min_cx2 << std::endl;
  }
}

// Eigen::Matrix4d RectifyProcessor::loadT(const mynteye::Extrinsics& in) {
// subEigen
models::Matrix4d RectifyProcessor4::loadT(const mynteye::Extrinsics &in) {
  models::Matrix3d R(3);
  R << in.rotation[0][0] << in.rotation[0][1] << in.rotation[0][2]
    << in.rotation[1][0] << in.rotation[1][1] << in.rotation[1][2]
    << in.rotation[2][0] << in.rotation[2][1] << in.rotation[2][2];

  double t_x = in.translation[0];
  double t_y = in.translation[1];
  double t_z = in.translation[2];

  models::Quaterniond q(R);
  q.normalize();
  models::Matrix4d T(4);
  T(3, 3) = 1;
  T.topLeftCorner<3, 3>() = q.toRotationMatrix();
  models::Vector3d t(3, 1);
  t << t_x << t_y << t_z;
  T.topRightCorner<3, 1>() = t;

  return T;
}

void RectifyProcessor4::loadCameraMatrix(
    cv::Mat &K, cv::Mat &D,                 // NOLINT
    cv::Size &image_size,                   // NOLINT
    struct CameraROSMsgInfo &calib_data) {  // NOLINT
  K = cv::Mat(3, 3, CV_64F, calib_data.K);
  std::size_t d_length = 4;
  D = cv::Mat(1, d_length, CV_64F, calib_data.D);
  image_size = cv::Size(calib_data.width, calib_data.height);
}

struct CameraROSMsgInfo RectifyProcessor4::getCalibMatData(
    const mynteye::IntrinsicsEquidistant &in) {
  struct CameraROSMsgInfo calib_mat_data;
  calib_mat_data.distortion_model = "KANNALA_BRANDT";
  calib_mat_data.height = in.height;
  calib_mat_data.width = in.width;

  for (unsigned int i = 0; i < 4; i++) {
    calib_mat_data.D[i] = in.coeffs[i];
  }

  calib_mat_data.K[0] = in.coeffs[4];  // mu
  calib_mat_data.K[4] = in.coeffs[5];  // mv();
  calib_mat_data.K[2] = in.coeffs[6];  // u0();
  calib_mat_data.K[5] = in.coeffs[7];  // v0();
  calib_mat_data.K[8] = 1;
  return calib_mat_data;
}
double notNAN(double e) {
  if (e == e)
    return e;
  else
    return 0;
}
std::shared_ptr<struct CameraROSMsgInfoPair> RectifyProcessor4::stereoRectify(
    models::CameraPtr leftOdo, models::CameraPtr rightOdo,
    mynteye::IntrinsicsEquidistant in_left,
    mynteye::IntrinsicsEquidistant in_right,
    mynteye::Extrinsics ex_right_to_left) {
  // Eigen::Matrix4d T = loadT(ex_right_to_left);
  // Eigen::Matrix3d R = T.topLeftCorner<3, 3>();
  // Eigen::Vector3d t = T.topRightCorner<3, 1>();
  models::Matrix4d T = loadT(ex_right_to_left);
  models::Matrix3d R;
  R = T.topLeftCorner<3, 3>();
  models::Vector3d t = T.topRightCorner<3, 1>();
  // cv::Mat cv_R, cv_t;
  // cv::eigen2cv(R, cv_R);
  cv::Mat cv_R(3, 3, CV_64FC1);
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      cv_R.at<double>(i, j) = R(i, j);
    }
  }
  // cv::eigen2cv(t, cv_t);
  cv::Mat cv_t(3, 1, CV_64FC1);
  for (int i = 0; i < 3; ++i) {
    cv_t.at<double>(i, 0) = t(i, 0);
  }
  cv::Mat K1, D1, K2, D2;
  cv::Size image_size1, image_size2;

  struct CameraROSMsgInfo calib_mat_data_left = getCalibMatData(in_left);
  struct CameraROSMsgInfo calib_mat_data_right = getCalibMatData(in_right);

  loadCameraMatrix(K1, D1, image_size1, calib_mat_data_left);
  loadCameraMatrix(K2, D2, image_size2, calib_mat_data_right);

  cv::Mat R1 = cv::Mat(cv::Size(3, 3), CV_64F);
  cv::Mat R2 = cv::Mat(cv::Size(3, 3), CV_64F);
  cv::Mat P1 = cv::Mat(3, 4, CV_64F);
  cv::Mat P2 = cv::Mat(3, 4, CV_64F);

  cv::Mat c_R = cv_R, c_t = cv_t;
  cv::Mat c_K1 = K1, c_K2 = K2, c_D1 = D1, c_D2 = D2;
  cv::Mat c_R1 = cv::Mat(R1), c_R2 = cv::Mat(R2), c_P1 = cv::Mat(P1),
          c_P2 = cv::Mat(P2);
  double T_mul_f;
  double cx1_min_cx2;

  stereoRectify(
      leftOdo, rightOdo, (c_K1), (c_K2), (c_D1), (c_D2), image_size1, (c_R),
      (c_t), (c_R1), (c_R2), (c_P1), (c_P2), T_mul_f, cx1_min_cx2);
  TRACE;
#ifdef _DOUTPUT
  std::cout << "K1: " << K1 << std::endl;
  std::cout << "D1: " << D1 << std::endl;
  std::cout << "K2: " << K2 << std::endl;
  std::cout << "D2: " << D2 << std::endl;
  std::cout << "R: " << cv_R << std::endl;
  std::cout << "t: " << cv_t << std::endl;
  std::cout << "R1: " << R1 << std::endl;
  std::cout << "R2: " << R2 << std::endl;
  std::cout << "P1: " << P1 << std::endl;
  std::cout << "P2: " << P2 << std::endl;
#endif
  R1 = rectifyrad((R1));
  R2 = rectifyrad((R2));
  TRACE;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 4; j++) {
      calib_mat_data_left.P[i * 4 + j] = notNAN(P1.at<double>(i, j));
      calib_mat_data_right.P[i * 4 + j] = notNAN(P2.at<double>(i, j));
    }
  }
  TRACE;
  for (std::size_t i = 0; i < 3; i++) {
    for (std::size_t j = 0; j < 3; j++) {
      calib_mat_data_left.R[i * 3 + j] = notNAN(R1.at<double>(i, j));
      calib_mat_data_right.R[i * 3 + j] = notNAN(R2.at<double>(i, j));
    }
  }
  TRACE;
  auto info_pair = std::make_shared<struct CameraROSMsgInfoPair>();
  info_pair->left = calib_mat_data_left;
  info_pair->right = calib_mat_data_right;
  info_pair->T_mul_f = notNAN(T_mul_f);
  info_pair->cx1_minus_cx2 = notNAN(cx1_min_cx2);
  for (std::size_t i = 0; i < 3 * 4; i++) {
    info_pair->P[i] = notNAN(calib_mat_data_left.P[i]);
  }
  TRACE;
  info_pair->R[0] = notNAN(ex_right_to_left.rotation[0][0]);
  info_pair->R[1] = notNAN(ex_right_to_left.rotation[0][1]);
  info_pair->R[2] = notNAN(ex_right_to_left.rotation[0][2]);
  info_pair->R[3] = notNAN(ex_right_to_left.rotation[1][0]);
  info_pair->R[4] = notNAN(ex_right_to_left.rotation[1][1]);
  info_pair->R[5] = notNAN(ex_right_to_left.rotation[1][2]);
  info_pair->R[6] = notNAN(ex_right_to_left.rotation[2][0]);
  info_pair->R[7] = notNAN(ex_right_to_left.rotation[2][1]);
  info_pair->R[8] = notNAN(ex_right_to_left.rotation[2][2]);
  TRACE;
  return info_pair;  // std::make_shared<struct
                     // CameraROSMsgInfoPair>(info_pair);
}

models::CameraPtr RectifyProcessor4::generateCameraFromIntrinsicsEquidistant(
    const mynteye::IntrinsicsEquidistant &in) {
  models::EquidistantCameraPtr camera(new models::EquidistantCamera(
      "KANNALA_BRANDT", in.width, in.height, in.coeffs[0], in.coeffs[1],
      in.coeffs[2], in.coeffs[3], in.coeffs[4], in.coeffs[5], in.coeffs[6],
      in.coeffs[7]));
  return camera;
}

void RectifyProcessor4::InitParams(
    IntrinsicsEquidistant in_left, IntrinsicsEquidistant in_right,
    Extrinsics ex_right_to_left) {
  TRACE;
  calib_model = CalibrationModel::KANNALA_BRANDT;
  in_left.ResizeIntrinsics();
  in_right.ResizeIntrinsics();
  in_left_cur = in_left;
  in_right_cur = in_right;
  ex_right_to_left_cur = ex_right_to_left;
  TRACE;
  models::CameraPtr camera_odo_ptr_left =
      generateCameraFromIntrinsicsEquidistant(in_left);
  models::CameraPtr camera_odo_ptr_right =
      generateCameraFromIntrinsicsEquidistant(in_right);
  TRACE;
  auto calib_info_tmp = stereoRectify(
      camera_odo_ptr_left, camera_odo_ptr_right, in_left, in_right,
      ex_right_to_left);
  TRACE;
  *calib_infos = *calib_info_tmp;
  cv::Mat rect_R_l = cv::Mat::eye(3, 3, CV_32F),
          rect_R_r = cv::Mat::eye(3, 3, CV_32F);
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      rect_R_l.at<float>(i, j) = calib_infos->left.R[i * 3 + j];
      rect_R_r.at<float>(i, j) = calib_infos->right.R[i * 3 + j];
    }
  }
  TRACE;
  double left_f[] = {calib_infos->left.P[0], calib_infos->left.P[5]};
  double left_center[] = {calib_infos->left.P[2], calib_infos->left.P[6]};
  double right_f[] = {calib_infos->right.P[0], calib_infos->right.P[5]};
  double right_center[] = {calib_infos->right.P[2], calib_infos->right.P[6]};
  TRACE;
  camera_odo_ptr_left->initUndistortRectifyMap(
      map11, map12, left_f[0], left_f[1], cv::Size(0, 0), left_center[0],
      left_center[1], rect_R_l);
  camera_odo_ptr_right->initUndistortRectifyMap(
      map21, map22, right_f[0], right_f[1], cv::Size(0, 0), right_center[0],
      right_center[1], rect_R_r);
  TRACE;
}

const char RectifyProcessor4::NAME[] = "RectifyProcessor";

RectifyProcessor4::RectifyProcessor4(
    std::shared_ptr<IntrinsicsBase> intr_left,
    std::shared_ptr<IntrinsicsBase> intr_right,
    std::shared_ptr<Extrinsics> extr, std::int32_t proc_period)
    : Processor(std::move(proc_period)),
      calib_model(CalibrationModel::UNKNOW),
      _alpha(-1) {
  calib_infos = std::make_shared<struct CameraROSMsgInfoPair>();
  InitParams(
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(intr_left),
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(intr_right), *extr);
}

RectifyProcessor4::~RectifyProcessor4() {
  VLOG(2) << __func__;
}

std::string RectifyProcessor4::Name() {
  return NAME;
}

void RectifyProcessor4::ReloadImageParams(
    std::shared_ptr<IntrinsicsBase> intr_left,
    std::shared_ptr<IntrinsicsBase> intr_right,
    std::shared_ptr<Extrinsics> extr) {
  InitParams(
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(intr_left),
      *std::dynamic_pointer_cast<IntrinsicsEquidistant>(intr_right), *extr);
}

Object *RectifyProcessor4::OnCreateOutput() {
  return new ObjMat2();
}

bool RectifyProcessor4::SetRectifyAlpha(float alpha) {
  _alpha = alpha;
  ReloadImageParams();
  return true;
}

bool RectifyProcessor4::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat2 *input = Object::Cast<ObjMat2>(in);
  ObjMat2 *output = Object::Cast<ObjMat2>(out);
  cv::remap(input->first, output->first, map11, map12, cv::INTER_LINEAR);
  cv::remap(input->second, output->second, map21, map22, cv::INTER_LINEAR);
  output->first_id = input->first_id;
  output->first_data = input->first_data;
  output->second_id = input->second_id;
  output->second_data = input->second_data;
  return true;
}

MYNTEYE_END_NAMESPACE
