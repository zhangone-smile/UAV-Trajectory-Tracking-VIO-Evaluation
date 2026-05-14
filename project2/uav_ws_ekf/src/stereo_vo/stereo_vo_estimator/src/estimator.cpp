#include "estimator.h"
#include <algorithm>

void drawImage(const cv::Mat &img, const vector<cv::Point2f> &pts, string name)
{
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

Estimator::Estimator()
{
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset()
{
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

bool Estimator::setParameter()
{
  if (TIC.size() < 2 || RIC.size() < 2)
  {
    ROS_ERROR("Stereo VO requires two camera extrinsics.");
    return false;
  }

  for (int i = 0; i < 2; i++)
  {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl
         << ric[i] << endl
         << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  if (!readIntrinsicParameter(CAM_NAMES))
  {
    return false;
  }

  // transform between left and right camera
  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;

  latest_time = ros::Time(0.0);
  rel_key_time = ros::Time(0.0);
  latest_P.setZero();
  latest_Q.setIdentity();
  latest_rel_P.setZero();
  latest_rel_Q.setIdentity();

  return true;
}

bool Estimator::readIntrinsicParameter(const vector<string> &calib_file)
{
  if (calib_file.size() < 2)
  {
    ROS_ERROR("Stereo VO requires two camera calibration files.");
    return false;
  }

  for (size_t i = 0; i < calib_file.size(); i++)
  {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    if (!camera)
    {
      ROS_ERROR("Failed to load camera calibration file: %s", calib_file[i].c_str());
      return false;
    }
    m_camera.push_back(camera);
  }
  return true;
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat &_img, const cv::Mat &_img1)
{
  if (_img.empty() || _img1.empty())
  {
    ROS_WARN_THROTTLE(1.0, "Stereo VO received an empty stereo image.");
    return false;
  }

  if (_img.size() != _img1.size())
  {
    ROS_ERROR_THROTTLE(1.0, "Stereo VO image size mismatch: left=%dx%d right=%dx%d",
                       _img.cols, _img.rows, _img1.cols, _img1.rows);
    return false;
  }

  if (_img.type() != CV_8UC1 || _img1.type() != CV_8UC1)
  {
    ROS_ERROR_THROTTLE(1.0, "Stereo VO expects mono8 images: left_type=%d right_type=%d",
                       _img.type(), _img1.type());
    return false;
  }

  if (m_camera.size() < 2 || !m_camera[0] || !m_camera[1])
  {
    ROS_ERROR_THROTTLE(1.0, "Stereo VO camera intrinsics are not initialized.");
    return false;
  }

  if (fail_cnt > 20)
  {
    reset();
  }
  ROS_DEBUG_STREAM("receive new image");

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> key_pts_3d;

  c_R_k.setIdentity();
  c_t_k.setZero();

  if (init_finish)
  {
    // 工作流步骤 1: 帧间特征跟踪 (纵向时间线)
    // 利用上一帧(关键帧)已知的3D点，在当前帧左图中寻找对应的2D像素位置
    if (trackFeatureBetweenFrames(key_frame, _img, key_pts_3d, left_pts_2d))
    {
      // 工作流步骤 2: 位姿解算 (纵向时间线)
      // 通过3D-2D对应关系，解算当前相机相对于关键帧的旋转R和平移t
      if (!estimateTBetweenFrames(key_pts_3d, left_pts_2d, c_R_k, c_t_k))
      {
        fail_cnt++;
        c_R_k.setIdentity();
        c_t_k.setZero();
      }
      else
      {
        fail_cnt = 0;
      }
    }
    else
    {
      fail_cnt++;
    }
  }

  // 工作流步骤 3: 提取新特征 (补充观测)
  // 当老特征点由于运动移出视野时，补充新的特征点以维持跟踪规模
  extractNewFeatures(_img, cur_frame.uv);

  // 计算当前帧在世界坐标系下的绝对位姿
  cur_frame.w_R_c = key_frame.w_R_c * c_R_k.transpose();
  cur_frame.w_t_c = key_frame.w_R_c * (-c_R_k.transpose() * c_t_k) + key_frame.w_t_c;

  // 工作流步骤 4: 左右目立体匹配 (横向空间线)
  // 将当前左图的新特征点匹配到右图中，用于后续深度计算
  if (trackFeatureLeftRight(_img, _img1, cur_frame.uv, right_pts_2d))
  {
    // 工作流步骤 5: 三角化生成3D点 (横向空间线)
    // 利用左右目视差和基线，计算新特征点的3D深度，作为未来帧跟踪的参考
    generate3dPoints(cur_frame.uv, right_pts_2d, cur_frame.xyz, cur_frame.uv);
  }

  // 关键帧更新逻辑：如果位移、旋转过大或特征点太少，则更换关键帧
  bool change_keyframe = c_t_k.norm() > TRANSLATION_THRESHOLD ||
                         acos(std::max(-1.0, std::min(1.0, Quaterniond(c_R_k).w()))) * 2.0 > ROTATION_THRESHOLD ||
                         key_pts_3d.size() < FEATURE_THRESHOLD || !init_finish;
  if (change_keyframe)
  {
    const double keyframe_trans = c_t_k.norm();
    const double keyframe_rot = acos(std::max(-1.0, std::min(1.0, Quaterniond(c_R_k).w()))) * 2.0;
    key_frame = cur_frame;
    c_R_k.setIdentity();
    c_t_k.setZero();
    ROS_INFO_STREAM("[stereo_vo][keyframe] change to t=" << cur_frame.frame_time.toSec()
                    << " reason: trans=" << keyframe_trans
                    << " rot=" << keyframe_rot
                    << " tracked_key_pts=" << key_pts_3d.size()
                    << " init_finish=" << init_finish);
  }

  prev_frame = cur_frame;

  // 更新并发布里程计状态
  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame &keyframe, const cv::Mat &cur_img,
                                          vector<cv::Point3f> &key_pts_3d,
                                          vector<cv::Point2f> &cur_pts_2d)
{
  if (keyframe.uv.empty() || keyframe.img.empty() || cur_img.empty())
    return false;

  if (keyframe.img.size() != cur_img.size() || keyframe.img.type() != CV_8UC1 || cur_img.type() != CV_8UC1)
    return false;

  vector<uchar> status;
  vector<float> err;
  cv::Mat prev_img = keyframe.img.isContinuous() ? keyframe.img : keyframe.img.clone();
  cv::Mat next_img = cur_img.isContinuous() ? cur_img : cur_img.clone();
  // 使用金字塔LK光流法在两帧之间进行特征点跟踪
  try
  {
    cv::calcOpticalFlowPyrLK(prev_img, next_img, keyframe.uv, cur_pts_2d, status, err, cv::Size(21, 21), 3);
  }
  catch (const cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(1.0, "trackFeatureBetweenFrames optical flow failed: %s", e.what());
    return false;
  }

  // 剔除超出图像边界的点
  for (int i = 0; i < (int)status.size(); i++)
  {
    if (status[i] && !inBorder(cur_pts_2d[i], cur_img.rows, cur_img.cols))
      status[i] = 0;
  }

  key_pts_3d = keyframe.xyz;
  // 保持3D点和2D点的一一对应关系
  reduceVector<cv::Point3f>(key_pts_3d, status);
  reduceVector<cv::Point2f>(cur_pts_2d, status);

  return cur_pts_2d.size() >= 10;
}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f> &key_pts_3d,
                                       vector<cv::Point2f> &cur_pts_2d, Matrix3d &R, Vector3d &t)
{
  if (key_pts_3d.size() < 10 || key_pts_3d.size() != cur_pts_2d.size())
    return false;

  cv::Mat r_vec, t_vec, inliers;
  cv::Mat K = cv::Mat::eye(3, 3, CV_64F); // Using normalized coordinates after undistortion
  
  // The input cur_pts_2d should be undistorted and in normalized coordinates.
  // Actually, I'll undistort them inside this function if they're not already.
  // Wait, let's assume they are already undistorted and in normalized coordinates.
  // If not, I'll handle that in the caller.
  
  // solvePnPRansac expects 2D points in pixel coordinates with camera matrix, 
  // or in normalized coordinates with K = I.
  
  vector<cv::Point2f> cur_pts_2d_un = undistortedPts(cur_pts_2d, m_camera[0]);

  cv::solvePnPRansac(key_pts_3d, cur_pts_2d_un, K, cv::Mat(), r_vec, t_vec, false, 100, 0.02, 0.99, inliers);

  if (inliers.rows < 10)
    return false;

  cv::Mat R_cv;
  cv::Rodrigues(r_vec, R_cv);
  cv::cv2eigen(R_cv, R);
  cv::cv2eigen(t_vec, t);

  vector<uchar> status(cur_pts_2d.size(), 0);
  for (int i = 0; i < inliers.rows; i++)
    status[inliers.at<int>(i)] = 1;

  reduceVector<cv::Point3f>(key_pts_3d, status);
  reduceVector<cv::Point2f>(cur_pts_2d, status);

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat &img, vector<cv::Point2f> &uv)
{
  int n_max_cnt = MAX_CNT - uv.size();
  if (n_max_cnt > 0)
  {
    cv::Mat mask;
    mask = cv::Mat(img.rows, img.cols, CV_8UC1, cv::Scalar(255));
    for (auto &pt : uv)
    {
      if (inBorder(pt, img.rows, img.cols))
      {
        cv::circle(mask, pt, MIN_DIST, 0, -1);
      }
    }

    vector<cv::Point2f> n_pts;
    cv::goodFeaturesToTrack(img, n_pts, n_max_cnt, 0.01, MIN_DIST, mask);
    uv.insert(uv.end(), n_pts.begin(), n_pts.end());
  }
}

bool Estimator::trackFeatureLeftRight(const cv::Mat &_img, const cv::Mat &_img1,
                                      vector<cv::Point2f> &left_pts, vector<cv::Point2f> &right_pts)
{
  if (left_pts.empty() || _img.empty() || _img1.empty())
    return false;

  if (_img.size() != _img1.size())
  {
    ROS_ERROR_THROTTLE(1.0, "trackFeatureLeftRight image size mismatch: left=%dx%d right=%dx%d",
                       _img.cols, _img.rows, _img1.cols, _img1.rows);
    return false;
  }

  if (_img.type() != CV_8UC1 || _img1.type() != CV_8UC1)
  {
    ROS_ERROR_THROTTLE(1.0, "trackFeatureLeftRight expects mono8 images: left_type=%d right_type=%d",
                       _img.type(), _img1.type());
    return false;
  }

  vector<uchar> status;
  vector<float> err;
  cv::Mat left_img = _img.isContinuous() ? _img : _img.clone();
  cv::Mat right_img = _img1.isContinuous() ? _img1 : _img1.clone();
  try
  {
    cv::calcOpticalFlowPyrLK(left_img, right_img, left_pts, right_pts, status, err, cv::Size(21, 21), 3);
  }
  catch (const cv::Exception &e)
  {
    ROS_ERROR_THROTTLE(1.0, "trackFeatureLeftRight optical flow failed: %s", e.what());
    return false;
  }

  if (FLOW_BACK)
  {
    vector<cv::Point2f> reverse_pts;
    vector<uchar> reverse_status;
    try
    {
      cv::calcOpticalFlowPyrLK(right_img, left_img, right_pts, reverse_pts, reverse_status, err, cv::Size(21, 21), 3);
    }
    catch (const cv::Exception &e)
    {
      ROS_ERROR_THROTTLE(1.0, "trackFeatureLeftRight reverse optical flow failed: %s", e.what());
      return false;
    }
    for (int i = 0; i < (int)status.size(); i++)
    {
      if (status[i] && reverse_status[i] && distance(left_pts[i], reverse_pts[i]) > 0.5)
        status[i] = 0;
    }
  }

  for (int i = 0; i < (int)status.size(); i++)
  {
    if (status[i] && !inBorder(right_pts[i], _img1.rows, _img1.cols))
      status[i] = 0;
  }

  reduceVector<cv::Point2f>(left_pts, status);
  reduceVector<cv::Point2f>(right_pts, status);

  return left_pts.size() >= 10;
}

void Estimator::generate3dPoints(const vector<cv::Point2f> &left_pts,
                                 const vector<cv::Point2f> &right_pts,
                                 vector<cv::Point3f> &cur_pts_3d,
                                 vector<cv::Point2f> &cur_pts_2d)
{

  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0;
  P2.block(0, 0, 3, 3) = (Tlr.block(0, 0, 3, 3).transpose());
  P2.block(0, 3, 3, 1) = -P2.block(0, 0, 3, 3) * Tlr.block(0, 3, 3, 1);

  vector<uchar> status;

  vector<cv::Point2f> left_pts_un = undistortedPts(left_pts, m_camera[0]);
  vector<cv::Point2f> right_pts_un = undistortedPts(right_pts, m_camera[1]);
  if (left_pts_un.size() != left_pts.size() || right_pts_un.size() != right_pts.size())
  {
    ROS_WARN_THROTTLE(1.0, "Stereo VO failed to undistort stereo feature points.");
    cur_pts_2d.clear();
    return;
  }

  for (unsigned int i = 0; i < left_pts.size(); ++i)
  {
    Vector2d pl(left_pts_un[i].x, left_pts_un[i].y);
    Vector2d pr(right_pts_un[i].x, right_pts_un[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0)
    {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    }
    else
    {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}

bool Estimator::inBorder(const cv::Point2f &pt, const int &row, const int &col)
{
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
         img_y < row - BORDER_SIZE;
}

double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2)
{
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}

template <typename Derived>
void Estimator::reduceVector(vector<Derived> &v, vector<uchar> status)
{
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i])
      v[j++] = v[i];
  v.resize(j);
}

void Estimator::updateLatestStates(frame &latest_frame)
{
  latest_time = latest_frame.frame_time;
  rel_key_time = key_frame.frame_time;
  latest_pointcloud = latest_frame.xyz;

  // w_T_b = w_T_c * c_T_b = w_T_c * (b_T_c)^-1
  // T_bc = [ric[0], tic[0]; 0, 1]
  // T_bc^-1 = [ric[0]^T, -ric[0]^T * tic[0]; 0, 1]
  Matrix3d w_R_b = latest_frame.w_R_c * ric[0].transpose();
  Vector3d w_t_b = latest_frame.w_R_c * (-ric[0].transpose() * tic[0]) + latest_frame.w_t_c;

  latest_P = w_t_b;
  latest_Q = Quaterniond(w_R_b);

  // Relative pose between key body frame and current body frame
  // kb_T_cb = T_bc * kc_T_cc * T_bc^-1
  // kc_T_cc = [c_R_k^T, -c_R_k^T * c_t_k; 0, 1]
  Matrix3d R_rel = c_R_k.transpose();
  Vector3d t_rel = -c_R_k.transpose() * c_t_k;

  Matrix3d kb_R_cb = ric[0] * R_rel * ric[0].transpose();
  Vector3d kb_t_cb = ric[0] * (R_rel * (-ric[0].transpose() * tic[0]) + t_rel) + tic[0];

  latest_rel_P = kb_t_cb;
  latest_rel_Q = Quaterniond(kb_R_cb);

  ROS_INFO_STREAM_THROTTLE(1.0, "[stereo_vo][state] t=" << latest_time.toSec()
                           << " key=" << rel_key_time.toSec()
                           << " w_t_b=" << latest_P.transpose()
                           << " rel_t=" << latest_rel_P.transpose()
                           << " rel_q_wxyz=" << latest_rel_Q.w() << " " << latest_rel_Q.x()
                           << " " << latest_rel_Q.y() << " " << latest_rel_Q.z()
                           << " pointcloud=" << latest_pointcloud.size()
                           << " fail_cnt=" << fail_cnt);
}

void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
                                 Eigen::Vector2d &point0, Eigen::Vector2d &point1,
                                 Eigen::Vector3d &point_3d)
{
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}

double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d)
{
  Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
  Vector3d pt2 = R * pt1 + t;
  pt2 = pt2 / pt2[2];
  return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}

vector<cv::Point2f> Estimator::undistortedPts(const vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
  vector<cv::Point2f> un_pts;
  if (!cam)
  {
    ROS_ERROR_THROTTLE(1.0, "Cannot undistort points because camera model is null.");
    return un_pts;
  }
  for (unsigned int i = 0; i < pts.size(); i++)
  {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}
