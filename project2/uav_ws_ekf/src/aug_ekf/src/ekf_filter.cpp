#include <ekf_filter.h>
#include <algorithm>
#include <cmath>

namespace ekf_imu_vision {

namespace {

double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

void normalizeAngles(Vec21& x) {
  for (int i = 3; i < 6; ++i) x(i) = wrapAngle(x(i));
  for (int i = 18; i < 21; ++i) x(i) = wrapAngle(x(i));
}

Vec3 quatToEuler(const geometry_msgs::Quaternion& q_msg) {
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() < 1e-12) q.setIdentity();
  q.normalize();
  Mat3x3 R = q.toRotationMatrix();
  const double phi = asin(std::max(-1.0, std::min(1.0, R(2, 1))));
  const double theta = atan2(-R(2, 0), R(2, 2));
  const double psi = atan2(-R(0, 1), R(1, 1));
  return Vec3(wrapAngle(phi), wrapAngle(theta), wrapAngle(psi));
}

int typePriority(int type) {
  if (type == imu) return 0;
  if (type == keyframe) return 1;
  if (type == vo) return 2;
  if (type == height_range) return 3;
  return 4;
}

bool sameStamp(const ros::Time& a, const ros::Time& b) {
  return std::fabs((a - b).toSec()) < 1e-7;
}

const char* stateTypeName(int type) {
  if (type == imu) return "imu";
  if (type == vo) return "vo";
  if (type == keyframe) return "keyframe";
  if (type == height_range) return "range";
  return "unknown";
}

}  // namespace

EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}

void EKFImuVision::init(ros::NodeHandle& nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt2_.setZero();
  Rt_range_.setZero();

  M_a_.setZero();
  M_r_.setZero();
  M_a_.block<15, 15>(0, 0).setIdentity();
  M_a_.block<6, 6>(15, 0).setIdentity();
  M_r_.block<15, 15>(0, 0).setIdentity();

  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }
  node_.param("aug_ekf/enable_range", enable_range_, true);
  node_.param("aug_ekf/range_var", Rt_range_(0, 0), 0.01);
  node_.param("aug_ekf/range_min", range_min_, 0.0);
  node_.param("aug_ekf/range_max", range_max_, 0.0);
  node_.param("aug_ekf/range_gate", range_gate_, 0.8);
  if (Rt_range_(0, 0) <= 0.0) {
    ROS_WARN_STREAM("Augmented EKF: invalid range_var=" << Rt_range_(0, 0)
                    << ", using 0.01.");
    Rt_range_(0, 0) = 0.01;
  }

  init_ = false;
  current_imu_ut.setZero();
  path_.poses.clear();

  for (int i = 0; i < state_type_count; i++) latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ = node_.subscribe<sensor_msgs::Imu>("imu", 100, &EKFImuVision::imuCallback, this);
  stereo_sub_ = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  if (enable_range_) {
    range_sub_ = node_.subscribe<sensor_msgs::Range>("range", 20, &EKFImuVision::rangeCallback, this,
                                                     ros::TransportHints().tcpNoDelay());
  }
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("/ekf/ekf_odom", 10);
  path_pub_ = node_.advertise<nav_msgs::Path>("/ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {
  if (!msg->key_stamp.isZero()) {
    bool known_keyframe = false;
    for (deque<AugState>::const_iterator it = aug_state_hist_.begin(); it != aug_state_hist_.end(); ++it) {
      if (it->type == keyframe && sameStamp(it->time_stamp, msg->key_stamp)) {
        known_keyframe = true;
        break;
      }
    }

    ros::Time active_keyframe = aug_state_hist_.empty() ? ros::Time(0) : aug_state_hist_.back().key_frame_time_stamp;
    if (!known_keyframe && !sameStamp(active_keyframe, msg->key_stamp)) {
      AugState key_state;
      key_state.time_stamp = msg->key_stamp;
      key_state.key_frame_time_stamp = msg->key_stamp;
      key_state.type = keyframe;
      key_state.mean.setZero();
      key_state.covariance.setZero();
      key_state.ut.setZero();
      ROS_INFO_STREAM_THROTTLE(0.5, "[aug_ekf][keyframe_cb] request keyframe copy at t="
                               << msg->key_stamp.toSec()
                               << " from vo t=" << msg->header.stamp.toSec());
      processNewState(key_state, true);
    }
  }

  AugState new_state;
  new_state.time_stamp = msg->header.stamp;
  new_state.key_frame_time_stamp = msg->key_stamp;
  new_state.type = vo;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();
  new_state.ut(0) = msg->relative_pose.position.x;
  new_state.ut(1) = msg->relative_pose.position.y;
  new_state.ut(2) = msg->relative_pose.position.z;
  new_state.ut.segment<3>(3) = quatToEuler(msg->relative_pose.orientation);

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][vo_cb] t=" << new_state.time_stamp.toSec()
                           << " key=" << new_state.key_frame_time_stamp.toSec()
                           << " rel_pos=" << new_state.ut.segment<3>(0).transpose()
                           << " rel_euler=" << new_state.ut.segment<3>(3).transpose());

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::rangeCallback(const sensor_msgs::RangeConstPtr& msg) {
  if (!enable_range_) {
    return;
  }

  const double range = msg->range;
  if (!std::isfinite(range) || range < 0.0) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[aug_ekf][range_cb] drop invalid range=" << range);
    return;
  }

  double min_range = range_min_;
  if (msg->min_range > 0.0) min_range = std::max(min_range, static_cast<double>(msg->min_range));
  if (range < min_range) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[aug_ekf][range_cb] drop below min range=" << range
                             << " min=" << min_range);
    return;
  }

  double max_range = range_max_;
  if (msg->max_range > 0.0) {
    max_range = max_range > 0.0 ? std::min(max_range, static_cast<double>(msg->max_range))
                                : static_cast<double>(msg->max_range);
  }
  if (max_range > 0.0 && range > max_range) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[aug_ekf][range_cb] drop above max range=" << range
                             << " max=" << max_range);
    return;
  }

  AugState new_state;
  new_state.time_stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
  new_state.key_frame_time_stamp = ros::Time(0);
  new_state.type = height_range;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();
  new_state.ut(0) = range;

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][range_cb] t=" << new_state.time_stamp.toSec()
                           << " range=" << range);

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {
  AugState new_state;
  new_state.time_stamp = imu_msg->header.stamp;
  new_state.key_frame_time_stamp = ros::Time(0);
  new_state.type = imu;
  new_state.mean.setZero();
  new_state.covariance.setZero();
  new_state.ut.setZero();
  new_state.ut(0) = imu_msg->angular_velocity.x;
  new_state.ut(1) = imu_msg->angular_velocity.y;
  new_state.ut(2) = imu_msg->angular_velocity.z;
  new_state.ut(3) = imu_msg->linear_acceleration.x;
  new_state.ut(4) = imu_msg->linear_acceleration.y;
  new_state.ut(5) = imu_msg->linear_acceleration.z;
  current_imu_ut = new_state.ut;

  ROS_INFO_STREAM_THROTTLE(2.0, "[aug_ekf][imu_cb] t=" << new_state.time_stamp.toSec()
                           << " gyro=" << new_state.ut.segment<3>(0).transpose()
                           << " acc=" << new_state.ut.segment<3>(3).transpose());

  if (!processNewState(new_state, false)) {
    return;
  }
}

void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {
  const double dt = std::max(0.0, (cur_state.time_stamp - prev_state.time_stamp).toSec());
  cur_state.mean = prev_state.mean;
  cur_state.covariance = prev_state.covariance;
  cur_state.key_frame_time_stamp = prev_state.key_frame_time_stamp;

  if (dt <= 1e-9) {
    return;
  }

  Vec12 nt = Vec12::Zero();
  Vec15 x15 = prev_state.mean.segment<15>(0);
  Mat15x15 Fx = jacobiFx(x15, ut, nt);
  Mat15x12 Fn = jacobiFn(x15, ut, nt);

  Mat21x21 F = Mat21x21::Identity();
  F.block<15, 15>(0, 0) += Fx * dt;

  Eigen::Matrix<double, 21, 12> N = Eigen::Matrix<double, 21, 12>::Zero();
  N.block<15, 12>(0, 0) = Fn * dt;

  cur_state.mean.segment<15>(0) = x15 + modelF(x15, ut, nt) * dt;
  normalizeAngles(cur_state.mean);
  cur_state.covariance = F * prev_state.covariance * F.transpose() + N * Qt_ * N.transpose();
  cur_state.covariance = 0.5 * (cur_state.covariance + cur_state.covariance.transpose());

  ROS_INFO_STREAM_THROTTLE(2.0, "[aug_ekf][predict] t=" << cur_state.time_stamp.toSec()
                           << " dt=" << dt
                           << " pos=" << cur_state.mean.segment<3>(0).transpose()
                           << " vel=" << cur_state.mean.segment<3>(6).transpose());
}

void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {
  (void)prev_state;

  if (cur_state.key_frame_time_stamp.isZero()) {
    return;
  }

  Vec6 residual = cur_state.ut - modelG2(cur_state.mean, Vec6::Zero());
  for (int i = 3; i < 6; ++i) residual(i) = wrapAngle(residual(i));

  Mat6x21 H = jacobiG2x(cur_state.mean, Vec6::Zero());
  Mat6x6 S = H * cur_state.covariance * H.transpose() + Rt2_;
  Mat21x6 K = cur_state.covariance * H.transpose() * S.inverse();
  Mat21x21 I = Mat21x21::Identity();

  cur_state.mean += K * residual;
  normalizeAngles(cur_state.mean);
  cur_state.covariance = (I - K * H) * cur_state.covariance * (I - K * H).transpose() + K * Rt2_ * K.transpose();
  cur_state.covariance = 0.5 * (cur_state.covariance + cur_state.covariance.transpose());

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][vo_update] t=" << cur_state.time_stamp.toSec()
                           << " key=" << cur_state.key_frame_time_stamp.toSec()
                           << " residual=" << residual.transpose()
                           << " residual_norm=" << residual.norm()
                           << " state_pos=" << cur_state.mean.segment<3>(0).transpose()
                           << " key_pos=" << cur_state.mean.segment<3>(15).transpose());
}

void EKFImuVision::updateRange(AugState& cur_state) {
  const double measured_z = cur_state.ut(0);
  const double predicted_z = cur_state.mean(2);
  const double residual = measured_z - predicted_z;

  if (range_gate_ > 0.0 && std::fabs(residual) > range_gate_) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[aug_ekf][range_update] reject residual=" << residual
                             << " measured_z=" << measured_z
                             << " state_z=" << predicted_z
                             << " gate=" << range_gate_);
    return;
  }

  Eigen::Matrix<double, 1, 21> H = Eigen::Matrix<double, 1, 21>::Zero();
  H(0, 2) = 1.0;

  Mat1x1 S = H * cur_state.covariance * H.transpose() + Rt_range_;
  if (S(0, 0) <= 1e-12 || !std::isfinite(S(0, 0))) {
    ROS_WARN_STREAM_THROTTLE(1.0, "[aug_ekf][range_update] skip invalid innovation variance="
                             << S(0, 0));
    return;
  }

  Eigen::Matrix<double, 21, 1> K = cur_state.covariance * H.transpose() / S(0, 0);
  Mat21x21 I = Mat21x21::Identity();

  cur_state.mean += K * residual;
  normalizeAngles(cur_state.mean);
  cur_state.covariance = (I - K * H) * cur_state.covariance * (I - K * H).transpose() +
                         K * Rt_range_ * K.transpose();
  cur_state.covariance = 0.5 * (cur_state.covariance + cur_state.covariance.transpose());

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][range_update] t=" << cur_state.time_stamp.toSec()
                           << " measured_z=" << measured_z
                           << " residual=" << residual
                           << " state_z=" << cur_state.mean(2));
}

void EKFImuVision::changeAugmentedState(AugState& state) {
  Vec15 x15 = M_r_ * state.mean;
  Mat15x15 P15 = M_r_ * state.covariance * M_r_.transpose();
  state.mean = M_a_ * x15;
  state.covariance = M_a_ * P15 * M_a_.transpose();
  state.key_frame_time_stamp = state.time_stamp;
  normalizeAngles(state.mean);

  ROS_INFO_STREAM_THROTTLE(0.5, "[aug_ekf][keyframe_change] t=" << state.time_stamp.toSec()
                           << " copied_pos=" << state.mean.segment<3>(15).transpose()
                           << " copied_euler=" << state.mean.segment<3>(18).transpose());
}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {
  if (new_state.time_stamp.isZero()) {
    return false;
  }
  if (init_ && !aug_state_hist_.empty() && new_state.time_stamp < aug_state_hist_.front().time_stamp) {
    ROS_WARN_THROTTLE(2.0, "Augmented EKF: dropping a measurement older than retained history.");
    return false;
  }

  (void)change_keyframe;
  deque<AugState>::iterator new_input_it = insertNewState(new_state);
  if (new_input_it == aug_state_hist_.end()) return false;

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][queue] inserted type=" << stateTypeName(new_state.type)
                           << " t=" << new_state.time_stamp.toSec()
                           << " size=" << aug_state_hist_.size()
                           << " insert_idx=" << std::distance(aug_state_hist_.begin(), new_input_it)
                           << " initialized=" << init_);

  bool during_init = false;
  if (!init_) {
    if (!initFilter()) {
      removeOldState();
      return false;
    }
    during_init = true;
    new_input_it = aug_state_hist_.begin();
  }

  repropagate(new_input_it, during_init);
  removeOldState();
  publishFusedOdom();
  return true;
}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state) {
  deque<AugState>::iterator insert_it = aug_state_hist_.begin();
  for (; insert_it != aug_state_hist_.end(); ++insert_it) {
    if (new_state.time_stamp < insert_it->time_stamp) break;
    if (sameStamp(new_state.time_stamp, insert_it->time_stamp) &&
        typePriority(new_state.type) < typePriority(insert_it->type)) {
      break;
    }
  }

  unsigned int idx = static_cast<unsigned int>(std::distance(aug_state_hist_.begin(), insert_it));
  aug_state_hist_.insert(insert_it, new_state);

  for (int i = 0; i < state_type_count; ++i) latest_idx[i] = 0;
  for (unsigned int i = 0; i < aug_state_hist_.size(); ++i) {
    latest_idx[aug_state_hist_[i].type] = i;
  }

  return aug_state_hist_.begin() + idx;
}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it, bool& init) {
  (void)init;
  if (aug_state_hist_.empty() || new_input_it == aug_state_hist_.end()) return;

  unsigned int start_idx = static_cast<unsigned int>(std::distance(aug_state_hist_.begin(), new_input_it));
  if (start_idx == 0) start_idx = 1;

  Vec6 last_imu_ut = current_imu_ut;
  bool have_imu = false;
  for (unsigned int i = 0; i < start_idx && i < aug_state_hist_.size(); ++i) {
    if (aug_state_hist_[i].type == imu) {
      last_imu_ut = aug_state_hist_[i].ut;
      have_imu = true;
    }
  }
  if (!have_imu) last_imu_ut.setZero();

  for (unsigned int i = start_idx; i < aug_state_hist_.size(); ++i) {
    AugState& cur_state = aug_state_hist_[i];
    AugState& prev_state = aug_state_hist_[i - 1];

    if (cur_state.type == imu) {
      predictIMU(cur_state, prev_state, cur_state.ut);
      last_imu_ut = cur_state.ut;
    } else if (cur_state.type == keyframe) {
      predictIMU(cur_state, prev_state, last_imu_ut);
      changeAugmentedState(cur_state);
    } else if (cur_state.type == vo) {
      predictIMU(cur_state, prev_state, last_imu_ut);
      updateVO(cur_state, prev_state);
    } else if (cur_state.type == height_range) {
      predictIMU(cur_state, prev_state, last_imu_ut);
      updateRange(cur_state);
    }
  }

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][repropagate] start_idx=" << start_idx
                           << " end_t=" << aug_state_hist_.back().time_stamp.toSec()
                           << " last_type=" << stateTypeName(aug_state_hist_.back().type)
                           << " queue_size=" << aug_state_hist_.size()
                           << " active_key=" << aug_state_hist_.back().key_frame_time_stamp.toSec());

  for (int i = 0; i < state_type_count; ++i) latest_idx[i] = 0;
  for (unsigned int i = 0; i < aug_state_hist_.size(); ++i) {
    latest_idx[aug_state_hist_[i].type] = i;
  }
}

void EKFImuVision::removeOldState() {
  if (aug_state_hist_.empty()) return;

  const ros::Time key_time = aug_state_hist_.back().key_frame_time_stamp;
  if (!key_time.isZero()) {
    while (aug_state_hist_.size() > 2 && aug_state_hist_[1].time_stamp < key_time) {
      aug_state_hist_.pop_front();
    }
  }

  const size_t max_hist_size = 1000;
  while (aug_state_hist_.size() > max_hist_size) {
    aug_state_hist_.pop_front();
  }

  ROS_INFO_STREAM_THROTTLE(5.0, "[aug_ekf][history] size=" << aug_state_hist_.size()
                           << " front_t=" << aug_state_hist_.front().time_stamp.toSec()
                           << " back_t=" << aug_state_hist_.back().time_stamp.toSec()
                           << " active_key=" << aug_state_hist_.back().key_frame_time_stamp.toSec());

  for (int i = 0; i < state_type_count; ++i) latest_idx[i] = 0;
  for (unsigned int i = 0; i < aug_state_hist_.size(); ++i) {
    latest_idx[aug_state_hist_[i].type] = i;
  }
}

void EKFImuVision::publishFusedOdom() {
  if (aug_state_hist_.empty()) return;

  AugState last_state = aug_state_hist_.back();

  double phi, theta, psi;
  phi = last_state.mean(3);
  theta = last_state.mean(4);
  psi = last_state.mean(5);

  if (last_state.mean.head(3).norm() > 20) {
    ROS_ERROR_STREAM("error state: " << last_state.mean.head(3).transpose());
    return;
  }

  // using the zxy euler angle
  Eigen::Quaterniond q = Eigen::AngleAxisd(psi, Eigen::Vector3d::UnitZ()) *
                         Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitY());
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.header.stamp = last_state.time_stamp;
  odom.child_frame_id = "base_link";

  odom.pose.pose.position.x = last_state.mean(0);
  odom.pose.pose.position.y = last_state.mean(1);
  odom.pose.pose.position.z = last_state.mean(2);

  odom.pose.pose.orientation.w = q.w();
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();

  odom.twist.twist.linear.x = last_state.mean(6);
  odom.twist.twist.linear.y = last_state.mean(7);
  odom.twist.twist.linear.z = last_state.mean(8);

  fuse_odom_pub_.publish(odom);

  geometry_msgs::PoseStamped path_pose;
  path_.header.frame_id = "world";
  path_.header.stamp = last_state.time_stamp;
  path_pose.header = path_.header;
  path_pose.pose.position.x = last_state.mean(0);
  path_pose.pose.position.y = last_state.mean(1);
  path_pose.pose.position.z = last_state.mean(2);
  path_pose.pose.orientation = odom.pose.pose.orientation;
  if (path_.poses.empty() || path_pose.header.stamp > path_.poses.back().header.stamp) {
    path_.poses.push_back(path_pose);
    const size_t max_path_size = 5000;
    if (path_.poses.size() > max_path_size) {
      path_.poses.erase(path_.poses.begin(), path_.poses.begin() + (path_.poses.size() - max_path_size));
    }
  }
  path_pub_.publish(path_);

  ROS_INFO_STREAM_THROTTLE(1.0, "[aug_ekf][publish] t=" << last_state.time_stamp.toSec()
                           << " pos=" << last_state.mean.segment<3>(0).transpose()
                           << " euler=" << last_state.mean.segment<3>(3).transpose()
                           << " vel=" << last_state.mean.segment<3>(6).transpose()
                           << " path_len=" << path_.poses.size());
}

bool EKFImuVision::initFilter() {
  if (init_) return true;

  if (aug_state_hist_.empty()) {
    ROS_WARN_THROTTLE(2.0, "Augmented EKF: waiting for the first IMU or VO measurement to initialize.");
    return false;
  }

  deque<AugState>::iterator it = aug_state_hist_.begin();
  if (!initFromState(it)) return false;

  init_ = true;
  ROS_INFO_STREAM("[aug_ekf][init] initialized at local origin t=" << it->time_stamp.toSec()
                  << " type=" << stateTypeName(it->type)
                  << " pos=" << it->mean.segment<3>(0).transpose()
                  << " euler=" << it->mean.segment<3>(3).transpose());
  return true;
}

bool EKFImuVision::initFromState(deque<AugState>::iterator start_it) {
  if (start_it == aug_state_hist_.end()) return false;

  if (start_it != aug_state_hist_.begin()) {
    aug_state_hist_.erase(aug_state_hist_.begin(), start_it);
    start_it = aug_state_hist_.begin();
  }

  Vec15 x15 = Vec15::Zero();

  Mat15x15 P15 = Mat15x15::Zero();
  P15.block<3, 3>(0, 0) = Mat3x3::Identity();
  P15.block<3, 3>(3, 3) = Mat3x3::Identity();
  P15.block<3, 3>(6, 6) = Mat3x3::Identity();
  P15.block<3, 3>(9, 9) = 0.1 * Mat3x3::Identity();
  P15.block<3, 3>(12, 12) = 0.1 * Mat3x3::Identity();

  start_it->mean = M_a_ * x15;
  start_it->covariance = M_a_ * P15 * M_a_.transpose();
  start_it->key_frame_time_stamp = (start_it->type == keyframe) ? start_it->time_stamp : ros::Time(0);
  normalizeAngles(start_it->mean);

  ROS_INFO_STREAM("[aug_ekf][init_origin] t=" << start_it->time_stamp.toSec()
                  << " type=" << stateTypeName(start_it->type)
                  << " init_cov_diag=" << start_it->covariance.diagonal().transpose());

  for (int i = 0; i < state_type_count; ++i) latest_idx[i] = 0;
  for (unsigned int i = 0; i < aug_state_hist_.size(); ++i) {
    latest_idx[aug_state_hist_[i].type] = i;
  }

  return true;
}

Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}

}  // namespace ekf_imu_vision
