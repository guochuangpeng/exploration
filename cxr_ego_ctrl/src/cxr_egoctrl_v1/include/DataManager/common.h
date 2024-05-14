#ifndef COMMON_H
#define COMMON_H
#include <Eigen/Eigen>

struct UavState {
  Eigen::Vector3d pos;
  Eigen::Vector3d vel;
  Eigen::Vector3d acc;
  Eigen::Quaterniond q;
  double yaw;
  // 默认构造函数，用于初始化成员变量
  UavState()
      : pos(Eigen::Vector3d::Zero()),  // 通常将位置初始化为零向量
        vel(Eigen::Vector3d::Zero()),  // 速度初始化为零向量
        acc(Eigen::Vector3d::Zero()),  // 加速度初始化为零向量
        q(Eigen::Quaterniond::
              Identity()),  // 四元数初始化为单位四元数（表示没有旋转）
        yaw(0.0)  // 偏航角初始化为0
  {}
};

template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 1> quaternion_to_ypr(
    const Eigen::Quaternion<Scalar_t>& q_) {
  Eigen::Quaternion<Scalar_t> q = q_.normalized();

  Eigen::Matrix<Scalar_t, 3, 1> ypr;
  ypr(2) = atan2(2 * (q.w() * q.x() + q.y() * q.z()),
                 1 - 2 * (q.x() * q.x() + q.y() * q.y()));
  ypr(1) = asin(2 * (q.w() * q.y() - q.z() * q.x()));
  ypr(0) = atan2(2 * (q.w() * q.z() + q.x() * q.y()),
                 1 - 2 * (q.y() * q.y() + q.z() * q.z()));

  return ypr;
}
template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t) {
  Eigen::Matrix<Scalar_t, 3, 3> R;
  R(0, 0) = std::cos(t);
  R(0, 1) = -std::sin(t);
  R(0, 2) = 0.0;
  R(1, 0) = std::sin(t);
  R(1, 1) = std::cos(t);
  R(1, 2) = 0.0;
  R(2, 0) = 0.0;
  R(2, 1) = 0.0;
  R(2, 2) = 1.0;

  return R;
}
template <typename Scalar_t>
Eigen::Quaternion<Scalar_t> yaw_to_quaternion(Scalar_t yaw) {
  return Eigen::Quaternion<Scalar_t>(rotz(yaw));
}


#endif