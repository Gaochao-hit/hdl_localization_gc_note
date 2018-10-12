#ifndef POSE_SYSTEM_HPP
#define POSE_SYSTEM_HPP

#include <kkl/alg/unscented_kalman_filter.hpp>

namespace hdl_localization {

/**
 * @brief Definition of system to be estimated by ukf
 * @note state = [px, py, pz, vx, vy, vz, qw, qx, qy, qz, acc_bias_x, acc_bias_y, acc_bias_z, gyro_bias_x, gyro_bias_y, gyro_bias_z]
 */
class PoseSystem {
public:
  typedef float T;
  typedef Eigen::Matrix<T, 3, 1> Vector3t;
  typedef Eigen::Matrix<T, 4, 4> Matrix4t;
  typedef Eigen::Matrix<T, Eigen::Dynamic, 1> VectorXt;
  typedef Eigen::Quaternion<T> Quaterniont;
public:
  PoseSystem() {
    dt = 0.01;
  }

  // system equation   //todo 系统状态转移方程
  VectorXt f(const VectorXt& state, const VectorXt& control) const {
    VectorXt next_state(16);//16个状态

    Vector3t pt = state.middleRows(0, 3);//平移
    Vector3t vt = state.middleRows(3, 3);//速度
    Quaterniont qt(state[6], state[7], state[8], state[9]);//旋转
    qt.normalize();

    Vector3t acc_bias = state.middleRows(10, 3);//加速度计的bias
    Vector3t gyro_bias = state.middleRows(13, 3);//陀螺仪的bias

    Vector3t raw_acc = control.middleRows(0, 3);//输入控制量：加速度计的值
    Vector3t raw_gyro = control.middleRows(3, 3);//输入控制量：陀螺仪的值

    // position
    next_state.middleRows(0, 3) = pt + vt * dt;					//匀速运动模型

    // velocity
    Vector3t g(0.0f, 0.0f, -9.80665f);
    Vector3t acc_ = raw_acc - acc_bias;//去除bias的值
    Vector3t acc = qt * acc_;
    next_state.middleRows(3, 3) = vt; // + (acc - g) * dt;		// acceleration didn't contribute to accuracy due to large noise   //todo 加速度的测量值没有考虑

    // orientation
    Vector3t gyro = raw_gyro - gyro_bias;
    Quaterniont dq(1, gyro[0] * dt / 2, gyro[1] * dt / 2, gyro[2] * dt / 2);
    dq.normalize();
    Quaterniont qt_ = (qt * dq).normalized();//计算新的陀螺仪姿态
    next_state.middleRows(6, 4) << qt_.w(), qt_.x(), qt_.y(), qt_.z();

    next_state.middleRows(10, 3) = state.middleRows(10, 3);		// constant bias on acceleration  //todo 未实时更新，认为恒定
    next_state.middleRows(13, 3) = state.middleRows(13, 3);		// constant bias on angular velocity

    return next_state;
  }

  // observation equation      //todo 观测方程
  VectorXt h(const VectorXt& state) const {
    VectorXt observation(7);
    observation.middleRows(0, 3) = state.middleRows(0, 3);//将状态的位置和姿态作为观测值
    observation.middleRows(3, 4) = state.middleRows(6, 4).normalized();

    return observation;
  }

  double dt;
};

}

#endif // POSE_SYSTEM_HPP
