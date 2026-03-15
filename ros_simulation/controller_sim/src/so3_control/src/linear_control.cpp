/*
 * @Description: 
 * @Version: V1.0.0
 * @Author: lsylsylsylsylsylsy 387836421@qq.com
 * @Date: 2021-04-09 20:45:14
 * @LastEditors: lsylsylsylsylsylsy 387836421@qq.com
 * @LastEditTime: 2026-03-15 16:17:52
 * @FilePath: linear_control.cpp
 * Copyright 2026 Marvin, All Rights Reserved. 
 * 2021-04-09 20:45:14
 */
#include <linear_control.h>
#include <iostream>
#include <ros/ros.h>
#include <algorithm>
#include <cmath>

LinearControl::LinearControl()
  : mass_(0.49)
  , g_(9.81)
{
}

void
LinearControl::setMass(const double mass)
{
  mass_ = mass;
}

void
LinearControl::setGravity(const double g)
{
  g_ = g;
}

void
LinearControl::calculateControl(const Desired_State_t &des,
                        const Odom_Data_t &odom, 
                        const Imu_Data_t &imu,
                        Controller_Output_t &u,
                        Gain gain)
{
    (void)imu;

  const Eigen::Vector3d e3(0.0, 0.0, 1.0);
  const Eigen::Vector3d minus_e3(0.0, 0.0, -1.0);

    const Eigen::Vector3d Kp(gain.Kp0, gain.Kp1, gain.Kp2);
    const Eigen::Vector3d Kv(gain.Kv0, gain.Kv1, gain.Kv2);

    const Eigen::Vector3d pos_err = des.p - odom.p;
    const Eigen::Vector3d vel_err = des.v - odom.v;

    // F = kx*ex + kv*ev - m*g*e3 + m*xddot_d
    const Eigen::Vector3d F_des = Kp.cwiseProduct(pos_err) +
                    Kv.cwiseProduct(vel_err) +
                    mass_ * g_ * e3 +
                    mass_ * des.a;

    // f = F * R * (-e3), where odom.q rotates body to world.
    //const Eigen::Vector3d body_minus_e3_in_world = odom.q * minus_e3;
    u.thrust = F_des.norm();

    const Eigen::Matrix3d R_des_q = des.q.toRotationMatrix();
    // R -> ZYX euler, then map to roll/pitch/yaw.
    const Eigen::Vector3d euler_zyx = R_des_q.eulerAngles(2, 1, 0);
    const double yaw_des = euler_zyx(0);

    const Eigen::Vector3d xc_des(std::cos(yaw_des), std::sin(yaw_des), 0.0);

    Eigen::Vector3d b3d = F_des;
    b3d.normalize();

    Eigen::Vector3d b2d = b3d.cross(xc_des);
    b2d.normalize();

    Eigen::Vector3d b1d = b2d.cross(b3d);
    b1d.normalize();

    Eigen::Matrix3d R_des;
    R_des.col(0) = b1d;
    R_des.col(1) = b2d;
    R_des.col(2) = b3d;

    u.q = Eigen::Quaterniond(R_des);
}
