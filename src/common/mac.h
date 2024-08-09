#ifndef SLAM_IN_AUTO_DRIVING_MAC_H
#define SLAM_IN_AUTO_DRIVING_MAC_H

#include "common/eigen_types.h"
#include "common/message_def.h"
#include <Eigen/Dense>
#include <Eigen/Core>

namespace sad {
/// A data struct contains the output of MAC algorithm
struct MAC {
    MAC() = default;
    MAC(double unix_time, const double transformation[16]){
        unix_time_ = unix_time;
        // Extract the rotation matrix (first 9 elements)
        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++)
            T_(i, j) = transformation[i * 4 + j];
        }
        R_ = T_.block<3, 3>(0, 0);
        translation_ = T_.block<3, 1>(0, 3);
    }

    double unix_time_ = 0;  // unix system time
    Vec3d translation_ = Vec3d::Zero();  // translation vector in transformation matrix
    Eigen::Matrix4d T_ = Eigen::Matrix4d::Zero();   // transformation matrix converting camera to world
    Eigen::Matrix3d R_ = Eigen::Matrix3d::Zero();   // the rotation matrix in transforamtion matrix
};

}

#endif  // SLAM_IN_AUTO_DRIVING_MAC_H