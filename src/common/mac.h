#ifndef SLAM_IN_AUTO_DRIVING_MAC_H
#define SLAM_IN_AUTO_DRIVING_MAC_H

#include "common/eigen_types.h"
#include "common/message_def.h"

namespace sad {
/// A data struct contains the output of MAC algorithm
struct MAC {
    MAC() = default;
    MAC(double unix_time, const double transformation_array[16]){
        unix_time_ = unix_time;
        Eigen::Matrix4d T_ = Eigen::Matrix4d::Zero();   // transformation matrix converting camera to world
        Eigen::Matrix3d R_ = Eigen::Matrix3d::Zero();   // the rotation matrix in transforamtion matrix

        for(int i = 0; i < 4; i++){
            for(int j = 0; j < 4; j++)
            T_(i, j) = transformation_array[i * 4 + j];
        }
        
        R_ = T_.block<3, 3>(0, 0);
        translation_ = T_.block<3, 1>(0, 3);
        transformation = SE3(R_, translation_);
        rotation = SO3(R_);
    }

    double unix_time_ = 0;  // unix system time
    Vec3d translation_ = Vec3d::Zero();  // translation vector in transformation matrix
    SO3 rotation;  // rotation matrix in transformation matrix
    SE3 transformation;  // transformation matrix converting camera to world
};

}   //namespace sad

using MACPtr = std::shared_ptr<sad::MAC>;

#endif  // SLAM_IN_AUTO_DRIVING_MAC_H