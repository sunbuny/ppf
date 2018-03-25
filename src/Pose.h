//
// Created by sun on 18-3-6.
//

#ifndef PPF3DETECTOR_POSE_H
#define PPF3DETECTOR_POSE_H

#include <Eigen/Dense>

class Pose {
private:
    double alpha_;
    int model_index_;
    double num_votes_;

    Eigen::Affine3d pose_;
    std::vector<int> voters_;
    Eigen::Vector3d omega_; //角轴表示的轴
    double angle_; //角轴表示的角
    Eigen::Quaterniond q_;
public:
    Pose(double alpha, int model_index, double num_votes):
            alpha_(alpha),model_index_(model_index),num_votes_(num_votes)
    {

    }

    void updatePose(Eigen::Affine3d new_pose);

    void updatePoseQuat(Eigen::Quaterniond new_q);

    void updatePoseT(Eigen::Vector3d t);

    void addVoter(std::vector<int> voter);

    void updateScore(double new_score);
};


#endif //PPF3DETECTOR_POSE_H
