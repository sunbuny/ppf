//
// Created by sun on 18-3-5.
//

#ifndef PPF3DETECTOR_PPF3DDETECTOR_H
#define PPF3DETECTOR_PPF3DDETECTOR_H
#include <Eigen/Dense>
#include <map>
#include <math.h>

#include "Pose.h"

class PPF3DDetector {
private:


public:
    PPF3DDetector(double sampling_relative,double distance_relative, int angle_bins):
            sampling_relative_(sampling_relative),distance_relative_(distance_relative),angle_bins_(angle_bins)
    {
        angle_step_ = (360/angle_bins_)*M_PI/180;
    }

    void trainModel(std::vector<Eigen::Vector3d>& pc);


};


#endif //PPF3DETECTOR_PPF3DDETECTOR_H
