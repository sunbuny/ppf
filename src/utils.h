//
// Created by sun on 18-3-5.
//

#ifndef PPF3DETECTOR_UTILS_H
#define PPF3DETECTOR_UTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <algorithm>
#include <map>
#include "MurmurHash3.h"

struct Pose3D {
    Eigen::Affine3d pose;
    int votes;
};

struct Cluster{
    std::vector<Pose3D> poses;
    int accu_votes;
};

bool compare_x(Eigen::Vector3d& i, Eigen::Vector3d& j);
bool compare_y(Eigen::Vector3d& i, Eigen::Vector3d& j);
bool compare_z(Eigen::Vector3d& i, Eigen::Vector3d& j);

void loadPLY(const std::string filename,  std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nor);

void savePLY(const std::string filename,  std::vector<Eigen::Vector3d>& pts);

void savePLY(const std::string filename,  std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nos);

void samplePCPoisson(std::vector<Eigen::Vector3d> &pts,std::vector<Eigen::Vector3d> &nos, double sample_step, std::vector<Eigen::Vector3d> &samplePC,std::vector<Eigen::Vector3d> &samplePCNor);

void computePPF(Eigen::Vector3d& pt1, Eigen::Vector3d& nor1, Eigen::Vector3d& pt2, Eigen::Vector3d& nor2,std::vector<double>& f);

uint32_t hashPPF(std::vector<double> f, double angle_step, double distance_step);

double computeAlpha(Eigen::Vector3d pt1,Eigen::Vector3d nor1,Eigen::Vector3d pt2,Eigen::Vector3d nor2);

Eigen::Affine3d transformRT(Eigen::Vector3d pt,Eigen::Vector3d nor);

double rangeMax(std::vector<Eigen::Vector3d> &pts, int dim);

double rangeMin(std::vector<Eigen::Vector3d> &pts, int dim);

bool checkDistance(std::vector<Eigen::Vector3d> &pts,std::vector<int>& neigh, int k, double rs);

std::vector<int> findTheMaxIndexsInAccu(std::vector<int>& accumulator, double rate);

std::vector<int> closeThanModelDiameter(Eigen::Vector3d& pt, std::vector<Eigen::Vector3d>& sampledPC,double diameter);

void trainModel(std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nor, std::vector<Eigen::Vector3d>& sampledPC, std::vector<Eigen::Vector3d>& sampledPCNor,double& angle_step, double& distance_step,int& model_num,std::map<int,std::vector<std::vector<double>>>& hash_table);

void match(std::vector<Eigen::Vector3d>& pts_s, std::vector<Eigen::Vector3d>& nor_s,
           std::vector<Eigen::Vector3d>& pts_m, std::vector<Eigen::Vector3d>& nor_m,
           std::map<int,std::vector<std::vector<double>>>& hash_table,
           double model_distance_step,double model_angle_step,
           int model_sample_num, std::vector<Pose3D>& poses_list);

void sortPoses(std::vector<Pose3D>& poses);

bool comparePoses(Pose3D& pose1, Pose3D& pose2, double distance_limitation, double angle_limitation);

void clusterPoses(std::vector<Pose3D>& poses, std::vector<Cluster>& clusters,double distance_limitation, double angle_limitation);

void averageClusters(const std::vector<Cluster>& clusters, std::vector<Pose3D>& out_poses);

Pose3D averagePoses(std::vector<Pose3D>& in_poses);

#endif //PPF3DETECTOR_UTILS_H
