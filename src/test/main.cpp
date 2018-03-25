//
// Created by sun on 18-3-5.
//

#include <iostream>
#include "../utils.h"
//#include "../PPF3DDetector.h"
//static void loadPLY(const std::string filename, std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nos)
//{
//    std::ifstream file(filename.c_str(), std::ifstream::in);
//    if(file.fail() == true)
//    {
//        std::cerr << filename << " cloud not be openned" << std::endl;
//    }
//    char line[1024] = {0};
//
//
//    unsigned long begin = 0;
//    int vertex = 0;
//    int cols = 6;
//
//    while (true)
//    {
//        file.getline(line, sizeof(line));
//        std::string line_str(line);
//        if((begin = line_str.find("element vertex"))!=std::string::npos)
//        {
//            std::stringstream vertex_str(line_str.substr(begin+14));
//            vertex_str >> vertex;
//            std::cout << "element vertex: " << vertex;
//        } else if(line_str.find("property uchar red")!=std::string::npos){
//            cols = 9;
//        } else if(line_str.find("property uchar alpha")!=std::string::npos){
//            cols = 10;
//        } else if(line_str.find("end_header")!=std::string::npos){
//            break;
//        }
//    }
//
//    for(int i=0; i < vertex; i++)
//    {
//        file.getline(line, sizeof(line));
//        std::stringstream line_str(line);
//        Eigen::Vector3d pt,no;
//        line_str >> pt.x() >> pt.y() >> pt.z() >> no.x() >> no.y() >> no.z();
//        pts.push_back(pt);
//        nos.push_back(no);
//    }
//
//}



int main(int argc, char** argv){
    std::string filename("/home/sun/ClionProjects/ppf3Detector/data/mian_T-rex_high.ply");
    std::string scene_name("/home/sun/ClionProjects/ppf3Detector/data/rs1.ply");
    std::string savename("/home/sun/ClionProjects/ppf3Detector/data/save.ply");
    std::vector<Eigen::Vector3d> pts, sampledPC,samplePCNor;
    std::vector<Eigen::Vector3d> nos;
    std::vector<Eigen::Vector3d> pts_scene, nos_scene;
    loadPLY(filename,pts,nos);
    loadPLY(scene_name,pts_scene,nos_scene);

//    std::cout << "Got :" << pts.size() << "vertexs" << std::endl;
//    std::cout << pts.back().x() << " "<< pts.back().y() <<" "<< pts.back().z();
//    std::cout << nos.back().x() << " "<< nos.back().y() <<" "<< nos.back().z();
////    savePLY(savename,pts);
//    std::cout<< std::endl << rangeMax(pts,1) << std::endl;
//    std::cout<< std::endl << rangeMin(pts,1) << std::endl;
//
////    samplePCPoisson(pts,nos,0.04,sampledPC,samplePCNor);
////    savePLY(savename,pts,nos);
//    Eigen::Vector3d pt1(0,0,0);
//    Eigen::Vector3d pt2(0,1,0);
//    Eigen::Vector3d nor1(1,0,0);
//    Eigen::Vector3d nor2(0,0,1);
//    std::vector<double> f;
//
//    computePPF(pt1,nor1,pt2,nor2,f);
//
//
//    std::cout << "hash out : " << hashPPF(f,0.2,10) << std::endl;
//
//    Eigen::Vector3d pt3(0.5,0.5,0.5);
//    Eigen::Vector3d nor3(0.5,0.5,0.5);
//    std::cout << transformRT(pt2,nor1).matrix() << std::endl;

    std::map<int,std::vector<std::vector<double>>> hash_table ;
    double angle_step = 0, distance_step = 0;
    int model_num = 0;
    trainModel(pts,nos,sampledPC,samplePCNor,angle_step,distance_step,model_num,hash_table);
    std::cout <<"hash table size: " <<  hash_table.size() << std::endl;
    std::vector<Pose3D> poses_list;
    match(pts_scene,nos_scene,sampledPC,samplePCNor,hash_table,distance_step,angle_step,model_num,poses_list);
    std::cout << poses_list.size() << std::endl;
    sortPoses(poses_list);
    std::vector<Cluster> clusters;
    clusterPoses(poses_list,clusters,27.3,angle_step);
    std::vector<Pose3D> ans_poses;
    std::cout << "clusters: " << clusters.size() << std::endl;
    averageClusters(clusters,ans_poses);
    sortPoses(ans_poses);
    std::cout << ans_poses[0].pose.rotation() << std::endl;
    std::cout << ans_poses[0].pose.translation() << std::endl;
    std::vector<Eigen::Vector3d> pts_save;

//    Eigen::MatrixXd TT(4,4);
//    TT << 0.986843,0.0105495,0.161341,-15.9453,-0.0349336,-0.960388,0.276469,185.037,0.157867,-0.278468,-0.947383,-1199.47,0,0,0,1;
//    std::cout << TT << std::endl;
    for (int i = 0; i < pts.size(); ++i) {
        Eigen::Vector4d v_tmp;
        v_tmp << pts[i] , 1;
        Eigen::Vector4d pt_after = ans_poses[0].pose*v_tmp;
        pts_save.push_back(pt_after.head(3));
    }
    savePLY(savename,pts_save);
////    for (int i = 0; i < f.size(); ++i) {
////        std::cout <<1 << ": " <<f[i] << std::endl;
////    }
//    sortPoses(poses_list);
//    int i = 0;
//    for (auto pose:poses_list) {
//        std::cout << "Pose" << i << " : "  << pose.votes << std::endl;
//    }

    return 0;
}

//int main(int argc, char** argv)
//{
//        Eigen::Vector3d pt3(0.5,0.5,0.5);
//    Eigen::Vector3d nor3(0.5,0.5,0.5);
//    std::cout << transformRT(pt3,nor3).matrix() << std::endl;
//
//}