//
// Created by sun on 18-3-5.
//

#include "utils.h"


void loadPLY(const std::string filename, std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nos)
{
    std::ifstream file(filename.c_str(), std::ifstream::in);
    if(file.fail() == true)
    {
        std::cerr << filename << " cloud not be openned" << std::endl;
    }
    char line[1024] = {0};


    unsigned long begin = 0;
    int vertex = 0;
    int cols = 6;

    while (true)
    {
        file.getline(line, sizeof(line));
        std::string line_str(line);
        if((begin = line_str.find("element vertex"))!=std::string::npos)
        {
            std::stringstream vertex_str(line_str.substr(begin+14));
            vertex_str >> vertex;
            std::cout << "element vertex: " << vertex<<std::endl;
        } else if(line_str.find("property uchar red")!=std::string::npos){
            cols = 9;
        } else if(line_str.find("property uchar alpha")!=std::string::npos){
            cols = 10;
        } else if(line_str.find("end_header")!=std::string::npos){
            break;
        }
    }

    for(int i=0; i < vertex; i++)
    {
        file.getline(line, sizeof(line));
        std::stringstream line_str(line);
        Eigen::Vector3d pt,no;
        line_str >> pt.x() >> pt.y() >> pt.z() >> no.x() >> no.y() >> no.z();
        pts.push_back(pt);
        nos.push_back(no);
    }
    file.close();

}

void savePLY(const std::string filename, std::vector<Eigen::Vector3d> &pts) {
    std::ofstream file(filename.c_str(),std::ofstream::out);
    if(file.fail() == true)
    {
        std::cerr << filename << " cloud not be openned" << std::endl;
    }
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment VCGLIB generated" << std::endl;
    file << "comment VCGLIB generated" << std::endl;
    file << "element vertex " << pts.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "element face 0" << std::endl;
    file << "property list uchar int vertex_indices" << std::endl;
    file << "end_header" << std::endl;

    for (int i = 0; i < pts.size(); ++i) {
        file <<std::setprecision(6)<< pts[i].x() << " " << pts[i].y() << " "<< pts[i].z() << std::endl;
    }
    file.close();

}

void savePLY(const std::string filename,  std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nos)
{
    std::ofstream file(filename.c_str(),std::ofstream::out);
    if(file.fail() == true)
    {
        std::cerr << filename << " cloud not be openned" << std::endl;
    }
    file << "ply" << std::endl;
    file << "format ascii 1.0" << std::endl;
    file << "comment VCGLIB generated" << std::endl;
    file << "comment VCGLIB generated" << std::endl;
    file << "element vertex " << pts.size() << std::endl;
    file << "property float x" << std::endl;
    file << "property float y" << std::endl;
    file << "property float z" << std::endl;
    file << "property float nx" << std::endl;
    file << "property float ny" << std::endl;
    file << "property float nz" << std::endl;
    file << "element face 0" << std::endl;
    file << "property list uchar int vertex_indices" << std::endl;
    file << "end_header" << std::endl;

    for (int i = 0; i < pts.size(); ++i) {
        file <<std::setprecision(6)<< pts[i].x() << " " << pts[i].y() << " "<< pts[i].z() << " "<< nos[i].x() << " " << nos[i].y() << " "<< nos[i].z() << std::endl;
    }
    file.close();
}

void samplePCPoisson(std::vector<Eigen::Vector3d> &pts,std::vector<Eigen::Vector3d> &nos, double sample_step, std::vector<Eigen::Vector3d> &samplePC,std::vector<Eigen::Vector3d> &samplePCNor) {
    // poisson disc sampling
    std::vector<double> range_x, range_y, range_z;
    // 求包围盒
    range_x.push_back(rangeMin(pts,1));
    range_x.push_back(rangeMax(pts,1));

    range_y.push_back(rangeMin(pts,2));
    range_y.push_back(rangeMax(pts,2));

    range_z.push_back(rangeMin(pts,3));
    range_z.push_back(rangeMax(pts,3));

    double dx = range_x.back() - range_x.front();
    double dy = range_y.back() - range_y.front();
    double dz = range_z.back() - range_z.front();

    double d = sqrt(dx*dx + dy*dy + dz*dz); //包围盒 对角线长度
    double r = d*sample_step; //点点之间最小距离
    double rs = r*r;

    double boxsize = r/sqrt(3); //分割网格的边长

    int samples_in_dimx = static_cast<int>(floor(dx / boxsize));
    int samples_in_dimy = static_cast<int>(floor(dy / boxsize));
    int samples_in_dimz = static_cast<int>(floor(dz / boxsize));

    int map[samples_in_dimx][samples_in_dimy][samples_in_dimz]  = {0};
    for (int i = 0; i < pts.size(); ++i) {
        int x_cell = floor(samples_in_dimx*(pts[i].x()-range_x[0])/dx);
        int y_cell = floor(samples_in_dimy*(pts[i].y()-range_y[0])/dy);
        int z_cell = floor(samples_in_dimz*(pts[i].z()-range_z[0])/dz);

        std::vector<int> neigh;
        // 选取周围的5*5*5个box
        for (int j = 0; j < 5; ++j) {
            for (int k = 0; k < 5; ++k) {
                for (int l = 0; l < 5; ++l) {
                    int pos_x = std::max(x_cell-2+j,0);
                    pos_x = std::min(pos_x,samples_in_dimx-1);

                    int pos_y = std::max(y_cell-2+k,0);
                    pos_y = std::min(pos_y,samples_in_dimy-1);

                    int pos_z = std::max(z_cell-2+l,0);
                    pos_z = std::min(pos_z,samples_in_dimz-1);

                    if(map[pos_x][pos_y][pos_z]!=0)
                    {
                        neigh.push_back(map[pos_x][pos_y][pos_z]);
                    }
                }
            }
        }
        if (neigh.size() == 0)
        {
            map[x_cell][y_cell][z_cell] = i;
        } else if(checkDistance(pts,neigh,i,rs)){
            //查看距离
            map[x_cell][y_cell][z_cell] = i;
        }

    }
    std::vector<int> index;
    for (int m = 0; m < samples_in_dimx; ++m) {
        for (int i = 0; i < samples_in_dimy; ++i) {
            for (int j = 0; j < samples_in_dimz; ++j) {
                if (map[m][i][j]!=0)
                {
                    index.push_back(map[m][i][j]);
                }
            }
        }
    }
    for (int n = 0; n < index.size(); ++n) {
        samplePC.push_back(pts[index[n]]);
        samplePCNor.push_back(nos[index[n]].normalized());
    }

}

void computePPF(Eigen::Vector3d& pt1, Eigen::Vector3d& nor1, Eigen::Vector3d& pt2, Eigen::Vector3d& nor2,std::vector<double>& f)
{
    Eigen::Vector3d d = pt2 - pt1;
    double norm = d.norm();
//    d.normalize();

    f.push_back(atan2(nor1.cross(d).norm(), nor1.dot(d)));
    f.push_back(atan2(nor2.cross(d).norm(), nor2.dot(d)));
    f.push_back(atan2(nor1.cross(nor2).norm(), nor1.dot(nor2)));
    f.push_back(norm);
    /**
    double a[6] = {0}; double b[6] = {0};
    for (int i = 0; i < 3; ++i) {
        a[i] = pt1(i);
        b[i] = pt2(i);
        a[i+3] = nor1(i);
        b[i+3] = nor2(i);
    }

    double d[3]= {b[0]-a[0], b[1]-a[1], b[2]-a[2]};
    double norm=sqrt(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
    d[0] /= norm;
    d[1] /= norm;
    d[2] /= norm;
    double cross1[3]= { a[4]*d[2]-a[5]*d[1]  ,  a[5]*d[0]-a[3]*d[2]  ,  a[3]*d[1]-a[4]*d[0] };
    double cross2[3]= { b[4]*d[2]-b[5]*d[1]  ,  b[5]*d[0]-b[3]*d[2]  ,  b[3]*d[1]-b[4]*d[0] };
    double cross3[3]= { a[4]*b[5]-a[5]*b[4]  ,  a[5]*b[3]-a[3]*b[5]  ,  a[3]*b[4]-a[4]*b[3] };
    f.push_back(atan2( sqrt(cross1[0]*cross1[0] + cross1[1]*cross1[1] + cross1[2]*cross1[2]) ,  d[0]*a[3]+d[1]*a[4]+d[2]*a[5] ));
    f.push_back(atan2( sqrt(cross2[0]*cross2[0] + cross2[1]*cross2[1] + cross2[2]*cross2[2]) ,  d[0]*b[3]+d[1]*b[4]+d[2]*b[5] ));
    f.push_back(atan2( sqrt(cross3[0]*cross3[0] + cross3[1]*cross3[1] + cross3[2]*cross3[2]) ,  a[3]*b[3]+a[4]*b[4]+a[5]*b[5] ));
    f.push_back(norm);
     **/

}

void trainModel(std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& nor, std::vector<Eigen::Vector3d>& sampledPC, std::vector<Eigen::Vector3d>& sampledPCNor,double& angle_step, double& distance_step,int& model_num,std::map<int,std::vector<std::vector<double>>>& hash_table)
{
//    double angle_step, distance_step;
    std::vector<double> range_x, range_y, range_z;
    // 求包围盒
    range_x.push_back(rangeMin(pts,1));
    range_x.push_back(rangeMax(pts,1));

    range_y.push_back(rangeMin(pts,2));
    range_y.push_back(rangeMax(pts,2));

    range_z.push_back(rangeMin(pts,3));
    range_z.push_back(rangeMax(pts,3));

    double dx = range_x.back() - range_x.front();
    double dy = range_y.back() - range_y.front();
    double dz = range_z.back() - range_z.front();

    double d = sqrt(dx*dx + dy*dy + dz*dz); //包围盒 对角线长度

    distance_step = d*0.04;
    angle_step    = (360/30)*M_PI/180;



    samplePCPoisson(pts,nor,0.04,sampledPC,sampledPCNor);


    std::cout << "sampled Point cloud size: " << sampledPC.size() << std::endl;
    model_num = sampledPC.size();
//    std::map<int,std::vector<std::vector<double>>> hash_table;

    for (int i = 0; i < sampledPC.size(); ++i) {
        Eigen::Vector3d pt1 = sampledPC[i];
        Eigen::Vector3d nor1 = sampledPCNor[i];
        for (int j = 0; j < sampledPC.size(); ++j) {
            if(i!=j)
            {
                Eigen::Vector3d pt2 = sampledPC[j];
                Eigen::Vector3d nor2 = sampledPCNor[j];
                std::vector<double> f ;
                computePPF(pt1,nor1,pt2,nor2,f);
                uint32_t hash = hashPPF(f,angle_step,distance_step);
                double model_alpha = computeAlpha(pt1,nor1,pt2,nor2);
                std::vector<double> node;

                node.push_back(i);
                node.push_back(model_alpha);

                std::map<int,std::vector<std::vector<double>>>::iterator iter;
                iter = hash_table.find(hash);
                if (iter == hash_table.end())
                {
                    std::vector<std::vector<double>> nodes;
                    nodes.push_back(node);
                    hash_table[hash] = nodes;
                }else{
                    std::vector<std::vector<double>> nodes;
//                    std::cout << "got one " << std::endl;
                    nodes = hash_table[hash];
                    nodes.push_back(node);
                    hash_table[hash] = nodes;
                }
            }
        }
        if(i%10 == 0)
        {
            std::cout << "trained: " << std::round(100*i/sampledPC.size()) << "%" << std::endl;
        }
    }

}


void match(std::vector<Eigen::Vector3d>& pts_s, std::vector<Eigen::Vector3d>& nor_s,
           std::vector<Eigen::Vector3d>& pts_m, std::vector<Eigen::Vector3d>& nor_m,
           std::map<int,std::vector<std::vector<double>>>& hash_table,
           double model_distance_step,double model_angle_step,
           int model_sample_num, std::vector<Pose3D>& poses_list)
{

    std::vector<double> range_x, range_y, range_z;
    // 求包围盒
    range_x.push_back(rangeMin(pts_s,1));
    range_x.push_back(rangeMax(pts_s,1));

    range_y.push_back(rangeMin(pts_s,2));
    range_y.push_back(rangeMax(pts_s,2));

    range_z.push_back(rangeMin(pts_s,3));
    range_z.push_back(rangeMax(pts_s,3));

    double dx = range_x.back() - range_x.front();
    double dy = range_y.back() - range_y.front();
    double dz = range_z.back() - range_z.front();

    double d = sqrt(dx*dx + dy*dy + dz*dz); //包围盒 对角线长度


    std::vector<Eigen::Vector3d> sampledPC, sampledPCNor;
    samplePCPoisson(pts_s,nor_s,model_distance_step/d,sampledPC,sampledPCNor);



    std::cout << "scene sampled Point cloud size: " << sampledPC.size() << std::endl;

    for (int i = 0; i < sampledPC.size(); i = i + 5) {
        Eigen::Vector3d pt1 = sampledPC[i];
        Eigen::Vector3d nor1 = sampledPCNor[i];
        std::cout << "Matching of: " << i << " of " << sampledPC.size() << "points" << std::endl;
        std::vector<int> accumulator;
        accumulator.resize(static_cast<unsigned long>(model_sample_num * 30));
//        Eigen::Affine3d gTm = transformRT(pt1,nor1);
        std::vector<int> index_close_than_diameter = closeThanModelDiameter(pt1,sampledPC,d);
        for (int j = 0; j < index_close_than_diameter.size(); ++j) {
            if (i!=j)
            {
                Eigen::Vector3d pt2 = sampledPC[j];
                Eigen::Vector3d nor2 = sampledPCNor[j];
                std::vector<double> f;
                computePPF(pt1,nor1,pt2,nor2,f);
                uint32_t hash = hashPPF(f,model_angle_step,model_distance_step);
                double scene_alpha = computeAlpha(pt1,nor1,pt2,nor2);
                if(hash_table.find(hash)!=hash_table.end())
                {
                    std::vector<std::vector<double>>& node_list = hash_table[hash];
                    for (int k = 0; k < node_list.size(); ++k) {
                        int model_index = static_cast<int>(node_list[k][0]);
                        double model_alpha = node_list[k][1];
                        double alpha = model_alpha - scene_alpha;
                        if (alpha < -M_PI)
                        {
                            alpha += 2 * M_PI;
                        } else if (alpha > M_PI)
                        {
                            alpha -= 2*M_PI;
                        }
                        int alpha_index = static_cast<int>(round((alpha + M_PI) / (2 * M_PI )* 29));
                        int accu_index  = model_index*30 + alpha_index;

                        accumulator[accu_index] = accumulator[accu_index] + 1; //投票

                    }
                }
            }
        }

        std::vector<int> the_max_index = findTheMaxIndexsInAccu(accumulator,0.95);
        for (int l = 0; l < the_max_index.size(); ++l) {
            int alpha_max_index = the_max_index[l]%30;
            int max_index = the_max_index[l]/30;
            Eigen::Affine3d gTs = transformRT(pt1,nor1);
            Eigen::Affine3d gTm = transformRT(pts_m[max_index],nor_m[max_index]);
            double alpha = 2*M_PI*alpha_max_index/29 -M_PI;
            Eigen::Affine3d Ta;
            Eigen::AngleAxisd angle_axis = Eigen::AngleAxisd(alpha,Eigen::Vector3d::UnitX());
            Ta = angle_axis.toRotationMatrix();
            Eigen::Affine3d Pose = gTs.inverse()*Ta*gTm;
            int votes = accumulator[the_max_index[l]];
            Pose3D pose_3d;
            pose_3d.pose = Pose;
            pose_3d.votes = votes;
            poses_list.push_back(pose_3d);

        }
    }
    std::cout << "Poses : " << poses_list.size() << std::endl;


}



void clusterPoses(std::vector<Pose3D>& poses, std::vector<Cluster>& clusters,double distance_limitation, double angle_limitation)
{


    for (auto pose:poses) {
        bool assigned = false;
        for(auto cluster: clusters)
        {
            Pose3D pose_center = cluster.poses.front();
            if(comparePoses(pose,pose_center,distance_limitation,angle_limitation))
            {
                cluster.poses.push_back(pose);
                cluster.accu_votes += pose.votes;
                assigned = true;
                break;
            }
        }
        if (!assigned)
        {
            Cluster cluster_1;
            cluster_1.poses.push_back(pose);
            cluster_1.accu_votes = pose.votes;
            clusters.push_back(cluster_1);
        }
    }
}

void averageClusters(const std::vector<Cluster>& clusters, std::vector<Pose3D>& out_poses)
{
    for (auto cluster:clusters) {
        Pose3D pose = averagePoses(cluster.poses);
        pose.votes = cluster.accu_votes;
        out_poses.push_back(pose);
    }
}

Pose3D averagePoses(std::vector<Pose3D>& in_poses)
{
    Eigen::Vector3d acc_t, avg_t;
    std::vector<Eigen::Vector4d> qs;
    Pose3D res;
    for(auto pose:in_poses)
    {
        Eigen::Quaterniond q = Eigen::Quaterniond(pose.pose.rotation());
        Eigen::Vector4d v(q.w(),q.x(),q.y(),q.z());
        acc_t += pose.pose.translation();
        qs.push_back(v);
    }
    Eigen::MatrixXd A(4,4);
    for(auto q:qs)
    {
        A += q*q.transpose();
    }
    A = A*(1.0/static_cast<double>(qs.size()));
    Eigen::EigenSolver<Eigen::MatrixXd> es(A);
    Eigen::VectorXcd q_avg = es.eigenvectors().col(0);
    avg_t = acc_t/static_cast<double>(in_poses.size());
    Eigen::Quaterniond q_ = Eigen::Quaterniond(q_avg.real().col(0)[0] , q_avg.real().col(0)[1] , q_avg.real().col(0)[2] , q_avg.real().col(0)[3]);
    res.pose = q_.toRotationMatrix();
    res.pose.translation() = avg_t;
    res.votes = 0;
    return res;

}

bool comparePoses(Pose3D& pose1, Pose3D& pose2, double distance_limitation, double angle_limitation)
{
    /**
     * 判断两个pose 是否类似
     */
    Eigen::Vector3d d;
    d = pose1.pose.translation() - pose2.pose.translation();
    double normd = d.norm();
    double alpha;
    Eigen::AngleAxisd angleAxis1 = Eigen::AngleAxisd(pose1.pose.rotation().matrix());
    Eigen::AngleAxisd angleAxis2 = Eigen::AngleAxisd(pose2.pose.rotation().matrix());
    alpha = std::abs(angleAxis1.angle() - angleAxis2.angle());
    if(normd < distance_limitation && alpha < angle_limitation)
    {
        return true;
    } else{
        return false;
    }


}

void sortPoses(std::vector<Pose3D>& poses)
{
    /**
     * 按照投票来对poses排序
     */
    std::sort(poses.begin(),poses.end(),[](const Pose3D& a, const Pose3D& b){
        return a.votes > b.votes;
    });
}

std::vector<int> findTheMaxIndexsInAccu(std::vector<int>& accumulator, double rate)
{
    int max = 0;
    for (int l = 0; l < accumulator.size(); ++l) {
        if(accumulator[l] > max)
            max = accumulator[l];
    }
    std::vector<int> the_max_list;
    for (int m = 0; m < accumulator.size(); ++m) {
        if(accumulator[m] > max*rate)
        {
            the_max_list.push_back(m);
        }
    }
    return the_max_list;
}

std::vector<int> closeThanModelDiameter(Eigen::Vector3d& pt, std::vector<Eigen::Vector3d>& sampledPC,double diameter)
{
    std::vector<int> index_in_sampledPC;
    for (int i = 0; i < sampledPC.size(); ++i) {
        Eigen::Vector3d d_v = sampledPC[i] - pt;
        if(d_v.norm() < diameter)
        {
            index_in_sampledPC.push_back(i);
        }
    }
    return index_in_sampledPC;
}

uint32_t hashPPF(std::vector<double> f, double angle_step, double distance_step)
{
    uint32_t *key = new uint32_t [4];             /* input scalar */
//    char key[4] = {0};
    key[0] = static_cast<uint32_t>(floor(f[0]/angle_step));
    key[1] = static_cast<uint32_t>(floor(f[1]/angle_step));
    key[2] = static_cast<uint32_t>(floor(f[2]/angle_step));
    key[3] = static_cast<uint32_t>(floor(f[3]/distance_step));


    uint32_t* outMatrix = new uint32_t [4];              /* output matrix */

    MurmurHash3_x86_32(key,16,42,outMatrix);

    return outMatrix[0];
//    return murmur3_32(key,16,42);
}

double computeAlpha(Eigen::Vector3d pt1,Eigen::Vector3d nor1,Eigen::Vector3d pt2,Eigen::Vector3d nor2)
{
    Eigen::Affine3d T = transformRT(pt1,nor1);
    Eigen::Vector3d mpt = T*pt2;
    double alpha = atan2(-mpt.z(),mpt.y());
    return alpha;
}

Eigen::Affine3d transformRT(Eigen::Vector3d pt,Eigen::Vector3d nor)
{
    /**
     * 把一个点法，归一到法向量在x轴上
     */

    double angle = acos(nor.x());
    Eigen::Vector3d axis(0,nor.z(),-nor.y());
    if(nor.y() == 0 && nor.z() == 0 )
    {
        axis.x() = 0;
        axis.y() = 1;
        axis.z() = 0;
    } else
    {
        axis.normalize();
    }
    Eigen::AngleAxisd angle_axis(angle,axis);
    Eigen::Matrix3d R = angle_axis.toRotationMatrix();
    Eigen::Vector3d t = -R*pt;
    Eigen::Affine3d T;
    T = R;
    T.translation() = t;
    return T;
}

bool checkDistance(std::vector<Eigen::Vector3d> &pts,std::vector<int>& neigh, int k, double rs)
{
    for (int i = 0; i < neigh.size(); ++i) {
        if (std::pow((pts[neigh[i]].x() - pts[k].x()),2)+std::pow((pts[neigh[i]].y() - pts[k].y()),2)+std::pow((pts[neigh[i]].z() - pts[k].z()),2) < rs)
        {
            return false;
        }
    }
    return true;
}

double rangeMax(std::vector<Eigen::Vector3d> &pts, int dim)
{

    if(dim == 1)
    {
        return (*std::max_element(pts.begin(),pts.end(),compare_x)).x();
    }else if(dim == 2)
    {
        return (*std::max_element(pts.begin(),pts.end(),compare_y)).y();
    }else if(dim == 3)
    {
        return (*std::max_element(pts.begin(),pts.end(),compare_z)).z();
    }else
    {
        std::cerr << "The dim of point cloud is error" << std::endl;
    }
}

double rangeMin(std::vector<Eigen::Vector3d> &pts, int dim)
{

    if(dim == 1)
    {
        return (*std::min_element(pts.begin(),pts.end(),compare_x)).x();
    }else if(dim == 2)
    {
        return (*std::min_element(pts.begin(),pts.end(),compare_y)).y();
    }else if(dim == 3)
    {
        return (*std::min_element(pts.begin(),pts.end(),compare_z)).z();
    }else
    {
        std::cerr << "The dim of point cloud is error" << std::endl;
    }
}

bool compare_x(Eigen::Vector3d& i, Eigen::Vector3d& j)
{
    return i.x() < j.x();
}

bool compare_y(Eigen::Vector3d& i, Eigen::Vector3d& j)
{
    return i.y() < j.y();
}

bool compare_z(Eigen::Vector3d& i, Eigen::Vector3d& j)
{
    return i.z() < j.z();
}
