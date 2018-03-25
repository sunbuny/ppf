//
// Created by sun on 18-3-24.
//

#include <iostream>
#include <Eigen/Dense>

void printQuaternion(Eigen::Quaterniond& q)
{
    std::cout << "q: " << std::endl;
    std::cout << q.w() << " " << q.x() << " "<< q.y() << " " << q.z() << " " <<std::endl;
}

void test()
{
    std::vector<Eigen::Vector4d> qs;
    qs.push_back(Eigen::Vector4d(0.205689724296306,0.974467119764441,-0.00438713894185257,0.0899239836128837));
    qs.push_back(Eigen::Vector4d(0.196697164585351,0.973694567732456,0.0417243753410247,0.107182977728699));
    Eigen::MatrixXd A(4,4);
    for(auto q:qs)
    {
        A += q*q.transpose();
    }
    A = A*(1.0/static_cast<double>(qs.size()));
    Eigen::EigenSolver<Eigen::MatrixXd> es(A);
    Eigen::VectorXcd q_avg = es.eigenvectors().col(0);
    std::cout << es.eigenvalues() << std::endl;
    std::cout << q_avg.real().col(0)[3]<< std::endl;
    Eigen::Matrix3d T;

    Eigen::Quaterniond q222 =  Eigen::Quaterniond(q_avg.real().col(0)[0],q_avg.real().col(0)[1],q_avg.real().col(0)[2],q_avg.real().col(0)[3]);
    printQuaternion(q222);
    T = Eigen::Quaterniond(q_avg.real().col(0)[0],q_avg.real().col(0)[1],q_avg.real().col(0)[2],q_avg.real().col(0)[3]).normalized().toRotationMatrix();
////    T.translation() = qs.front().head<3>();
//    std::cout << T.rotation() << std::endl;
//    std::cout << T.translation() << std::endl;
    Eigen::Quaterniond qqq = Eigen::Quaterniond(T);

    std::cout << qqq.w() << " " << qqq.x() << " "<< qqq.y() << " " << qqq.z() << " " <<std::endl;
    std::cout << T << std::endl << std::endl;
    std::cout << T*T.transpose() << std::endl;

}


int main(int argc, char** argv)
{
    test();
    return 0;
}
