#include "ros/ros.h"
#include <iostream>
#include "iyileme/QuadraticOptimization.hpp"

void read(Eigen::MatrixXd& A_,Eigen::VectorXd& B_
         ,std::vector<double> A ,std::vector<double> B)
{
  int m = B.size();
  int n = A.size()/m;


  A_ = Eigen::MatrixXd(n,m);
  B_ = Eigen::VectorXd(m);
  for (int i = 0; i<m; i++){
    for (int j = 0; j<n; j++){
        A_(i,j) = A[m*i+j];
    }
    B_[i] = B[i];
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  iyi::QuadraticOptimization optimizer = iyi::QuadraticOptimization();
  Eigen::MatrixXd H;
  Eigen::VectorXd f;
  Eigen::MatrixXd E;
  Eigen::VectorXd d;
  Eigen::MatrixXd A;
  Eigen::VectorXd b;
  std::vector<double> H_;
  std::vector<double> f_;
  std::vector<double> E_;
  std::vector<double> d_;
  std::vector<double> A_;
  std::vector<double> b_;
  nh.getParam("/QuadraticOptimization/H", H_);
  nh.getParam("/QuadraticOptimization/f", f_);
  nh.getParam("/QuadraticOptimization/A", A_);
  nh.getParam("/QuadraticOptimization/b", b_);
  nh.getParam("/QuadraticOptimization/E", E_);
  nh.getParam("/QuadraticOptimization/d", d_);
  read(H,f,H_,f_);
  read(E,d,E_,d_);
  read(A,b,A_,b_);
  optimizer.setCost(H,f);
  optimizer.setEqualityConstrains(E,d);
  /*
  optimizer.setInequalityConstrains(A,b);
  optimizer.optimize();
  */
  optimizer.optimize();


  return 0;
}
