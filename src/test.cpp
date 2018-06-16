#include "ros/ros.h"
#include <iostream>
#include "iyileme/ParticleSwarmOptimization.hpp"


double cost(Eigen::VectorXd p){
  return (p[0]-2.0)*(p[0]-2.0) + (p[1]-3.0)*(p[1]-3.0);
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;


  iyi::ParticleSwarmOptimization optimizer = iyi::ParticleSwarmOptimization();
  int maxIteration;
  int particleNumber;
  nh.getParam("/maxIteration", maxIteration);
  nh.getParam("/particleNumber", particleNumber);
  // Add bounds in yaml file
  iyi::Bound bound = iyi::Bound();
  nh.getParam("/bound/upper", bound.upper);
  nh.getParam("/bound/lower", bound.lower);
  nh.getParam("/bound/startVelocityMax", bound.startVelocityMax);

  // Add variables in yaml file
  iyi::ParticleSwarmOptimizationConstants constants = iyi::ParticleSwarmOptimizationConstants();
  nh.getParam("/gamma", constants.gamma);
  nh.getParam("/alpha", constants.alpha);
  nh.getParam("/beta", constants.beta);
  nh.getParam("/zeta", constants.zeta);



  optimizer.initilize(2,maxIteration,particleNumber,bound,constants,cost);
  optimizer.optimize();
  return 0;
}
