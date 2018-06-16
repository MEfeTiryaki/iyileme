#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include "IterativeOptimizerBase.hpp"
namespace iyi {

struct Particle {
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  double cost;
  Eigen::VectorXd localMinimumPosition;
  double localMinimumCost;
};

struct Bound{
  std::vector<double> lower;
  std::vector<double> upper;
  std::vector<double> startVelocityMax;
};

struct ParticleSwarmOptimizationConstants{
  double gamma;
  double alpha;
  double beta;
  double zeta;
};

class ParticleSwarmOptimization: public IterativeOptimizerBase
{
  public:
    // Constructor.
    ParticleSwarmOptimization();
    // Destructor.
    virtual ~ParticleSwarmOptimization();
    // Init
    virtual void initilize( int stateLength
                          , int maxIteration
                          , int numberOfParticles
                          , iyi::Bound bound
                          , iyi::ParticleSwarmOptimizationConstants constants
                          , std::function<double(Eigen::VectorXd) >  func);

    virtual void optimize() override;

  private:
    void particleInitiate();

  protected:
    // Particles
    int numberOfParticles_;
    std::vector<Particle> particles_;
    // Upper and Lower Bound of the optimization variables
    Bound bound_;
    // Update constants
    iyi::ParticleSwarmOptimizationConstants constants_;
    // Cost Function
    std::function<double(Eigen::VectorXd) >  cost_;

};

} // iyi
