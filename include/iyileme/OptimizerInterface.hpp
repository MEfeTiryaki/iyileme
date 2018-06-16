#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

namespace iyi {
class OptimizerInterface
{
  public:
    // Constructor.
    OptimizerInterface(){};
    // Destructor.
    virtual ~OptimizerInterface(){};
    // Init
    virtual void initilize( int stateLength){
      stateLength_ = stateLength ;
    };

    virtual void optimize(){};

  protected:

    int stateLength_;

    Eigen::VectorXd globalMinimumValues_;
    double globalMinimumCost_;
};

} // iyi
