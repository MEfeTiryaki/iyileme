#pragma once

#include "OptimizerInterface.hpp"

namespace iyi {
class IterativeOptimizerBase : public OptimizerInterface
{
  public:
    // Constructor.
    IterativeOptimizerBase():
      OptimizerInterface(),
      maxIteration_(1000)
    {};
    // Destructor.
    virtual ~IterativeOptimizerBase(){};
    // Init
    virtual void initilize( int stateLength
                          , int maxIteration){
      OptimizerInterface::initilize(stateLength);
      maxIteration_ = maxIteration;
    };

  protected:
    int maxIteration_;

};

} // iyi
