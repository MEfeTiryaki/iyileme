#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <Eigen/Dense>
#include <iostream>
#include <random>
#include <ctime>

#include "OptimizerInterface.hpp"
namespace iyi {


class QuadraticOptimization: public OptimizerInterface
{
  public:
    // Constructor.
    QuadraticOptimization();
    // Destructor.
    virtual ~QuadraticOptimization();
    // Init
    virtual void setCost( Eigen::MatrixXd H
                        , Eigen::VectorXd f);
    virtual void setInequalityConstrains( Eigen::MatrixXd A
                          , Eigen::VectorXd b);
    virtual void setEqualityConstrains( Eigen::MatrixXd E
                          , Eigen::VectorXd ds);

    virtual void optimize() override;

    virtual void EqualityOptimize();
    virtual void InequalityOptimize();
  private:
    void particleInitiate();

  protected:
    Eigen::MatrixXd H_;
    Eigen::VectorXd f_;

    bool isEqualityConstrained;
    Eigen::MatrixXd E_;
    Eigen::VectorXd d_;

    bool isInequalityConstrained;
    Eigen::MatrixXd A_;
    Eigen::VectorXd b_;

};

} // iyi
