
#include "iyileme/QuadraticOptimization.hpp"


namespace iyi {

QuadraticOptimization::QuadraticOptimization():
  OptimizerInterface(),
  isInequalityConstrained(false),
  isEqualityConstrained(false)
{
}

QuadraticOptimization::~QuadraticOptimization()
{
}


void QuadraticOptimization::setCost( Eigen::MatrixXd H
                                   , Eigen::VectorXd f)
{
  OptimizerInterface::initilize(f.size());
  // Problem varibles
  H_ = H;
  f_ = f;
}
void QuadraticOptimization::setInequalityConstrains( Eigen::MatrixXd A
                                   , Eigen::VectorXd b)
{
  A_ = A;
  b_ = b;
  int rows = A_.rows();
  int cols = A_.cols();
  Eigen::VectorXd x ;
  Eigen::VectorXd y ;
  Eigen::VectorXd z ;
  double k ;
  for(int i = 0 ;i < rows-1; i++){
    for(int j = i+1 ;j < rows; j++){
      x = A.row(i);
      y = A.row(j);
      k = (x.transpose()*x)/(x.transpose()*y) ;
      z = y*k;
      bool flag = true;
      for (int k=0 ;k <cols ;k++ ){
          flag &= z[k]==x[k];
          if(!flag)
            break;
      }
      if(flag){
        A_.conservativeResize(A_.rows(), A_.cols()+1);
        b_.conservativeResize(b_.size()+1);
        if (k*b[i]<b[j]){
          b
        }
        A_.conservativeResize(mat.rows(), mat.cols()+1);
        A_.col(mat.cols()-1) = vec;
        std::cout<< "equal const" << std::endl;
      }
    }
  }

  isInequalityConstrained = true ;
}
void QuadraticOptimization::setEqualityConstrains( Eigen::MatrixXd E
                                   , Eigen::VectorXd d)
{
  E_ = E;
  d_ = d;
  isEqualityConstrained = true ;
}


void QuadraticOptimization::optimize(){
  int start_s=clock();
  if(isEqualityConstrained&&!isInequalityConstrained){
    EqualityOptimize();
  }else if (isInequalityConstrained){
    InequalityOptimize();
  }else{
    globalMinimumValues_ = Eigen::VectorXd::Zero(f_.size());
    globalMinimumCost_ = 0.0;
  }
  int stop_s = clock();
  std::cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC) << " sec" << std::endl;
  std::cout << globalMinimumCost_ << std::endl;
  std::cout << globalMinimumValues_.transpose() << std::endl;
}

void QuadraticOptimization::EqualityOptimize(){
  std::cout<<"EqualityOptimize"<<std::endl;
  const int n_c = f_.size() ;
  const int n_e = d_.size() ;
  const int n = n_c + n_e;

  Eigen::MatrixXd K = Eigen::MatrixXd::Zero(n,n);
  Eigen::VectorXd l = Eigen::VectorXd::Zero(n);
  K.block(0,0,n_c,n_c) = H_;
  K.block(n_c,0,n_e,n_c) = E_;
  K.block(0,n_c,n_c,n_e) = E_.transpose();
  l.segment(0,n_c) = f_;
  l.segment(n_c,n_e) = d_;
  std::cout<< K <<std::endl ;
  std::cout<< l <<std::endl ;
  globalMinimumValues_ = (K.inverse()*l).segment(0,n_c);
  globalMinimumCost_ = (0.5 * globalMinimumValues_.transpose()
                     * H_ * globalMinimumValues_
                   - globalMinimumValues_.transpose()*f_)[0];
}
void QuadraticOptimization::InequalityOptimize(){

}
} /* namespace iyi*/
