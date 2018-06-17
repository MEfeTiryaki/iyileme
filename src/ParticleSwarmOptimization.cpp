
#include "iyileme/ParticleSwarmOptimization.hpp"


namespace iyi {

ParticleSwarmOptimization::ParticleSwarmOptimization():
  IterativeOptimizerBase(),
  numberOfParticles_(100)
{
}

ParticleSwarmOptimization::~ParticleSwarmOptimization()
{
}


void ParticleSwarmOptimization::initilize( int stateLength
                   , int maxIteration
                   , int numberOfParticles
                   , iyi::Bound bound
                   , iyi::ParticleSwarmOptimizationConstants constants
                   , std::function<double(Eigen::VectorXd) > func)
{
  IterativeOptimizerBase::initilize(stateLength,maxIteration);
  // Problem varibles
  numberOfParticles_ = numberOfParticles;
  cost_ = func;
  bound_ = bound;
  constants_ = constants;

  // Particle initiation
  particleInitiate();

}

void ParticleSwarmOptimization::particleInitiate(){
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0,1.0);
  for(int i=0; i<numberOfParticles_; i++){
    particles_.push_back(Particle());
    particles_.back().position = Eigen::VectorXd(stateLength_);
    particles_.back().velocity = Eigen::VectorXd(stateLength_);
    particles_.back().cost = 0.0;
    particles_.back().localMinimumPosition = Eigen::VectorXd(stateLength_);
    particles_.back().localMinimumCost = 0.0;
  }
  double k ;
  for( int i=0 ;i<numberOfParticles_;i++){
    for( int j=0;j<stateLength_;j++){
      k = distribution(generator);
      particles_[i].position[j] = bound_.lower[j]
                             + k
                             * ( bound_.upper[j]
                               - bound_.lower[j]);
       k = distribution(generator);
       particles_[i].velocity[j] = bound_.startVelocityMax[j]*(k - 0.5);
    }
    particles_[i].cost = cost_(particles_[i].position);
    particles_[i].localMinimumCost = particles_[i].cost;
    particles_[i].localMinimumPosition = particles_[i].position;
    /*
    std::cout << "particle_"<<i <<"\n";
    std::cout <<"\t"<< particles_[i].position.transpose()<<"\n";
    std::cout <<"\t"<< particles_[i].velocity.transpose()<<"\n";
    std::cout << particles_[i].cost <<"\n";
    //*/
  }
}

void ParticleSwarmOptimization::optimize(){
  int start_s=clock();
  int i = 0 ;
  Eigen::VectorXd temp_pos ;
  Eigen::VectorXd temp_vel ;
  while( i < maxIteration_){
    for(int j=0; j<numberOfParticles_; j++){
      // Save current position and velocities
      temp_pos = particles_[j].position ;
      temp_vel = particles_[j].velocity ;
      // Update position and velocity
      particles_[j].position += constants_.gamma * temp_vel;
      particles_[j].velocity = constants_.alpha * particles_[j].velocity
            + constants_.beta * (globalMinimumValues_ - temp_pos)
            + constants_.zeta * Eigen::VectorXd::Random(stateLength_);
      // Update cost
      particles_[j].cost = cost_(particles_[j].position);
      if (particles_[j].cost<particles_[j].localMinimumCost){
        particles_[j].localMinimumCost = particles_[j].cost;
        particles_[j].localMinimumPosition = particles_[j].position;
        if (particles_[j].localMinimumCost < globalMinimumCost_){
          globalMinimumCost_ = particles_[j].localMinimumCost;
          globalMinimumValues_ = particles_[j].localMinimumPosition;
        }
      }
    }
    constants_.alpha *=0.99;
    i++ ;
  }
  // the code you wish to time goes here
  int stop_s = clock();
  std::cout << "time: " << (stop_s-start_s)/double(CLOCKS_PER_SEC) << " sec" << std::endl;
  std::cout << globalMinimumCost_ <<"\n"<<  globalMinimumValues_.transpose() <<"\n";

}

} /* namespace iyi*/
