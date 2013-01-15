#define _USE_MATH_DEFINES
#include <cmath>
#include <random>

#include "particle_filter.h"
#include <iostream>

ParticleFilter::ParticleFilter(double map_min_x, double map_max_x, double map_min_y, double map_max_y, ComputeParticleWeightCallback callback)
: callback_(callback)
, particle_set_size_( (map_max_x-map_min_x)*(map_max_y-map_min_y)*4 )
, motion_guesses_(1)
, random_particles_size_(0.05)
, motion_model_(0.02, 0.01)
, distribution_x_(map_min_x, map_max_x)
, distribution_y_(map_min_y, map_max_y)
, distribution_theta_(-M_PI, M_PI)

{
  // initialize particle set with number of random particles
  for (uint i = 0; i < particle_set_size_; i++) {
    particles_.push_back(generateRandomParticle());
  }
}

void ParticleFilter::update(double x, double y, double yaw)
{
  //============================== YOUR CODE HERE ==============================
  // Instructions: do one complete update of the particle filter. It should
  //               generate new particle set based on the given motion
  //               parameters and the old particle set, compute the weights of
  //               the particles, resample the set, and update the private
  //               member field that holds the current best pose estimate
  //               (pose_estimate_). Note that the motion parameters provided
  //               to this function are in robot's reference frame.
  //
  // Hint: to compute the weight of a particle, use the callback_ member field:
  //
  //           Particle particle;
  //           double weight = callback_(particle);
  //
  // Hint: to sample a uniformly distributed number in [min, max] range use:
  //
  //           double r = std::uniform_real_distribution<double>(min, max)(random_generator_);
  //

  //std::cout << "particles: size: " << particles_.size() << std::endl;


  // initialize resampling wheel
  std::vector<double> wheel;
  double total_weight = 0.0;

  motion_model_.setMotion(x, y, yaw);

  // get new pose for all particles based upon the motion model
  for ( uint i = 0; i < particle_set_size_; i++ ){
    particles_.at(i).pose = motion_model_.sample( particles_.at(i).pose );
    particles_.at(i).weight = callback_( particles_.at(i) );

    // add total weight to the resampling wheel
    total_weight = total_weight + particles_.at(i).weight;
    wheel.push_back( total_weight );
  }

  // resample particle set

  ParticleVector new_particle_set;
  double ran = std::uniform_real_distribution<double>(0.0, total_weight)(random_generator_);
  for (uint i = 0; i < particle_set_size_; i++) {
    for ( uint i = 0; i < wheel.size(); i++ ){
      if (ran > wheel.at(i)) {
        new_particle_set.push_back( particles_.at(i) );
      }
    }
  }




  //============================================================================
}

Particle ParticleFilter::generateRandomParticle()
{
  Particle p;
  p.pose.x = distribution_x_(random_generator_);
  p.pose.y = distribution_y_(random_generator_);
  p.pose.theta = distribution_theta_(random_generator_);
  p.weight = 0.0;
  return p;
}
