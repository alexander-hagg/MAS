#ifndef PARTICLE_FILTER_H
#define PARTICLE_FILTER_H

#include <memory>
#include <functional>

#include "particle.h"
#include "motion_model.h"

class ParticleFilter
{

public:

  /** Convenience type for unique pointer to ParticleFilter. */
  typedef std::unique_ptr<ParticleFilter> UPtr;

  /** Convenince type for functions that compute the weigth of a particle. */
  typedef std::function<double(const Particle&)> ComputeParticleWeightCallback;

  /** Construct a particle filter.
    *
    * The first four parameters define the extent of the world. This influences
    * the region in which a random particle might appear. The last parameter is
    * a callback to a function that should be used to compute the weight of a
    * particle. The particle filter per se does not depend on what physical
    * meaning the particle weight has and how it is computed, and thus its
    * computation is outsourced to this function. */
  ParticleFilter(double map_min_x, double map_max_x,
                 double map_min_y, double map_max_y,
                 ComputeParticleWeightCallback callback);

  /** Given a motion command compute the new belief state of the filter. */
  void update(double x, double y, double yaw);

  /** Get the current porticle set that represents the belief state of the
    * particle filter. */
  const ParticleVector& getParticles() const { return particles_; }

  /** Get the current best estimate of the robot's pose. */
  Pose getPoseEstimate() const { return pose_estimate_; }

private:

  /** A helper function that generates a particle with zero weight and a random
    * uniformly distributed pose within the map. */
  Particle generateRandomParticle();

  /** Particle weight computation callback. */
  ComputeParticleWeightCallback callback_;

  /** Current particle set. */
  ParticleVector particles_;

  /* Particle filter parameters */

  /** Number of particles in the current particle set after resampling.
    *
    * The suggested default value: 4 particles per square meter of map area. */
  size_t particle_set_size_;

  /** Number of new particles to generate per existing particle in the motion
    * update step.
    *
    * The suggested default value is 1. */
  size_t motion_guesses_;

  /** Number of new random particles drawn from the uniform distribution over
    * the map on each update.
    *
    * This is need to recover from grossly wrong position estimates.
    *
    * The suggested default value: 5% of the particle set size or a minimum of 1
    * particle per square meter map space. */
  double random_particles_size_;

  // The current best pose estimate
  Pose pose_estimate_;

  // Motion model used to sample the poses in motion update step
  MotionModel motion_model_;

  // Required for random number generation
  std::default_random_engine random_generator_;

  // Uniform distributions from which poses for random particles are sampled
  std::uniform_real_distribution<double> distribution_x_;
  std::uniform_real_distribution<double> distribution_y_;
  std::uniform_real_distribution<double> distribution_theta_;

};

#endif /* PARTICLE_FILTER_H */

