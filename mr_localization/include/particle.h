#ifndef PARTICLE_H
#define PARTICLE_H

#include <vector>

#include "pose.h"

/** Representation of a weighted particle for the particle filter.
  *
  * This structure represents a particle of the particle filter, composed of the
  * robot's assumed pose and the weight assigned to this pose based on
  * observation match. */
struct Particle
{
  /** Robot pose proposed by this particle. */
  Pose pose;

  /** Importance of this particle after observation update.
    *
    * Measured as match between real observation and simulated observation from
    * this particle's pose. This should be a positive value between 0 and some
    * limit @c max_weight, such that a value close to zero indicate bad match
    * with real observation and a value close to @c max_weight indicates almost
    * perfect match. */
  double weight;

  /** Particle comparison operator (based on weight).
    *
    * Implementing this operator allows to use standard library algorithms to
    * find minimum and maximum elements in a vector of Particles, or sort it. */
  bool operator<(const Particle& rhs) const
  {
    return this->weight < rhs.weight;
  }

};

typedef std::vector<Particle> ParticleVector;


#endif /* PARTICLE_H */

