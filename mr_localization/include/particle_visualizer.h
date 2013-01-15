#ifndef PARTICLE_VISUALIZER_H
#define PARTICLE_VISUALIZER_H

#include <memory>
#include <string>

#include <ros/ros.h>

#include "particle.h"

/** A helper class that visualizes sets of partiles.
  *
  * Given a vector of particles it constructs and publishes ROS marker array,
  * that could be viewed with RViz. Each particle is represented by an arrow
  * having the pose proposed by the particle and the alpha corresponding to its
  * weight. The particle with the largest weight has alpha 1.0 (i.e. completely
  * opaque), and the particle with the smallest weight has alpha 0.1 (i.e.
  * almost transparent). */
class ParticleVisualizer
{

public:

  typedef std::unique_ptr<ParticleVisualizer> UPtr;

  ParticleVisualizer(const std::string& topic_name, const std::string& frame_id);

  void publish(const ParticleVector& particles);

private:

  ros::Publisher marker_publisher_;

  const std::string frame_id_;

};

#endif /* PARTICLE_VISUALIZER_H */

