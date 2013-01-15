#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "particle_visualizer.h"

ParticleVisualizer::ParticleVisualizer(const std::string& topic_name, const std::string& frame_id)
: frame_id_(frame_id)
{
  ros::NodeHandle nh;
  marker_publisher_ = nh.advertise<visualization_msgs::MarkerArray>(topic_name, 1);
}

void ParticleVisualizer::publish(const ParticleVector& particles)
{
  if (marker_publisher_.getNumSubscribers() == 0) return;

  double min = std::min_element(particles.begin(), particles.end())->weight;
  double max = std::max_element(particles.begin(), particles.end())->weight;
  double range = (max - min) * 0.9;

  visualization_msgs::MarkerArray markers;
  for (const auto& particle : particles)
  {
    visualization_msgs::Marker marker;
    marker.ns = "particles";
    marker.header.frame_id = frame_id_;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.6;
    marker.scale.y = 0.6;
    marker.scale.z = 0.2;
    marker.color.a = range > 0 ? 0.1 + (particle.weight - min) / range : 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.2;
    marker.color.b = 0.8;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(particle.pose.theta);
    marker.pose.position.x = particle.pose.x;
    marker.pose.position.y = particle.pose.y;
    marker.pose.position.z = 0.05;
    marker.id = markers.markers.size();
    markers.markers.push_back(marker);
  }
  marker_publisher_.publish(markers);
}

