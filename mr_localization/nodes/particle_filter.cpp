#include <ros/ros.h>
#include <ros/console.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <mr_srvs/GetPoseLikelihood.h>

#include "particle_filter.h"
#include "particle_visualizer.h"

class ParticleFilterNode
{

public:

  ParticleFilterNode()
  {
    // Query parameters from server
    ros::NodeHandle pn("~");
    double update_rate;
    double world_width;
    double world_height;
    pn.param<double>("update_rate", update_rate, 5.0);
    pn.param<std::string>("frame_id", frame_id_, "pf_pose");
    // This will block until the stage node is loaded and has set the world
    // dimensions parameters
    while (!nh_.hasParam("world_width") || !nh_.hasParam("world_height"))
      ros::spinOnce();
    nh_.getParam("world_width", world_width);
    nh_.getParam("world_height", world_height);

    // Compute world extent
    double min_x, max_x, min_y, max_y;
    min_x = - world_width / 2.0;
    max_x = world_width / 2.0;
    min_y = - world_height / 2.0;
    max_y = world_height / 2.0;

    // Create particle filter and visualizer
    particle_filter_ = ParticleFilter::UPtr(new ParticleFilter(min_x, max_x, min_y, max_y, std::bind(&ParticleFilterNode::computeParticleWeight, this, std::placeholders::_1)));
    particle_visualizer_ = ParticleVisualizer::UPtr(new ParticleVisualizer("particles", "odom"));

    // Schedule periodic filter updates
    update_timer_ = nh_.createTimer(ros::Rate(update_rate).expectedCycleTime(), &ParticleFilterNode::updateCallback, this);

    // Misc
    pose_likelihood_client_ = nh_.serviceClient<mr_srvs::GetPoseLikelihood>("/pose_likelihood_server/get_pose_likelihood");
    previous_pose_.setIdentity();
    ROS_INFO("Started [particle_filter] node.");
  }

  void updateCallback(const ros::TimerEvent& event)
  {
    // Determine the motion since the last update
    tf::StampedTransform transform;
    try
    {
      ros::Time now = ros::Time::now();
      tf_listener_.waitForTransform("base_link", "odom", now, ros::Duration(0.1));
      tf_listener_.lookupTransform("base_link", "odom", now, transform);
    }
    catch (tf::TransformException& e)
    {
      ROS_ERROR("Unable to compute the motion since the last update...");
      return;
    }
    tf::Transform delta_transform = previous_pose_ * transform.inverse();
    previous_pose_ = transform;

    // In theory the obtained transform should be imprecise (because it is
    // based on the wheel odometry). In our system, however, we in fact get the
    // ground truth transform. It is therefore impossible to simulate the
    // "kidnapped robot" problem, because even if the robot is teleported in a
    // random spot in the world, the transform will be exact, and the particle
    // filter will not lose the track.
    // We test the length of the transform vector and if it exceeds some "sane"
    // limit we assume that the odometry system "failed" and feed identity
    // transform to the particle filter.
    if (delta_transform.getOrigin().length() > 2.0)
      delta_transform.setIdentity();

    // Perform particle filter update, note that the transform is in robot's
    // coordinate frame
    double forward = delta_transform.getOrigin().getX();
    double lateral = delta_transform.getOrigin().getY();
    double yaw = tf::getYaw(delta_transform.getRotation());
    particle_filter_->update(forward, lateral, yaw);

    // Visualize the particle set, broadcast transform, and print some information
    const auto& p = particle_filter_->getParticles();
    double avg_weight = std::accumulate(p.begin(), p.end(), 0.0, [](double sum, Particle p) { return sum + p.weight; }) / p.size();
    particle_visualizer_->publish(p);
    broadcastTransform();
    ROS_INFO("Motion: [%.3f, %.3f, %.3f] Average partile weight: %3f", forward, lateral, yaw, avg_weight);
  }

  /** Broadcast the current estimation of robot's position. */
  void broadcastTransform()
  {
    tf::Transform transform;
    const auto& pose = particle_filter_->getPoseEstimate();
    transform.getOrigin().setX(pose.x);
    transform.getOrigin().setY(pose.y);
    transform.getOrigin().setZ(2.0); // elevate so that the tf frame appears above the particles in RViz
    transform.setRotation(tf::createQuaternionFromYaw(pose.theta));
    tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", frame_id_));
  }

  /** Compute the weight of a partile.
    *
    * This function takes the pose from the particle, and uses external service
    * to determine how likely is that the robot is in this pose given the data
    * that it sensed. */
  double computeParticleWeight(const Particle& p)
  {
    mr_srvs::GetPoseLikelihood srv;
    srv.request.pose.pose.position.x = p.pose.x;
    srv.request.pose.pose.position.y = p.pose.y;
    srv.request.pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.pose.theta);
    if (pose_likelihood_client_.call(srv))
    {
      return srv.response.likelihood;
    }
    else
    {
      ROS_ERROR("Service call to [get_pose_likelihood] failed, returning zero weight for the particle.");
      return 0.0;
    }
  }

private:

  ros::NodeHandle nh_;
  ros::ServiceClient pose_likelihood_client_;
  ros::Timer update_timer_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  ParticleFilter::UPtr particle_filter_;
  ParticleVisualizer::UPtr particle_visualizer_;

  std::string frame_id_;
  tf::Transform previous_pose_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "particle_filter");
  ParticleFilterNode pfn;
  ros::spin();
  return 0;
}

