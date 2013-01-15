#ifndef MOTION_MODEL_H
#define MOTION_MODEL_H

#include <random>

#include "pose.h"

/** This class represents a motion model for omnidirectional robot and could be
  * used to sample the possible pose given the starting pose and the commanded
  * robot's motion.
  *
  * The two parameters of the class is standard deviations of translational and
  * rotational components of the motion.
  *
  * The motion is decomposed into two translations alond the x axis of the
  * robot (forward), and along the y axis of the robot (lateral), and one
  * rotation.
  *
  * Usage:
  *
  * @code
  *   // Create motion model with 0.02 and 0.01 stddev
  *   MotionModel motion_model(0.02, 0.01);
  *   // Set the commanded robot's motion
  *   motion_model.setMotion(0.5, 0.1, 0.1);
  *   // Sample the possible pose given the starting pose
  *   // Note that it could be repeated multiple times for the same starting
  *   // pose of for different starting poses
  *   Pose new_pose = motion_model.sample(pose);
  * @code
  *
  * */
class MotionModel
{

public:

  MotionModel(double sigma_translation, double sigma_rotation)
  : forward_(0.0)
  , lateral_(0.0)
  , rotation_(0.0)
  , generator_(device_())
  , distribution_trans_(0, sigma_translation)
  , distribution_rot_(0, sigma_rotation)
  {
  }

  /** Set the commanded robot's motion. */
  void setMotion(double forward, double lateral, double rotation)
  {
    forward_ = forward;
    lateral_ = lateral;
    rotation_ = rotation;
  }

  /** Sample a possible pose resulting from the commanded robot's motion, if
    * the robot was in given pose. */
  Pose sample(const Pose& pose)
  {
    //============================== YOUR CODE HERE ============================
    // Instructions: given the starting pose, compute the new pose according to
    //               the motion model. Note that both input and output pose are
    //               in world coordinate frame, but the motion parameters are
    //               in robot's reference frame.
    //
    // Hint: there are two member fields that represent translational and
    //       rotational error distributions. For example, to get a random
    //       translational error use:
    //
    //           double error = distribution_trans_(generator_);
    //
    Pose estimated_pose;

    // use motion (set in setMotion) to calculate the error values
    double error_plus_x = forward_ + distribution_trans_(generator_);
    double error_plus_y = lateral_ + distribution_trans_(generator_);
    double error_plus_theta = rotation_ + distribution_rot_(generator_);

    // The omnidrive motion model let's us rotate during rotation so the rotational 
    // error can influence the translational error in a lot of different ways 
    // (fully in the beginning of the motion, at the end, or everything in between)
    // Thus we are overestimating the influence of the rotational error. We think this
    // is not a bad thing. It will increase uncertainty about the pose but 
    // at least will not underestimate the influence of the rotational error.
    estimated_pose.x = pose.x + error_plus_x * cos(pose.theta + error_plus_theta);
    estimated_pose.y = pose.y + error_plus_x * sin(pose.theta + error_plus_theta);
    estimated_pose.theta = pose.theta + error_plus_theta;
    estimated_pose.theta = normalizeAngle(estimated_pose.theta);

    return estimated_pose;
    //==========================================================================
  }

private:

  inline double normalizeAngle(double angle)
  {
    return atan2(sin(angle), cos(angle));
  }

  double forward_;
  double lateral_;
  double rotation_;
  std::random_device device_;
  std::mt19937 generator_;
  std::normal_distribution<double> distribution_trans_;
  std::normal_distribution<double> distribution_rot_;

};

#endif /* MOTION_MODEL_H */

