#define _USE_MATH_DEFINES
#include <cmath> 
#include <string> 
#include <iostream>

#include <ros/ros.h>
#include <ros/console.h>
#include <ros/assert.h>
#include <tf/transform_listener.h>

#include <mr_msgs/Ranges.h>
#include <mr_srvs/GetPoseLikelihood.h>
#include <mr_srvs/GetNearestOccupiedPointOnBeam.h>
#include <mr_srvs/SwitchRanger.h>

#define PI 3.1415926f 


/******************************************************************************************************************
* TODO
* - Use the parameter server!
* 
* EXPLANATION
* Parameters in math formulas: 
* sigma_      :   The assumed standard deviation of the distance error between the simulated and actual distance reading of 
*                 a sonar sensor. Increasing it will allow for larger differences between actual and simulated readings.
*                 This parameter can be used to tweak the particle filter to become more tolerant / less accurate and vice versa.
*                
* error       :   The difference between the measured sonar reading and the simulated reading. Larger distances will produce
*                 a lower output probability (weight)
* 
* threshold   :   Sets the threshold occupancy probability of the occupancy grid, which is now either 0 or 100 but in the future will be 
*                 an actual probability
* 
******************************************************************************************************************/

class PoseLikelihoodServerNode
{

public:

  /******************************************************************************************************************
   * Constructor, taking a ros::ServiceClient pointing to the getNearestOccupiedPointOnBeam service
   *
   ******************************************************************************************************************/
  PoseLikelihoodServerNode (ros::ServiceClient & beam_client) { 
    // define (default) class values
    this->static_tfs_ready_ = false;
    this->beam_client_ = beam_client;
    this->sigma_ = 0.2;
    this->max_range_ = 5.0;
  }



  /******************************************************************************************************************
   * Calculates the likelihood of a requested pose comparing the actual sonar values of the robot to the simulated
   * values belonging to the requested pose. 
   * @param &req        :   the service request
   * @param &res[out]   :   the service response
   *
   ******************************************************************************************************************/
  bool calculatePoseLikelihood (mr_srvs::GetPoseLikelihood::Request &req, mr_srvs::GetPoseLikelihood::Response &res) {
    
    double total_likelihood = 0.0;
    int allowed_bad_matches = 2;                                                // Allow 3 sonars with a match > 2 * sigma
    if (this->static_tfs_ready_ == false) {                                     // Get static TFs (sonar to robot base_link) once
      for (int i = 0; i < 16; i++){                                             // For all sonars, create a name string and get TF
        try{
          this->tf_listener_.lookupTransform("/base_link", "/sonar_pioneer_" + 
            std::to_string(i)+"_link", ros::Time(0), transforms_[i]);
        } catch (tf::TransformException& ex){
          ROS_ERROR("Transform lookup failed: %s", ex.what());
        }    
      }  
      this->static_tfs_ready_ = true;
    }

    geometry_msgs::Pose pose_msg = req.pose.pose;                               // Convert requested pose message to tf::Transform object
    tf::Transform pose_tf;
    tf::poseMsgToTF(pose_msg, pose_tf); 

    mr_srvs::GetNearestOccupiedPointOnBeam srv;                                 // Create service message
    for (int i = 0; i < 16; i++) {
      tf::Transform beam_tf = pose_tf*transforms_[i];                           // Calculate virtual pose of all sensors using Transform multiplication
      geometry_msgs::Pose2D beam;                                               // Create geometry_msgs::Pose2D object containing the sonar pose
      beam.x = beam_tf.getOrigin().getX();
      beam.y = beam_tf.getOrigin().getY();
      beam.theta = tf::getYaw(beam_tf.getRotation());
      srv.request.beams.push_back(beam);
    }
    srv.request.threshold = 25;                                                 // Set the service request to 15
    this->beam_client_.call(srv);                                               // Call the service

    double error = 0.0;                                                 
    for (int i = 0; i < 16; i++){                                                 
      clamp<double>( srv.response.distances.at(i), 0.0, this->max_range_ );     // Clamp getNearestOccupiedPointOnBeam service response (beam distances)
      error = fabs(srv.response.distances.at(i) - ranges_[i] );                 // Calculate absolute distance between real and simulated values

      if ( error < ( 2.0*this->sigma_ ) || allowed_bad_matches <= 0  ){         // IF the abs distance is small enough or there are just too many bad matches
                                                                                // calculate the likelihood weight, clamp it and add it to the total likelihood
        double likelihood_weight = (1.0 / (this->sigma_*sqrt(2.0*M_PI))) * (exp( ( -1.0 * pow(error,2.0) )/( 2.0*pow(this->sigma_,2.0 )) ));
        clamp<double>( likelihood_weight, 0.0, 1.0 );
        total_likelihood += likelihood_weight;
      } else if (allowed_bad_matches > 0){                                      // ELSE reduce no. of bad matches and ignore value, allowing it.
        allowed_bad_matches -= 1;
      } 
    }
    
    res.likelihood = total_likelihood / 16.0;                                   // write likelihood (/16) into the service response
    ROS_INFO("total_likelihood: %lf", res.likelihood);
    return true;
  }

  /**   Sonar callback is triggered every time the Stage node publishes new data
  *     to the sonar topic. */
  void sonarCallback(const mr_msgs::Ranges::ConstPtr& msg)
  {
    this->max_range_ = msg->ranges[0].max_range;
    for (int i = 0; i < 16; i++) this->ranges_[i] = msg->ranges[i].range;
  }

private:

  /** Helper function to clamp a variable to a given range. */
  template<typename T>
  static void clamp(T& value, T min, T max)
  {
    if (value < min)
      value = min;
    else if (value > max)
      value = max;
  }

  double ranges_[16];
  tf::StampedTransform transforms_[16];
  tf::TransformListener tf_listener_;  
  bool static_tfs_ready_;
  ros::ServiceClient beam_client_;
  double sigma_;
  double max_range_;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_likelihood_server");
  ros::NodeHandle nh;
  ros::Subscriber sonar_subscriber;
  // Wait until SwitchRanger service (and hence stage node) becomes available.
  ROS_INFO("Waiting for the /switch_ranger service to be advertised...");
  ros::ServiceClient switch_ranger_client = nh.serviceClient<mr_srvs::SwitchRanger>("/switch_ranger");
  switch_ranger_client.waitForExistence();
  // Make sure that the pioneer sonars are available and enable them.
  mr_srvs::SwitchRanger srv;
  srv.request.name = "sonar_pioneer";
  srv.request.state = true;
  if (switch_ranger_client.call(srv)){
    ROS_INFO("Enabled pioneer sonars.");
  }
  else{
    ROS_ERROR("Pioneer sonars are not available, shutting down.");
    return 1;
  }
  // connect to GetNearestOccupiedPointOnBeam service
  ros::ServiceClient beam_client = nh.serviceClient<mr_srvs::GetNearestOccupiedPointOnBeam>("occupancy_query_server/get_nearest_occupied_point_on_beam");
  ROS_INFO("Waiting for the /occupancy_query_server/get_nearest_occupied_point_on_beam service to be advertised...");
  beam_client.waitForExistence();
  ROS_INFO("mr_srvs::GetNearestOccupiedPointOnBeam is alive!");
  // instantiate PoseLikelihoodServerNode and throw it a beam_client
  PoseLikelihoodServerNode plsn(beam_client);
  // subscribe to sonar topic
  sonar_subscriber = nh.subscribe("/sonar_pioneer", 100, &PoseLikelihoodServerNode::sonarCallback, &plsn);
  ROS_INFO("Subscribed to /sonar_pioneer topic");
  // advertise get_pose_likelihood service at the right node
  ros::ServiceServer service = nh.advertiseService("pose_likelihood_server/get_pose_likelihood", &PoseLikelihoodServerNode::calculatePoseLikelihood, &plsn);
  ROS_INFO("Ready to calculate likelihood of pose");  
  ros::spin();
  return 0;
}
