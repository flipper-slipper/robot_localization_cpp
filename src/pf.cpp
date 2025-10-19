#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <random>
#include <rclcpp/time.hpp>
#include <string>
#include <tuple>

#include "angle_helpers.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "helper_functions.hpp"
#include "occupancy_field.hpp"
#include "pf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
using std::placeholders::_1;

Particle::Particle(float w, float theta, float x, float y)
{
  this->w = w;
  this->theta = theta;
  this->x = x;
  this->y = y;
}

/**
 * A helper function to convert a particle to a geometry_msgs/Pose message
 */
geometry_msgs::msg::Pose Particle::as_pose()
{
  geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
  pose.position.x = this->x;
  pose.position.y = this->y;
  pose.orientation = quaternion_from_euler(0, 0, this->theta);

  return pose;
}

ParticleFilter::ParticleFilter() : Node("pf")
{
  base_frame = "base_footprint"; // the frame of the robot base
  map_frame = "map";             // the name of the map coordinate frame
  odom_frame = "odom";           // the name of the odometry coordinate frame
  scan_topic = "scan";           // the topic where we will get laser scans from

  n_particles = 300; // the number of particles to use

  d_thresh = 0.2; // the amount of linear movement before performing an update
  a_thresh =
      M_PI / 6; // the amount of angular movement before performing an update

  // TODO: define additional constants if needed

  // pose_listener responds to selection of a new approximate robot
  // location (for instance using rviz)
  auto sub1_opt = rclcpp::SubscriptionOptions();
  sub1_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  initial_pose_subscriber = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", 10,
      std::bind(&ParticleFilter::update_initial_pose, this, _1),
      sub1_opt);

  // publish the current particle cloud.  This enables viewing particles
  // in rviz.
  particle_pub = this->create_publisher<nav2_msgs::msg::ParticleCloud>(
      "particle_cloud", 10);

  auto sub2_opt = rclcpp::SubscriptionOptions();
  sub2_opt.callback_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  // laser_subscriber listens for data from the lidar
  laserscan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic,
      10,
      std::bind(&ParticleFilter::scan_received, this, _1),
      sub2_opt);

  // this is used to keep track of the timestamps coming from bag files
  // knowing this information helps us set the timestamp of our map ->
  // odom transform correctly
  last_scan_timestamp.reset();
  // this is the current scan that our run_loop should process
  scan_to_process.reset();

  timer = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ParticleFilter::pub_latest_transform, this));
}

void ParticleFilter::pub_latest_transform()
{
  if (last_scan_timestamp.has_value())
  {
    rclcpp::Time last_scan_time(last_scan_timestamp.value());
    rclcpp::Duration offset(0, 100000000);
    auto postdated_timestamp = last_scan_time + offset;
    transform_helper_->send_last_map_to_odom_transform(map_frame, odom_frame,
                                                       postdated_timestamp);
  }
}

void ParticleFilter::run_loop()
{
  if (!scan_to_process.has_value())
  {
    return;
  }
  auto msg = scan_to_process.value();
  std::tuple<std::optional<geometry_msgs::msg::Pose>,
             std::optional<std::chrono::nanoseconds>>
      matching_odom_pose = transform_helper_->get_matching_odom_pose(
          odom_frame, base_frame, msg.header.stamp);
  auto new_pose = std::get<0>(matching_odom_pose);
  auto dt = std::get<1>(matching_odom_pose);
  if (!new_pose.has_value())
  {
    // we were unable to get the pose of the robot corresponding to the
    // scan timestamp
    if (dt.has_value() && dt.value() < std::chrono::nanoseconds(0))
    {
      //  we will never get this transform, since it is before our
      //  oldest one
      scan_to_process.reset();
    }
    return;
  }
  auto polar_coord = transform_helper_->convert_scan_to_polar_in_robot_frame(
      msg, base_frame);
  auto r = std::get<0>(polar_coord);
  auto theta = std::get<1>(polar_coord);
  // clear the current scan so that we can process the next one
  scan_to_process.reset();
  odom_pose = new_pose;
  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
  if (current_odom_xy_theta.size() == 0)
  {
    current_odom_xy_theta = new_odom_xy_theta;
  }
  else if (particle_cloud.size() == 0)
  {
    // now that we have all of the necessary transforms we can update
    // the particle cloud
    initialize_particle_cloud();
  }
  else if (moved_far_enough_to_update(new_odom_xy_theta))
  {
    // we have moved far enough to do an update!
    update_particles_with_odom(); // update based on odometry
    update_particles_with_laser(r,
                                theta); // update based on laser scan
    update_robot_pose();                // update robot's pose based on particles
    resample_particles();               // resample particles to focus on areas of
                                        // high density
  }

  // publish particles (so things like rviz can see them)
  publish_particles(msg.header.stamp);
}

bool ParticleFilter::moved_far_enough_to_update(std::vector<float> new_odom_xy_theta)
{
  return abs(new_odom_xy_theta[0] - current_odom_xy_theta[0] > d_thresh ||
             abs(new_odom_xy_theta[1] - current_odom_xy_theta[1]) >
                 d_thresh ||
             abs(new_odom_xy_theta[2] - current_odom_xy_theta[2]) > a_thresh);
}

void ParticleFilter::update_robot_pose()
{
  // first make sure that the particle weights are normalized
  normalize_particles();

  // david
  // TODO: assign the latest pose into self.robot_pose as a
  // geometry_msgs.Pose object just to get started we will fix the robot's
  // pose to always be at the origin
  geometry_msgs::msg::Pose robot_pose;
  if (odom_pose.has_value())
  {
    transform_helper_->fix_map_to_odom_transform(robot_pose,
                                                 odom_pose.value());
  }
  else
  {
    // david
    // TODO: print something
  }
}

void ParticleFilter::update_particles_with_odom()
{
  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());

  // compute the change in x,y,theta since our last update
  if (current_odom_xy_theta.size() >= 3)
  {
    auto old_odom_xy_theta = current_odom_xy_theta;
    auto delta_x = new_odom_xy_theta[0] - current_odom_xy_theta[0];
    auto delta_y = new_odom_xy_theta[1] - current_odom_xy_theta[1];
    auto delta_theta = new_odom_xy_theta[2] - current_odom_xy_theta[2];
  }
  else
  {
    current_odom_xy_theta = new_odom_xy_theta;
    return;
  }

  // khoi
  // Update each particle with the odometry motion model
  
  // Noise parameters - Set to 0 for no noise, or small values for robustness
  // Noise helps particles explore and recover from errors
  float linear_noise = 0.02;   // meters (can set to 0.0 for testing)
  float angular_noise = 0.02;  // radians (can set to 0.0 for testing)
  
  // Random number generators (only used if noise > 0)
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> noise_dist(0.0, 1.0);
  
  // Update each particle
  for (auto& particle : particle_cloud)
  {
    // Transform the odometry delta from robot frame to map frame
    // This rotation accounts for each particle's different heading
    float dx_map = delta_x * cos(particle.theta) - delta_y * sin(particle.theta);
    float dy_map = delta_x * sin(particle.theta) + delta_y * cos(particle.theta);
    
    // Apply motion to particle (with optional noise)
    particle.x += dx_map + noise_dist(gen) * linear_noise;
    particle.y += dy_map + noise_dist(gen) * linear_noise;
    particle.theta += delta_theta + noise_dist(gen) * angular_noise;
    
    // Keep angle in range [-pi, pi]
    particle.theta = atan2(sin(particle.theta), cos(particle.theta));
  }
  
  // Update the current odometry pose for next iteration
  current_odom_xy_theta = new_odom_xy_theta;
}

void ParticleFilter::resample_particles()
{
  // make sure the distribution is normalized
  normalize_particles();
  // khoi
  // Resample particles based on their weights (importance resampling)
  
  // Check if we have particles to resample
  if (particle_cloud.empty())
  {
    return;
  }
  
  // Create choice vector (indices of current particles)
  std::vector<unsigned int> choices;
  for (unsigned int i = 0; i < particle_cloud.size(); i++)
  {
    choices.push_back(i);
  }
  
  // Extract weights as probabilities
  std::vector<float> probabilities;
  for (const auto& particle : particle_cloud)
  {
    probabilities.push_back(particle.w);
  }
  
  // Draw random samples based on particle weights
  std::vector<unsigned int> sampled_indices = draw_random_sample(
      choices, probabilities, n_particles);
  
  // Create new particle cloud from sampled indices
  std::vector<Particle> new_particle_cloud;
  for (unsigned int idx : sampled_indices)
  {
    // Copy the selected particle
    Particle new_particle = particle_cloud[idx];
    // Reset weight to uniform (will be normalized later)
    new_particle.w = 1.0;
    new_particle_cloud.push_back(new_particle);
  }
  
  // Replace old particle cloud with resampled one
  particle_cloud = new_particle_cloud;
  
  // Normalize weights again
  normalize_particles();
}

void ParticleFilter::update_particles_with_laser(std::vector<float> r,
                                                 std::vector<float> theta)
{
  // khoi
  
  // Sensor model parameters (can be tuned)
  float sigma_hit = 0.1;  // Standard deviation for measurement noise (meters)
  float z_hit = 0.95;     // Weight for correct measurements
  float z_rand = 0.05;    // Weight for random measurements
  float max_range = 10.0; // Maximum laser range for normalization
  
  // Downsample laser readings for efficiency (use every Nth reading)
  int step_size = 5;
  
  // Update weight for each particle
  for (auto& particle : particle_cloud)
  {
    float log_likelihood = 0.0;  // Use log probabilities to avoid underflow
    int num_valid_readings = 0;
    
    // Process laser scan readings
    for (size_t i = 0; i < r.size(); i += step_size)
    {
      // Skip invalid readings (too close, too far, or NaN)
      if (std::isnan(r[i]) || std::isinf(r[i]) || r[i] <= 0.1 || r[i] >= max_range)
      {
        continue;
      }
      
      // Transform laser endpoint to map frame using particle's pose
      float x_endpoint = particle.x + r[i] * cos(particle.theta + theta[i]);
      float y_endpoint = particle.y + r[i] * sin(particle.theta + theta[i]);
      
      // Get distance from endpoint to closest obstacle in the map
      double dist_to_obstacle = occupancy_field->get_closest_obstacle_distance(x_endpoint, y_endpoint);
      
      // Skip if out of map bounds
      if (dist_to_obstacle >= UINT16_MAX)
      {
        continue;
      }
      
      // Compute probability using mixture model:
      // p(z) = z_hit * Gaussian(dist, 0, sigma) + z_rand * uniform
      float gaussian_prob = exp(-0.5 * pow(dist_to_obstacle / sigma_hit, 2)) / 
                           (sigma_hit * sqrt(2 * M_PI));
      float uniform_prob = 1.0 / max_range;
      float prob = z_hit * gaussian_prob + z_rand * uniform_prob;
      
      // Accumulate log likelihood
      log_likelihood += log(prob + 1e-10);  // Add small epsilon to avoid log(0)
      num_valid_readings++;
    }
    
    // Update particle weight
    if (num_valid_readings > 0)
    {
      // Convert log likelihood back to probability and update weight
      particle.w *= exp(log_likelihood);
    }
    else
    {
      // If no valid readings, keep current weight but slightly reduce it
      particle.w *= 0.5;
    }
  }
}

void ParticleFilter::update_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  auto xy_theta = transform_helper_->convert_pose_to_xy_theta(msg.pose.pose);
  initialize_particle_cloud(xy_theta);
}

void ParticleFilter::initialize_particle_cloud(
    std::optional<std::vector<float>> xy_theta)
{
  if (!xy_theta.has_value())
  {
    xy_theta = transform_helper_->convert_pose_to_xy_theta(odom_pose.value());
  }
  // david
  // TODO: create particles

  normalize_particles();
  update_robot_pose();
}

void ParticleFilter::normalize_particles()
{
  david
  // TODO: implement this
}

void ParticleFilter::publish_particles(rclcpp::Time timestamp)
{
  nav2_msgs::msg::ParticleCloud msg;
  msg.header.stamp = timestamp;
  msg.header.frame_id = map_frame;

  for (unsigned int i = 0; i < particle_cloud.size(); i++)
  {
    nav2_msgs::msg::Particle converted;
    converted.weight = particle_cloud[i].w;
    converted.pose = particle_cloud[i].as_pose();
    msg.particles.push_back(converted);
  }

  // actually send the message so that we can view it in rviz
  particle_pub->publish(msg);
}

void ParticleFilter::scan_received(sensor_msgs::msg::LaserScan msg)
{
  last_scan_timestamp = msg.header.stamp;
  /**
   * we throw away scans until we are done processing the previous scan
   * self.scan_to_process is set to None in the run_loop
   */
  if (!scan_to_process.has_value())
  {
    scan_to_process = msg;
  }
  // call run_loop to see if we need to update our filter, this will prevent more scans from coming in
  run_loop();
}

void ParticleFilter::setup_helpers(std::shared_ptr<ParticleFilter> nodePtr)
{
  occupancy_field = std::make_shared<OccupancyField>(OccupancyField(nodePtr));
  std::cout << "done generating occupancy field" << std::endl;
  transform_helper_ = std::make_shared<TFHelper>(TFHelper(nodePtr));
  std::cout << "done generating TFHelper" << std::endl;
}

int main(int argc, char **argv)
{
  // this is useful to give time for the map server to get ready...
  // TODO: fix in some other way
  sleep(5);
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<ParticleFilter>();
  node->setup_helpers(node);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
