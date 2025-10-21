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

  n_particles = 800; // the number of particles to use

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
  // Guard against malformed input
  if (current_odom_xy_theta.size() < 3 || new_odom_xy_theta.size() < 3)
  {
    return false;
  }

  // Compare absolute differences against thresholds
  bool moved_x = std::fabs(new_odom_xy_theta[0] - current_odom_xy_theta[0]) > d_thresh;
  bool moved_y = std::fabs(new_odom_xy_theta[1] - current_odom_xy_theta[1]) > d_thresh;
  bool moved_a = std::fabs(new_odom_xy_theta[2] - current_odom_xy_theta[2]) > a_thresh;

  return moved_x || moved_y || moved_a;
}

void ParticleFilter::update_robot_pose()
{
  // first make sure that the particle weights are normalized
  normalize_particles();

  // david DONE
  // TODO: assign the latest pose into self.robot_pose as a
  // geometry_msgs.Pose object just to get started we will fix the robot's
  // pose to always be at the origin

  // Sort particles by weight for the loop below
  std::sort(particle_cloud.begin(), particle_cloud.end(),
            [](const Particle& a, const Particle& b) { return a.w > b.w; });
  
  // use top 5% of particles to estimate Pose 
  int thresh = static_cast<int>(0.05 * n_particles);
  thresh = std::max(1, thresh);  // At least 1 particle
  
  double x_sum = 0.0;
  double y_sum = 0.0;
  
  // Use circular mean for angles
  double sin_sum = 0.0;
  double cos_sum = 0.0;

  // Calculate the average of only the top particles
  for (int i = 0; i < thresh; i++)
  {
    x_sum += particle_cloud[i].x;
    y_sum += particle_cloud[i].y;
    sin_sum += std::sin(particle_cloud[i].theta);
    cos_sum += std::cos(particle_cloud[i].theta);
  }

  // Create the robot pose from the averaged top particle positions
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = x_sum / thresh;
  robot_pose.position.y = y_sum / thresh;
  robot_pose.position.z = 0.0;
  
  // Compute circular mean of angle
  double theta_avg = std::atan2(sin_sum, cos_sum);
  robot_pose.orientation = quaternion_from_euler(0, 0, theta_avg);
  
  if (odom_pose.has_value())
  {
    transform_helper_->fix_map_to_odom_transform(robot_pose,
                                                 odom_pose.value());
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "No odometry pose available for transform update");
  }
}

void ParticleFilter::update_particles_with_odom()
{
  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());

  // compute the change in x,y,theta since our last update

  float delta_x, delta_y, delta_theta;

  if (current_odom_xy_theta.size() >= 3)
  {
    auto old_odom_xy_theta = current_odom_xy_theta;
    delta_x = new_odom_xy_theta[0] - current_odom_xy_theta[0];
    delta_y = new_odom_xy_theta[1] - current_odom_xy_theta[1];
    delta_theta = new_odom_xy_theta[2] - current_odom_xy_theta[2];
    
    // Convert global delta to robot's local frame at old pose
    float old_theta = old_odom_xy_theta[2];
    float dx_robot = delta_x * cos(old_theta) + delta_y * sin(old_theta);
    float dy_robot = -delta_x * sin(old_theta) + delta_y * cos(old_theta);
    
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
      // Transform the robot-frame delta to map frame using particle's heading
      float dx_map = dx_robot * cos(particle.theta) - dy_robot * sin(particle.theta);
      float dy_map = dx_robot * sin(particle.theta) + dy_robot * cos(particle.theta);
      
      // Apply motion to particle (with optional noise)
      particle.x += dx_map + noise_dist(gen) * linear_noise;
      particle.y += dy_map + noise_dist(gen) * linear_noise;
      particle.theta += delta_theta + noise_dist(gen) * angular_noise;
      
      // Keep angle in range [-pi, pi]
      particle.theta = atan2(sin(particle.theta), cos(particle.theta));
    }
  }
  else
  {
    current_odom_xy_theta = new_odom_xy_theta;
    return;
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
  
  // Only resample 3% of particles
  int proportion_to_resample = static_cast<int>(0.3 * n_particles);
  int random_particles = n_particles - proportion_to_resample;
  
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
  
  // Draw random samples for only a proportion of the particles
  std::vector<unsigned int> sampled_indices = draw_random_sample(
      choices, probabilities, proportion_to_resample);
  
  // Add noise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> pos_noise(0.0, 0.5); 
  std::normal_distribution<float> angle_noise(0.0, 0.1);
  
  std::vector<Particle> new_particle_cloud;
  
  // Add resampled particles with noise
  for (unsigned int idx : sampled_indices)
  {
    Particle new_particle = particle_cloud[idx];
    
    // Add noise to maintain particle diversity
    new_particle.x += pos_noise(gen);
    new_particle.y += pos_noise(gen);
    new_particle.theta += angle_noise(gen);
    
    // Keep angle in range
    new_particle.theta = atan2(sin(new_particle.theta), cos(new_particle.theta));
    
    // Reset weight to uniform (will be normalized later)
    new_particle.w = 1.0;
    new_particle_cloud.push_back(new_particle);
  }
  
  // Add random particles
  auto bbox = occupancy_field->get_obstacle_bounding_box();
  double x_min = bbox[0];
  double x_max = bbox[1];
  double y_min = bbox[2];
  double y_max = bbox[3];
  
  std::uniform_real_distribution<float> x_dist(x_min, x_max);
  std::uniform_real_distribution<float> y_dist(y_min, y_max);
  std::uniform_real_distribution<float> theta_dist(-M_PI, M_PI);
  
  for (int i = 0; i < random_particles; i++)
  {
    float x_rand = x_dist(gen);
    float y_rand = y_dist(gen);
    float theta_rand = theta_dist(gen);
    
    Particle p = Particle(1.0, theta_rand, x_rand, y_rand);
    new_particle_cloud.push_back(p);
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
  float sigma_hit = 0.5;  // Standard deviation for measurement noise (meters) - higher = more tolerant
  float z_hit = 0.8;      // Weight for correct measurements
  float z_rand = 0.2;     // Weight for random measurements - higher = less discriminating
  float max_range = 10.0; // Maximum laser range for normalization
  
  // Downsample laser readings for efficiency (use every Nth reading)
  int step_size = 10;  // Increased from 5 to use fewer readings
  
  // Track max log likelihood for numerical stability
  float max_log_likelihood = -std::numeric_limits<float>::infinity();
  std::vector<float> log_likelihoods(particle_cloud.size());
  
  // Compute log likelihoods for all particles
  for (size_t p = 0; p < particle_cloud.size(); p++)
  {
    auto& particle = particle_cloud[p];
    float log_likelihood = 0.0;
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
      float x_endpoint = particle.x + r[i] * std::cos(particle.theta + theta[i]);
      float y_endpoint = particle.y + r[i] * std::sin(particle.theta + theta[i]);
      
      // Get distance from endpoint to closest obstacle in the map
      double dist_to_obstacle = occupancy_field->get_closest_obstacle_distance(x_endpoint, y_endpoint);
      
      // Skip if out of map bounds
      if (dist_to_obstacle >= UINT16_MAX)
      {
        continue;
      }
      
      // Compute probability using mixture model:
      // p(z) = z_hit * Gaussian(dist, 0, sigma) + z_rand * uniform
      float gaussian_prob = std::exp(-0.5f * std::pow(dist_to_obstacle / sigma_hit, 2)) / 
                           (sigma_hit * std::sqrt(2 * M_PI));
      float uniform_prob = 1.0f / max_range;
      float prob = z_hit * gaussian_prob + z_rand * uniform_prob;
      
      // Accumulate log likelihood
      log_likelihood += std::log(std::max(prob, 1e-10f));
      num_valid_readings++;
    }
    
    // Store log likelihood (or penalty if no valid readings)
    if (num_valid_readings > 0)
    {
      log_likelihoods[p] = log_likelihood;
    }
    else
    {
      log_likelihoods[p] = std::log(0.1f);  // Small penalty
    }
    
    max_log_likelihood = std::max(max_log_likelihood, log_likelihoods[p]);
  }
  
  // Update weights using log-sum-exp trick for numerical stability
  for (size_t p = 0; p < particle_cloud.size(); p++)
  {
    // Subtract max before exp to prevent overflow
    particle_cloud[p].w *= std::exp(log_likelihoods[p] - max_log_likelihood);
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

  // david DONE
  // TODO: create particles
  
  // Clear existing particles
  particle_cloud.clear();
  
  // Get the bounding box of obstacles from the occupancy field
  auto bbox = occupancy_field->get_obstacle_bounding_box();
  double x_min = bbox[0];
  double x_max = bbox[1];
  double y_min = bbox[2];
  double y_max = bbox[3];
  
  // Create random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<float> x_dist(x_min, x_max);
  std::uniform_real_distribution<float> y_dist(y_min, y_max);
  std::uniform_real_distribution<float> theta_dist(-M_PI, M_PI);
  
  // Generate n_particles random particles within the map bounds
  for (int i = 0; i < n_particles; i++)
  {
    float x_rand = x_dist(gen);
    float y_rand = y_dist(gen);
    float theta_rand = theta_dist(gen);
    
    // Create particle with equal initial weight (will be normalized)
    Particle p = Particle(1.0, theta_rand, x_rand, y_rand);
    particle_cloud.push_back(p);
  }

  normalize_particles();
  update_robot_pose();
}

void ParticleFilter::normalize_particles()
{

  // david DONE
  // TODO: implement this
  // Calculate sum of all weights
  float weight_sum = 0.0;
  for (size_t i = 0; i < particle_cloud.size(); i++)
  {
    weight_sum += particle_cloud[i].w;
  }
  
  // Normalize each weight by dividing by the sum
  // This ensures all weights sum to 1.0 (probability distribution)
  if (weight_sum > 0.0)
  {
    for (size_t i = 0; i < particle_cloud.size(); i++)
    {
      particle_cloud[i].w /= weight_sum;
    }
  }
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
