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

/**
 * @brief Constructor for a Particle representing a robot pose hypothesis.
 * @param w Weight of the particle 
 * @param theta Yaw angle of the robot in radians
 * @param x X-coordinate in the map frame
 * @param y Y-coordinate in the map frame
 */
Particle::Particle(float w, float theta, float x, float y)
{
  this->w = w;
  this->theta = theta;
  this->x = x;
  this->y = y;
}

/**
 * @brief Convert a particle to a geometry_msgs/Pose message.
 * @return A Pose message representing the particle's position and orientation
 */
geometry_msgs::msg::Pose Particle::as_pose()
{
  geometry_msgs::msg::Pose pose = geometry_msgs::msg::Pose();
  pose.position.x = this->x;
  pose.position.y = this->y;
  pose.orientation = quaternion_from_euler(0, 0, this->theta);

  return pose;
}

/**
 * @brief Constructor for the ParticleFilter ROS2 node.
 * Initializes all parameters, subscribers, publishers, and timers needed for particle filter localization.
 */
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

  // resampling constants
  position_noise_scale = 0.05; 
  angle_noise_scale = 0.1;
  
  // Update particles with odom noise params
  odom_linear_noise = 0.1;   
  odom_angular_noise = 0.02; 
  
  // Update particles with laser params
  laser_range_noise = 0.1;   
  
  // Update robot pose from particles threshold
  best_particles_ratio = 0.035;
  max_normalized_deviation = 1000.0;
  
  // Noise distribution parameters
  noise_distribution_stddev = 0.33;  
  
  // Minimum particle weight for invalid measurements
  min_particle_weight = 0.001;

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

/**
 * @brief Publish the latest map to odom transform.
 * This function is called periodically by a timer to broadcast the transform
 * with a postdated timestamp to keep the tf tree valid.
 */
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

/**
 * @brief Main processing loop for the particle filter.
 * Checks for available scans, retrieves corresponding odometry, and performs
 * particle filter updates (odometry update, laser update, pose estimation, resampling)
 * when the robot has moved far enough. Publishes particles for visualization.
 */
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

/**
 * @brief Check if the robot has moved far enough to trigger a filter update.
 * @param new_odom_xy_theta Vector containing [x, y, theta] of the current odometry pose
 * @return true if the robot has moved beyond d_thresh or rotated beyond a_thresh, false otherwise
 */
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

/**
 * @brief Update the robot's estimated pose based on the particle cloud.
 * Sorts particles by weight and averages the top particles to compute the best
 * pose estimate. Updates the map to odom transform accordingly.
 */
void ParticleFilter::update_robot_pose()
{
  // first make sure that the particle weights are normalized
  normalize_particles();

  // david DONE
  // TODO: assign the latest pose into self.robot_pose as a
  // geometry_msgs.Pose object just to get started we will fix the robot's
  // pose to always be at the origin

  // sort by weight, best ones first
  std::sort(particle_cloud.begin(), particle_cloud.end(),
            [](const Particle& a, const Particle& b) { return a.w > b.w; });
  
  // only use top few particles for pose
  int thresh = static_cast<int>(std::round(best_particles_ratio * n_particles));
  thresh = std::max(1, thresh);
  
  double x = 0.0;
  double y = 0.0;
  double theta = 0.0;
  int counter = 0;

  // average the top particles
  for (int i = 0; i < thresh && i < (int)particle_cloud.size(); i++)
  {
    x += particle_cloud[i].x;
    y += particle_cloud[i].y;
    theta += particle_cloud[i].theta;
    counter++;
  }

  x /= counter;
  y /= counter;
  theta /= counter;

  // make pose from average
  geometry_msgs::msg::Pose robot_pose;
  robot_pose.position.x = x;
  robot_pose.position.y = y;
  robot_pose.position.z = 0.0;
  robot_pose.orientation = quaternion_from_euler(0, 0, theta);
  
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

/**
 * @brief Update all particles based on odometry motion.
 * Computes the change in pose since the last update and moves all particles
 * accordingly, adding Gaussian noise to model motion uncertainty.
 */
void ParticleFilter::update_particles_with_odom()
{
  auto new_odom_xy_theta =
      transform_helper_->convert_pose_to_xy_theta(odom_pose.value());

  // compute the change in x,y,theta since our last update

  float delta_x, delta_y, delta_theta;

  if (current_odom_xy_theta.size() >= 3)
  {
    delta_x = new_odom_xy_theta[0] - current_odom_xy_theta[0];
    delta_y = new_odom_xy_theta[1] - current_odom_xy_theta[1];
    delta_theta = new_odom_xy_theta[2] - current_odom_xy_theta[2];
    
    current_odom_xy_theta = new_odom_xy_theta;
  }
  else
  {
    current_odom_xy_theta = new_odom_xy_theta;
    return;
  }
  
  // Initialize random number generator for noise
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> noise_dist(0.0, noise_distribution_stddev);
  
  // move all particles by odom delta with motion model noise
  for (auto& particle : particle_cloud)
  {
    float noise_x = noise_dist(gen);
    float noise_y = noise_dist(gen);
    float noise_theta = noise_dist(gen);
    
    particle.x += delta_x + odom_linear_noise * noise_x;
    particle.y += delta_y + odom_linear_noise * noise_y;
    particle.theta += delta_theta + odom_angular_noise * noise_theta;
  }
}

/**
 * @brief Resample particles according to their weights.
 * Performs weighted sampling to focus particles on high-probability regions.
 * Adds small noise to each resampled particle to maintain diversity and
 * resets all weights to 1.0 after resampling.
 */
void ParticleFilter::resample_particles()
{
  normalize_particles();

  if (particle_cloud.empty()) {
    return;
  }

  // Get current number of particles just in case they don't equal n_particles
  const size_t total_particles = particle_cloud.size();

  // Random generators
  std::random_device random_device;
  std::mt19937 generator(random_device());

  // Sampling distrubutions (they follow a normal distribution) for the noise parameters
  std::normal_distribution<float> position_noise_dist(0.0, position_noise_scale);
  std::normal_distribution<float> angle_noise_dist(0.0, angle_noise_scale);

  // New particles we will return at the end
  std::vector<Particle> new_particle_cloud;
  new_particle_cloud.reserve(total_particles);

  // get a list of particle indices and weights to use in our weighted sampling later on.
  std::vector<unsigned int> particle_indices;
  std::vector<float> particle_weights;
  particle_indices.reserve(total_particles);
  particle_weights.reserve(total_particles);

  for (const auto& particle : particle_cloud) {
    particle_indices.push_back(particle_indices.size());
    particle_weights.push_back(particle.w);
  }

  // Perform a weighted sample of our particles to change our particles to have more of the particles that are of higher weight
  std::vector<unsigned int> sampled_indices = draw_random_sample(
      particle_indices, particle_weights, total_particles);

  // For each particle we sampled, add noise, ensure angle is proper, and reset the weight to 1.
  // we reset the weight to 1 so we do not get bogged by particle history
  for (unsigned int original_index : sampled_indices) {
    Particle resampled_particle = particle_cloud[original_index];

    // (x, y, theta) noise
    resampled_particle.x += position_noise_dist(generator);
    resampled_particle.y += position_noise_dist(generator);
    resampled_particle.theta += angle_noise_dist(generator);

    // Bound angle to (-pi, pi) just in case the noise changed that
    resampled_particle.theta = atan2(sin(resampled_particle.theta), cos(resampled_particle.theta));

    // reset weight
    resampled_particle.w = 1.0;

    // Finally add the adjusted particle to the new cloud 
    new_particle_cloud.push_back(resampled_particle);
  }
  
  // Replace particle cloud 
  particle_cloud = new_particle_cloud;

  normalize_particles();
}

/**
 * @brief Update particle weights based on laser scan data.
 * For each particle, projects laser readings into the map frame and compares
 * endpoints to obstacles in the occupancy field. Particles whose laser readings
 * match obstacles well receive higher weights.
 * @param r Vector of range measurements from the laser scan
 * @param theta Vector of angles (in robot frame) corresponding to each range
 */
void ParticleFilter::update_particles_with_laser(std::vector<float> r,
                                                 std::vector<float> theta)
{
  // khoi
  // weight particles based on laser scan
  
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<float> noise_dist(0.0, noise_distribution_stddev);
  
  for (auto& particle : particle_cloud)
  {
    float total_deviation = 0.0;
    int counter = 0;
    
    // check each laser reading
    for (size_t i = 0; i < r.size(); i++)
    {
      float ri = r[i];
      float ti = theta[i];
      
      if (!std::isfinite(ri))
      {
        continue;
      }
      
      // where does this laser hit in map frame
      float ang = particle.theta + ti;
      float ri_adj = ri + laser_range_noise * noise_dist(gen);
      float x_endpoint = particle.x + ri_adj * std::cos(ang);
      float y_endpoint = particle.y + ri_adj * std::sin(ang);
      
      // how far from an obstacle
      double closest = occupancy_field->get_closest_obstacle_distance(x_endpoint, y_endpoint);
      
      if (std::isfinite(closest))
      {
        total_deviation += closest;
        counter++;
      }
    }
    
    // closer to obstacles = higher weight
    if (counter > 0)
    {
      float normalized_dev = total_deviation / (counter * counter);
      normalized_dev = std::min(max_normalized_deviation, normalized_dev);
      particle.w = 1.0f / (normalized_dev * normalized_dev);
    }
    else
    {
      particle.w = min_particle_weight;
    }
  }
}

/**
 * @brief Check if a particle position is valid (within map bounds and on free space).
 * @param x X-coordinate in the map frame
 * @param y Y-coordinate in the map frame
 * @return true if the position is valid, false otherwise
 */
bool ParticleFilter::is_particle_valid(float x, float y)
{
  //  check if the particle is withing the map
  double dist = occupancy_field->get_closest_obstacle_distance(x, y);
  return std::isfinite(dist);
}

/**
 * @brief Callback for receiving initial pose estimates
 * Reinitializes the particle cloud around the given pose.
 * @param msg PoseWithCovarianceStamped message containing the initial pose estimate
 */
void ParticleFilter::update_initial_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
  auto xy_theta = transform_helper_->convert_pose_to_xy_theta(msg.pose.pose);
  initialize_particle_cloud(xy_theta);
}

/**
 * @brief Initialize the particle cloud with random particles.
 * Creates n_particles distributed uniformly across the map's obstacle bounding box.
 * If xy_theta is provided, it could be used for localized initialization (currently uses global).
 * @param xy_theta Optional initial pose [x, y, theta] to center particles around
 */
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

/**
 * @brief Normalize all particle weights to sum to 1.0.
 * This ensures the particle cloud represents a valid probability distribution.
 */
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

/**
 * @brief Publish the current particle cloud for visualization.
 * Creates a ParticleCloud message and publishes it on the /particle_cloud topic
 * so it can be visualized in RViz.
 * @param timestamp Timestamp to use for the published message
 */
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

/**
 * @brief Callback for receiving laser scan messages.
 * Stores the latest scan and timestamp, then triggers the run_loop to process it.
 * Only accepts new scans if the previous one has been processed.
 * @param msg LaserScan message from the lidar sensor
 */
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

/**
 * @brief Initialize helper objects (occupancy field and transform helper).
 * Must be called after node construction to set up dependencies that require
 * the node pointer.
 * @param nodePtr Shared pointer to this ParticleFilter node
 */
void ParticleFilter::setup_helpers(std::shared_ptr<ParticleFilter> nodePtr)
{
  occupancy_field = std::make_shared<OccupancyField>(OccupancyField(nodePtr));
  std::cout << "done generating occupancy field" << std::endl;
  transform_helper_ = std::make_shared<TFHelper>(TFHelper(nodePtr));
  std::cout << "done generating TFHelper" << std::endl;
}

/**
 * @brief Main entry point for the particle filter node.
 * Initializes ROS2, creates the ParticleFilter node, sets up helpers,
 * and starts the executor to process callbacks.
 * @param argc Number of command-line arguments
 * @param argv Array of command-line argument strings
 * @return Exit status code
 */
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
