#include "pf_node.hpp"

#define ANGLE_STEP (0.00435928675336234f) // ((double) fov) / (total_rays + 1) * M_PI / 180.0f

using std::placeholders::_1;

/** constructor */
ParticleFilterNode::ParticleFilterNode()
{
    node = std::make_shared<rclcpp::Node>("particle_node");
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(node);

    node->declare_parameter("conf_path");
    conf_path = node->get_parameter("conf_path").as_string();

    // Subscribers
    map_sub = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 1, std::bind(&ParticleFilterNode::mapCb, this, _1));

    click_sub = node->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 1, std::bind(&ParticleFilterNode::clicked_poseCb, this, _1));

    odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 1, std::bind(&ParticleFilterNode::OdomCb, this, _1));

    lidar_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 1, std::bind(&ParticleFilterNode::lidarCb, this, _1));

    // Publisher
    pose_pub = node->create_publisher<nav_msgs::msg::Odometry>("/thundershot/pf/position", 1);
    particles_pub =
      node->create_publisher<geometry_msgs::msg::PoseArray>("/thundershot/pf/particles", 1);
    lidar_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/thundershot/pf/scan", 1);

    std::cout << "resize" << std::endl;

    initial_pose.resize(3);
    scan.resize(N_RAYS_DS);
    poses.resize(3);
    rays_angles.resize(N_RAYS_DS);

    std::cout << "spinno" << std::endl;
    rclcpp::spin(node);
}

/**
    map callback relative to map_sub subsciber in main
*/
void ParticleFilterNode::mapCb(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg)
{
    particleFilter.init(conf_path);
    std::cout << "[INFO]  init DONE" << std::endl;
    particleFilter.getOmap(map_msg);
    std::cout << "[INFO]  getOmap DONE" << std::endl;
    particleFilter.precompute_sensor_model();
    std::cout << "[INFO] precompute sensor model DONE" << std::endl;
    map_initialized = true;
    std::cout << "[INFO] Waiting for initial pose." << std::endl;
}

/**
    clicked_pose callback relative to click_sub subsciber in main
*/
void ParticleFilterNode::clicked_poseCb(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    double initial_roll, initial_pitch, initial_yaw;
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);

    m.getRPY(initial_roll, initial_pitch, initial_yaw);

    initial_pose[0] = msg->pose.pose.position.x;
    initial_pose[1] = msg->pose.pose.position.y;
    initial_pose[2] = initial_yaw;

    std::cout << "X = " << msg->pose.pose.position.x << std::endl;
    std::cout << "Y = " << msg->pose.pose.position.y << std::endl;
    std::cout << "Yaw = " << initial_yaw << std::endl;

    particleFilter.initialize_particles_pose(initial_pose);
    std::cout << "[INFO] initalize particle pose DONE" << std::endl;
    //    visualize();
    take_pose = true;
    first_MCL = true;
    std::cout << "[INFO] ready" << std::endl;
}

/**
    odometry callback relative to odom_sub subsciber in main
*/
void ParticleFilterNode::OdomCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    double roll, pitch, yaw;
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                      msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z,
                      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    odom_x = msg->pose.pose.position.x;
    odom_y = msg->pose.pose.position.y;
    odom_v = msg->twist.twist.linear.x;
    if (first_odom) {
        oldYaw = yaw;
        oldX = odom_x;
        oldY = odom_y;
        last_stamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
        first_odom = false;
        return;
    }

    dts = (msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec - last_stamp);
    w = (yaw - oldYaw) / dts;
    x = (odom_x - oldX);
    y = (odom_y - oldY);
    velocity = (sqrt(pow(x, 2) + pow(y, 2))) / dts;

    last_stamp = msg->header.stamp.sec + 1e-9 * msg->header.stamp.nanosec;
    oldYaw = yaw;
    oldX = odom_x;
    oldY = odom_y;
    oldDts = dts;

    odom_initialized = true;
    // update();
}

/**
    lidar callback relative to lidar_sub subsciber in main
*/
float downsample_time, loop_time;
int downsample_counter, loop_counter; 

void ParticleFilterNode::lidarCb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!map_initialized || !take_pose)
        return;

    sensor_msgs::msg::LaserScan downsampled_msg = *msg;
    std::vector<float> ranges;
    downsampled_msg.ranges = std::vector<float>();

    static std::chrono::steady_clock::time_point start, end;
    start = std::chrono::steady_clock::now();
    angle_min = msg->angle_min;
    angle_max = msg->angle_max;
    int valid_rays = 0;
    for (int i = 0; i < msg->ranges.size(); i++) {
        if (msg->ranges[i] < msg->range_max && msg->ranges[i] > msg->range_min)
            valid_rays++;
    }

    // nearest neighbor without multiplications or divisions
    int i = 0, j = 0, tmp = valid_rays;
    for (int i = 0; i < msg->ranges.size(); ++i) {
        tmp += N_RAYS_DS - 1;
        if (tmp >= valid_rays) {
            tmp -= valid_rays;
            while (msg->ranges[i] > msg->range_max || msg->ranges[i] < msg->range_min) {
                ranges.push_back(0.0);
                i++;
            }
            rays_angles[j] = i * ANGLE_STEP;
            scan[j] = msg->ranges[i];
            ranges.push_back(scan[j]);
            j++;
        } else {
            ranges.push_back(0.0);
        }
    }

    while (j < N_RAYS_DS) {
        scan[j++] = 0.0f;
    }
    end = std::chrono::steady_clock::now();
    //float l = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    //std::cout << "downsample time: " << l << "us" << std::endl;
    //downsample_time = downsample_time + l;
    //downsample_counter++;
    
    //float downsample_mean = downsample_time/downsample_counter;
    //std::cout << "downsample mean: " << downsample_mean << "us" << std::endl;

    // lidar_initialized = true;
    update(msg->header.stamp);

    //end = std::chrono::steady_clock::now();

    downsampled_msg.ranges = ranges;
    lidar_pub->publish(downsampled_msg);
    //float ds = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
    //std::cout << "loop time: " << ds << "us" << std::endl;
    //loop_time = loop_time + ds;
    //loop_counter++;
    
    //float loop_mean = loop_time/loop_counter;
    //std::cout << "loop mean: " << loop_mean << "us" << std::endl;
    
    //int end_ = rclcpp::Clock{}.now().seconds();
    //std::cout << end_ << std::endl;
    
    std::cout << "______________________" << std::endl;   
    
    
    
    
}

/**
   Function that calls the Monte Carlo algorithm and updates
   the position of the car at each cycle
*/
void ParticleFilterNode::update(rclcpp::Time timestamp)
{
    if (odom_initialized) {
        MCL();
        prev_pose = poses;
        poses = particleFilter.expected_pose();
        variance = particleFilter.expected_variance(poses);

        publish_tf(timestamp);
        visualize(timestamp);
        // always run when lidar is ready
        // odom_initialized = false;
        // lidar_initialized = false;
        is_update = true;
    }
}

/**
   Function that implements the Monte Carlo algorithm by calling
   the various functions present in the Particle Filter class
*/
void ParticleFilterNode::MCL()
{
    if (!first_MCL) {
        particleFilter.particles = particleFilter.resample(particleFilter.MAX_PARTICLES);
    }

    particleFilter.motion_model(odom_v, dts, w);
    particleFilter.sensor_model(scan, rays_angles);
    particleFilter.normalize();

    first_MCL = false;
}

/**
   It puts as origin of the axes of the TF not the map
   but the moving car
*/
void ParticleFilterNode::publish_tf(rclcpp::Time timestamp)
{
    geometry_msgs::msg::TransformStamped ts;

    ts.header.stamp = timestamp;
    ts.header.frame_id = "lidar_link";
    ts.child_frame_id = "map";

    ts.transform.translation.x = poses[0];
    ts.transform.translation.y = poses[1];
    ts.transform.translation.z = 0.0;

    tf2::Quaternion q = createQuaternionFromYaw(poses[2]);
    ts.transform.rotation.x = q.x();
    ts.transform.rotation.y = q.y();
    ts.transform.rotation.z = q.z();
    ts.transform.rotation.w = q.w();

    tf2::Transform tf;
    tf2::fromMsg(ts.transform, tf);
    ts.transform = tf2::toMsg(tf.inverse());

    tf_broadcaster->sendTransform(ts);
}

/**
  Publish position of car and particles as vectors
*/
void ParticleFilterNode::visualize(rclcpp::Time timestamp)
{
    if (!first_MCL) {
        position.header.frame_id = "map";
        position.header.stamp = timestamp;

        // position
        position.pose.pose.position.x = (double)poses[0];
        position.pose.pose.position.y = (double)poses[1];
        position.pose.pose.position.z = 0.0f;

        // orientation
        position.pose.pose.orientation = createQuaternionMsgFromYaw(poses[2]);

        // covariance
        position.pose.covariance[0] = variance[0];  // x
        position.pose.covariance[7] = variance[1];  // y
        position.pose.covariance[35] = variance[2]; // yaw

        // speed
        position.twist.twist.linear.x = ((double)(poses[0] - prev_pose[0])) / oldDts;
        position.twist.twist.linear.y = ((double)(poses[1] - prev_pose[1])) / oldDts;
        position.twist.twist.angular.z = ((double)(poses[2] - prev_pose[2])) / oldDts;

        pose_pub->publish(position);
    }

    int end = std::min(particleFilter.MAX_PARTICLES, 100);
    particles.header.frame_id = "map";
    particles.poses.resize(end);
    particle_pose.resize(end);

    for (int i = 0; i < end; i++) {
        particle_pose[i].x = particleFilter.particles[i].x;
        particle_pose[i].y = particleFilter.particles[i].y;
        particle_pose[i].yaw = particleFilter.particles[i].yaw;
    }

    for (int i = 0; i < end; i++) {
        particles.poses[i].position.x = particle_pose[i].x;
        particles.poses[i].position.y = particle_pose[i].y;
        particles.poses[i].position.z = 0.0f;
        particles.poses[i].orientation = createQuaternionMsgFromYaw(particle_pose[i].yaw);
    }
    particles_pub->publish(particles);

    first_visualize = false;
}

tf2::Quaternion createQuaternionFromYaw(float yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return q;
}

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(float yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    ParticleFilterNode* pfn = new ParticleFilterNode();

    rclcpp::shutdown();
    return 0;
}
