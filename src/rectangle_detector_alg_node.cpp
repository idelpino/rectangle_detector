#include "rectangle_detector_alg_node.h"

RectangleDetectorAlgNode::RectangleDetectorAlgNode(void) :
    algorithm_base::IriBaseAlgorithm<RectangleDetectorAlgorithm>()
{
  //init class attributes if necessary
  flag_new_laser_scan_data_ = false;

  lidar_configuration.grid_horizontal_angular_resolution = 0.1;
  lidar_configuration.max_horizontal_angle = 359.9;
  lidar_configuration.min_horizontal_angle = 0.0;
  lidar_configuration.num_of_horizontal_cells = 3600;

  clustering_configuration.clustering_tolerance = 0.1;
  clustering_configuration.max_num_points = 10000;
  clustering_configuration.min_cluster_radius = 0.01;
  clustering_configuration.min_num_points = 2;

  //this->loop_rate_ = 2;//in [Hz]

  // [init publishers]
  this->pointcloud_publisher_ = this->public_node_handle_.advertise
          < sensor_msgs::PointCloud2 > ("pointcloud", 1);

  this->markers_publisher_ = this->public_node_handle_.advertise < visualization_msgs::MarkerArray
      > ("markers", 1);

  // [init subscribers]
  this->laser_scan_subscriber_ = this->public_node_handle_.subscribe("/scan", 1,
                                                                     &RectangleDetectorAlgNode::laser_scan_callback,
                                                                     this);

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

RectangleDetectorAlgNode::~RectangleDetectorAlgNode(void)
{
  // [free dynamic memory]
}

void RectangleDetectorAlgNode::mainNodeThread(void)
{
  this->laser_scan_mutex_enter();
  if (flag_new_laser_scan_data_)
  {
    // [fill msg structures]
    flag_new_laser_scan_data_ = false;
    local_copy_of_input_scan_ = input_scan_;
    this->laser_scan_mutex_exit();

    this->alg_.extractObservations(local_copy_of_input_scan_, lidar_configuration, clustering_configuration,
                                   observed_2D_perimeters, observations, markers);

    observed_2D_perimeters.header.frame_id = local_copy_of_input_scan_.header.frame_id;
    observed_2D_perimeters.header.stamp    = local_copy_of_input_scan_.header.stamp;
    pointcloud_msg_ = observed_2D_perimeters;
    this->pointcloud_publisher_.publish (this->pointcloud_msg_);

    marker_array_msg_ = markers;
    this->markers_publisher_.publish (this->marker_array_msg_);

  }
  else
  {
    this->laser_scan_mutex_exit();
  }
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void RectangleDetectorAlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  this->alg_.unlock();
}

void RectangleDetectorAlgNode::addNodeDiagnostics(void)
{
}

void RectangleDetectorAlgNode::laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  this->laser_scan_mutex_enter();

  //if(strcmp(msg->header.frame_id.c_string(), "Laser") == 0)
  if (msg->header.frame_id == "laser")
  {
    input_scan_ = *msg;

    //DEBUG!!
    //std::cout << "Laser scan received!" << std::endl;
    if (msg == NULL)
      std::cout << std::endl << "Null pointer!!! in function laser_scan_callback!";

    flag_new_laser_scan_data_ = true;
  }

  this->laser_scan_mutex_exit();
}

void RectangleDetectorAlgNode::laser_scan_mutex_enter(void)
{
  pthread_mutex_lock(&this->laser_scan_mutex_);
}

void RectangleDetectorAlgNode::laser_scan_mutex_exit(void)
{
  pthread_mutex_unlock(&this->laser_scan_mutex_);
}

/* main function */
int main(int argc, char *argv[])
{
  return algorithm_base::main < RectangleDetectorAlgNode > (argc, argv, "rectangle_detector_alg_node");
}
