// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author 
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
// 
// IMPORTANT NOTE: This code has been generated through a script from the 
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _rectangle_detector_alg_node_h_
#define _rectangle_detector_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "rectangle_detector_alg.h"

#include "struct_definitions.h"

#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <sensor_msgs/LaserScan.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class RectangleDetectorAlgNode : public algorithm_base::IriBaseAlgorithm<RectangleDetectorAlgorithm>
{
private:

  // Input
  bool flag_new_laser_scan_data_;
  sensor_msgs::LaserScan input_scan_;
  sensor_msgs::LaserScan local_copy_of_input_scan_;

  std::vector<Observation> observations;

  SensorConfiguration lidar_configuration;

  ClusteringConfiguration clustering_configuration;

  // [publisher attributes]

  sensor_msgs::PointCloud2 observed_2D_perimeters;
  ros::Publisher pointcloud_publisher_;
  sensor_msgs::PointCloud2 pointcloud_msg_;

  ros::Publisher markers_publisher_;
  visualization_msgs::MarkerArray marker_array_msg_;
  visualization_msgs::MarkerArray markers;

  // [subscriber attributes]

  ros::Subscriber laser_scan_subscriber_;
  void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  // [service attributes]

  // [client attributes]

  // [action server attributes]

  // [action client attributes]

  pthread_mutex_t laser_scan_mutex_;
  void laser_scan_mutex_enter(void);
  void laser_scan_mutex_exit(void);

  /**
   * \brief config variable
   *
   * This variable has all the driver parameters defined in the cfg config file.
   * Is updated everytime function config_update() is called.
   */
  Config config_;
public:
  /**
   * \brief Constructor
   *
   * This constructor initializes specific class attributes and all ROS
   * communications variables to enable message exchange.
   */
  RectangleDetectorAlgNode(void);

  /**
   * \brief Destructor
   *
   * This destructor frees all necessary dynamic memory allocated within this
   * this class.
   */
  ~RectangleDetectorAlgNode(void);

protected:
  /**
   * \brief main node thread
   *
   * This is the main thread node function. Code written here will be executed
   * in every node loop while the algorithm is on running state. Loop frequency
   * can be tuned by modifying loop_rate attribute.
   *
   * Here data related to the process loop or to ROS topics (mainly data structs
   * related to the MSG and SRV files) must be updated. ROS publisher objects
   * must publish their data in this process. ROS client servers may also
   * request data to the corresponding server topics.
   */
  void mainNodeThread(void);

  /**
   * \brief dynamic reconfigure server callback
   *
   * This method is called whenever a new configuration is received through
   * the dynamic reconfigure. The derivated generic algorithm class must
   * implement it.
   *
   * \param config an object with new configuration from all algorithm
   *               parameters defined in the config file.
   * \param level  integer referring the level in which the configuration
   *               has been changed.
   */
  void node_config_update(Config &config, uint32_t level);

  /**
   * \brief node add diagnostics
   *
   * In this abstract function additional ROS diagnostics applied to the
   * specific algorithms may be added.
   */
  void addNodeDiagnostics(void);

  // [diagnostic functions]

  // [test functions]
};

#endif
