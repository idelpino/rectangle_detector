#include "rectangle_detector_alg.h"

#define OUT_OF_RANGE 100.0

RectangleDetectorAlgorithm::RectangleDetectorAlgorithm(void)
{
  pthread_mutex_init(&this->access_, NULL);
}

RectangleDetectorAlgorithm::~RectangleDetectorAlgorithm(void)
{
  pthread_mutex_destroy(&this->access_);
}

void RectangleDetectorAlgorithm::config_update(Config& config, uint32_t level)
{
  this->lock();

  // save the current configuration
  this->config_ = config;

  this->unlock();
}

void initializeGrid(float* grid, int cells_number)
{
  const float IMPOSSIBLE_RANGE_VALUE = -1.0;

  for (int i = 0; i < cells_number; ++i)
  {
    grid[i] = IMPOSSIBLE_RANGE_VALUE;
  }
}

void cartesian2PolarInDegrees(float x, float y, float& rho, float& theta)
{
  theta = atan2(y, x) * 180.0 / M_PI;
  if (theta < 0)
    theta += 360.0;
  if (theta >= 360)
    theta -= 360;

  rho = sqrt((x * x) + (y * y));

  return;
}

void searchRectangleVertex(pcl::PointCloud<pcl::PointXYZ>& cluster, float search_angle_in_degrees, pcl::PointXYZ& a,
                           pcl::PointXYZ& b, pcl::PointXYZ& c, pcl::PointXYZ& d)
{
  pcl::PointXYZ y_up_frontier;
  pcl::PointXYZ y_down_frontier;
  pcl::PointXYZ x_right_frontier;
  pcl::PointXYZ x_left_frontier;

  pcl::PointXYZ candidate_point;

  float m1, m2, b1, b2, b_y_max, b_y_min, b_x_max, b_x_min;
  float x_min, x_max, y_min, y_max;

  //ROS_INFO_STREAM (std::endl << " Search angle = " << search_angle_in_degrees);
  if (search_angle_in_degrees != 0.0) // to avoid singularities tg -> inf
  {
    //ROS_INFO_STREAM (std::endl << " Using tangents to find vertex! ");

    m1 = tan(search_angle_in_degrees * M_PI / 180.0);
    m2 = tan((search_angle_in_degrees - 90.0) * M_PI / 180.0);

    b1 = 0.0;
    b2 = 0.0;

    b_y_max = -1000000.0;
    b_y_min = 1000000.0;

    b_x_max = -1000000.0;
    b_x_min = 1000000.0;

    for (int i = 0; i < cluster.points.size(); ++i)
    {
      candidate_point = cluster.points[i];

      b1 = candidate_point.y - m1 * candidate_point.x;
      b2 = candidate_point.y - m2 * candidate_point.x;

      if (b1 > b_y_max)
        b_y_max = b1;

      if (b2 > b_x_max)
        b_x_max = b2;

      if (b1 < b_y_min)
        b_y_min = b1;

      if (b2 < b_x_min)
        b_x_min = b2;
    }

    a.x = (b_x_min - b_y_max) / (m1 - m2);
    a.y = m2 * a.x + b_x_min;
    a.z = 0.0;

    b.x = (b_x_max - b_y_max) / (m1 - m2);
    b.y = m2 * b.x + b_x_max;
    b.z = 0.0;

    c.x = (b_x_max - b_y_min) / (m1 - m2);
    c.y = m2 * c.x + b_x_max;
    c.z = 0.0;

    d.x = (b_x_min - b_y_min) / (m1 - m2);
    d.y = m2 * d.x + b_x_min;
    d.z = 0.0;

  }
  else
  {
    y_max = -1000000.0;
    y_min = 1000000.0;

    x_max = -1000000.0;
    x_min = 1000000.0;

    for (int i = 0; i < cluster.points.size(); ++i)
    {
      candidate_point = cluster.points[i];

      if (candidate_point.x > x_max)
        x_max = candidate_point.x;

      if (candidate_point.y > y_max)
        y_max = candidate_point.y;

      if (candidate_point.x < x_min)
        x_min = candidate_point.x;

      if (candidate_point.y < y_min)
        y_min = candidate_point.y;
    }

    a.x = x_min;
    a.y = y_max;
    a.z = 0.0;

    b.x = x_max;
    b.y = y_max;
    b.z = 0.0;

    c.x = x_max;
    c.y = y_min;
    c.z = 0.0;

    d.x = x_min;
    d.y = y_min;
    d.z = 0.0;
  }
}

void generatePolarGrid(pcl::PointCloud<pcl::PointXYZ>& cluster, float* grid, SensorConfiguration lidar_configuration)
{
  int index;
  float x, y, range, h_ang;

  initializeGrid(grid, lidar_configuration.num_of_horizontal_cells);

  for (size_t i = 0; i < cluster.points.size(); ++i)
  {
    x = cluster.points[i].x;
    y = cluster.points[i].y;

    cartesian2PolarInDegrees(x, y, range, h_ang);

    index = (int)round(
        (h_ang - lidar_configuration.min_horizontal_angle) / lidar_configuration.grid_horizontal_angular_resolution);

    if (index < 0 || index > lidar_configuration.num_of_horizontal_cells)
    {
      // This warning was programmed before adding the angular limits min_h_ang
      // and max_h_ang, now it is normal to have points falling out of the grid
      // because this limits, so now it not represent a problem, the warning anyway
      // is left here commented for debugging purposes
      //std::cout << std::endl << "Illegal index in function generatePolarGrid!!";
    }
    else
    {
      if (grid[index] == -1.0)
      {
        grid[index] = range;
      }
      else
      {
        if (range < grid[index])
          grid[index] = range;
      }
    }
  }
}

void generatePolarGridFromStraightLine(pcl::PointXYZ a, pcl::PointXYZ b, SensorConfiguration lidar_configuration,
                                       float* grid)
{
  float theta_min = 1000.0;
  float theta_max = -1000.0;

  float theta_deg = 0.0;
  float theta_rad = 0.0;

  float rho = 0.0;

  float alpha_rad = atan2((b.y - a.y), (b.x - a.x));
  float alpha_deg = alpha_rad * 180.0 / M_PI;

  float m = 0.0;
  float n = 0.0;

  bool vertical = false;
  int index = 0;

  if (fabs(fabs(alpha_deg) - 90.0) < lidar_configuration.grid_horizontal_angular_resolution
      || fabs(fabs(alpha_deg) - 270.0) < lidar_configuration.grid_horizontal_angular_resolution)
  {
    vertical = true;
  }

  if (!vertical)
  {
    m = (b.y - a.y) / (b.x - a.x);
    n = a.y - m * a.x;
  }

  //ROS_INFO_STREAM ( std::endl << "alpha = " << alpha_deg );

  float cos_alpha = cos(alpha_rad);
  float sin_alpha = sin(alpha_rad);

  cartesian2PolarInDegrees(a.x, a.y, rho, theta_deg);
  if (theta_deg < theta_min)
    theta_min = theta_deg;
  if (theta_deg > theta_max)
    theta_max = theta_deg;

  cartesian2PolarInDegrees(b.x, b.y, rho, theta_deg);
  if (theta_deg < theta_min)
    theta_min = theta_deg;
  if (theta_deg > theta_max)
    theta_max = theta_deg;

  if (theta_max - theta_min >= 180.0)
  {
    float swap = theta_min;
    theta_min = theta_max;
    theta_max = swap + 360.0;
  }

  int initial_index = (int)round(
      (theta_min - lidar_configuration.min_horizontal_angle) / lidar_configuration.grid_horizontal_angular_resolution);
  theta_min = ((float)initial_index * lidar_configuration.grid_horizontal_angular_resolution)
      + lidar_configuration.min_horizontal_angle;

  int final_index = (int)round(
      (theta_max - lidar_configuration.min_horizontal_angle) / lidar_configuration.grid_horizontal_angular_resolution);
  theta_max = ((float)final_index * lidar_configuration.grid_horizontal_angular_resolution)
      + lidar_configuration.min_horizontal_angle;

  for (theta_deg = theta_min; theta_deg <= theta_max;
      theta_deg = theta_deg + lidar_configuration.grid_horizontal_angular_resolution)
  {
    theta_rad = theta_deg * M_PI / 180.0;
    if (!vertical)
    {
      if (fabs(theta_deg - alpha_deg) > lidar_configuration.grid_horizontal_angular_resolution
          && fabs(fabs(theta_deg - alpha_deg) - 180.0) > lidar_configuration.grid_horizontal_angular_resolution)
      {
        rho = (n * cos_alpha) / sin(theta_rad - alpha_rad);
      }
      else
      {
        float d_a_squared = (a.x * a.x) + (a.y * a.y);
        float d_b_squared = (b.x * b.x) + (b.y * b.y);

        if (d_a_squared < d_b_squared)
        {
          rho = sqrt(d_a_squared);
        }
        else
        {
          rho = sqrt(d_b_squared);
        }
      }
    }
    else
    {
      if (fabs(fabs(theta_deg) - 90.0) > lidar_configuration.grid_horizontal_angular_resolution
          && fabs(fabs(theta_deg) - 270.0) > lidar_configuration.grid_horizontal_angular_resolution)
      {
        rho = a.x / cos(theta_rad);
      }
      else
      {
        if (fabs(a.y) < fabs(b.y))
        {
          rho = a.y;
        }
        else
        {
          rho = b.y;
        }
      }
    }

    float aux = 0.0;
    if (theta_deg > 360)
    {
      aux = theta_deg - 360;
    }
    else
    {
      aux = theta_deg;
    }

    index = (int)round(
        (aux - lidar_configuration.min_horizontal_angle) / lidar_configuration.grid_horizontal_angular_resolution);

    // If min_h_ang is greater than zero it is normal having negative indexes that correspond to points that
    // fall in the filtered area, what we do is simply don't put them in the grid, also if max_h_ang is smaller
    // than 360-ang_res indexes could be greater than the vector dimension
    if (index >= 0
        && index
            <= (int)round(
                (lidar_configuration.max_horizontal_angle - lidar_configuration.min_horizontal_angle)
                    / lidar_configuration.grid_horizontal_angular_resolution) + 1)
    {
      grid[index] = fabs(rho);
    }
    else
    {
      // This warning was programmed previous to the addition of the angular limits, is left here only for debuggin
      // purposes

      //ROS_INFO_STREAM (
      //"In function generatePolarGridFromStraightLine: WARNING!!! Attempted to write in invalid index, index = "
      //<< index << "Rho = " << rho << " vertical = " << vertical
      //<< " alpha = " << alpha_deg << " theta = " << theta_deg << " n = "
      //<< n);
    }
  }
  //ROS_INFO_STREAM ( "Polar grid from straight line generated!!" );
}

void generateRectangularPolarGrid(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c, pcl::PointXYZ d, float* grid,
                                  SensorConfiguration lidar_configuration, float& width, float& length, float &x,
                                  float& y)
{
  float grid_ab[lidar_configuration.num_of_horizontal_cells];
  float grid_dc[lidar_configuration.num_of_horizontal_cells];

  float grid_ad[lidar_configuration.num_of_horizontal_cells];
  float grid_bc[lidar_configuration.num_of_horizontal_cells];

  float d1 = sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y)); // Length of one of the sides
  float d2 = sqrt((c.x - b.x) * (c.x - b.x) + (c.y - b.y) * (c.y - b.y));

  float m1 = atan2((b.y - a.y), (b.x - a.x));
  float m2 = atan2((c.y - b.y), (c.x - b.x));

  float cos_m1 = cos(m1);
  float sin_m1 = sin(m1);

  float cos_m2 = cos(m2);
  float sin_m2 = sin(m2);

  float x_center_of_d1 = a.x + (d1 / 2.0) * cos_m1;
  float y_center_of_d1 = a.y + (d1 / 2.0) * sin_m1;

  x = x_center_of_d1 + (d2 / 2.0) * cos_m2;
  y = y_center_of_d1 + (d2 / 2.0) * sin_m2;

  length = d1;
  width = d2;

  initializeGrid(grid, lidar_configuration.num_of_horizontal_cells);

  initializeGrid(grid_ab, lidar_configuration.num_of_horizontal_cells);
  initializeGrid(grid_dc, lidar_configuration.num_of_horizontal_cells);
  initializeGrid(grid_ad, lidar_configuration.num_of_horizontal_cells);
  initializeGrid(grid_bc, lidar_configuration.num_of_horizontal_cells);

  generatePolarGridFromStraightLine(a, b, lidar_configuration, grid_ab);
  generatePolarGridFromStraightLine(d, c, lidar_configuration, grid_dc);
  generatePolarGridFromStraightLine(a, d, lidar_configuration, grid_ad);
  generatePolarGridFromStraightLine(b, c, lidar_configuration, grid_bc);

  float rho_min;
  for (int i = 0; i < lidar_configuration.num_of_horizontal_cells; ++i)
  {
    rho_min = OUT_OF_RANGE;
    if (grid_ab[i] != -1 && grid_ab[i] < rho_min)
      rho_min = grid_ab[i];

    if (grid_dc[i] != -1 && grid_dc[i] < rho_min)
      rho_min = grid_dc[i];

    if (grid_ad[i] != -1 && grid_ad[i] < rho_min)
      rho_min = grid_ad[i];

    if (grid_bc[i] != -1 && grid_bc[i] < rho_min)
      rho_min = grid_bc[i];

    if (rho_min != OUT_OF_RANGE)
    {
      grid[i] = rho_min;
    }
    else
    {
      grid[i] = -1;
    }
  }
}

void generateRectangleCandidatePolarGrid(pcl::PointCloud<pcl::PointXYZ>& original_cluster,
                                         float search_angle_in_degrees, float* candidate_grid,
                                         SensorConfiguration lidar_configuration, float& candidate_width,
                                         float& candidate_length, float& x, float& y, float &error)
{
  const float IMPOSSIBLE_ERROR_VALUE = 1000000.0;

  pcl::PointXYZ a, b, c, d;

  searchRectangleVertex(original_cluster, search_angle_in_degrees, a, b, c, d);

  float cluster_grid[lidar_configuration.num_of_horizontal_cells];

  generatePolarGrid(original_cluster, cluster_grid, lidar_configuration);

  generateRectangularPolarGrid(a, b, c, d, candidate_grid, lidar_configuration, candidate_width, candidate_length, x,
                               y);

  //std::cout << std::endl << "Starting error calculus in generateRectangleCandidatePolarGrid!!";

  error = 0.0;
  float partial_error = 0.0;
  int num_of_points_evaluated = 0;

  for (int i = 0; i < lidar_configuration.num_of_horizontal_cells; ++i)
  {
    //std::cout << std::endl << " Iterator = " << i << "    rho = " << candidate_grid[i] << std::endl;
    if (cluster_grid[i] != -1)
    {
      if (candidate_grid[i] != -1)
      {
        partial_error = (cluster_grid[i] - candidate_grid[i]) * (cluster_grid[i] - candidate_grid[i]);
        error += partial_error;
        num_of_points_evaluated++;
      }
    }
  }
  error = error / (float)num_of_points_evaluated;
}

void RectangleDetectorAlgorithm::findBestOrientedRectangleContainingTheCluster(
    pcl::PointCloud<pcl::PointXYZ>& cluster, SensorConfiguration lidar_configuration,
    ClusteringConfiguration clustering_configuration, float* central_point, float& width, float& length, float& theta,
    float& mean_squared_error)
{
  const float SEARCH_ANG_RESOLUTION = 1.0;

  const float COARSE_SEARCH_ANG_RESOLUTION = 5.0;

  const float SEARCH_MIN_ANGLE = -45.0;
  const float SEARCH_MAX_ANGLE = 45.0;

  const float IMPOSSIBLE_ERROR_VALUE = 1000000.0;

  float candidate_grid[lidar_configuration.num_of_horizontal_cells];

  float candidate_width = 0.0;
  float candidate_length = 0.0;

  float angle_of_best_candidate = 0.0;
  float search_angle_in_degrees = 0.0;

  float x = 0.0;
  float y = 0.0;

  float error = 0.0;
  float min_error = IMPOSSIBLE_ERROR_VALUE;

  initializeGrid(candidate_grid, lidar_configuration.num_of_horizontal_cells);

  // Coarse search
  for (float i = SEARCH_MIN_ANGLE; i < SEARCH_MAX_ANGLE; i = i + COARSE_SEARCH_ANG_RESOLUTION)
  {
    search_angle_in_degrees = i;

    generateRectangleCandidatePolarGrid(cluster, search_angle_in_degrees, candidate_grid, lidar_configuration,
                                        candidate_width, candidate_length, x, y, error);

    //ROS_INFO_STREAM ( std::endl << " Search angle = " << i << " Error = " << error);
    if (error < min_error)
    {
      min_error = error;
      angle_of_best_candidate = search_angle_in_degrees;
    }
  }

  // Fine search
  float fine_search_min_angle = angle_of_best_candidate - COARSE_SEARCH_ANG_RESOLUTION + SEARCH_ANG_RESOLUTION;
  if (fine_search_min_angle < SEARCH_MIN_ANGLE)
    fine_search_min_angle = SEARCH_MIN_ANGLE;

  float fine_search_max_angle = angle_of_best_candidate + COARSE_SEARCH_ANG_RESOLUTION - SEARCH_ANG_RESOLUTION;
  if (fine_search_max_angle > SEARCH_MAX_ANGLE)
    fine_search_max_angle = SEARCH_MAX_ANGLE;

  error = 0.0;
  min_error = IMPOSSIBLE_ERROR_VALUE;
  for (float i = fine_search_min_angle; i < fine_search_max_angle; i = i + SEARCH_ANG_RESOLUTION)
  {
    search_angle_in_degrees = i;

    generateRectangleCandidatePolarGrid(cluster, search_angle_in_degrees, candidate_grid, lidar_configuration,
                                        candidate_width, candidate_length, x, y, error);

    //ROS_INFO_STREAM ( std::endl << " Search angle = " << i << " Error = " << error);
    if (error < min_error)
    {
      min_error = error;
      angle_of_best_candidate = search_angle_in_degrees;
    }
  }

  //ROS_INFO_STREAM ( std::endl << " Best rectangle found in angle = " << angle_of_best_candidate << " with error of = " << error << std::endl );

  // Now generate the data of the best result
  generateRectangleCandidatePolarGrid(cluster, angle_of_best_candidate, candidate_grid, lidar_configuration,
                                      candidate_width, candidate_length, x, y, error);

  width = candidate_width;
  length = candidate_length;
  theta = angle_of_best_candidate * M_PI / 180.0;
  mean_squared_error = error;
  central_point[0] = x;
  central_point[1] = y;
  central_point[2] = 0.0;
}

void RectangleDetectorAlgorithm::extractObservations(sensor_msgs::LaserScan& laser_scan,
                                                     SensorConfiguration lidar_configuration,
                                                     ClusteringConfiguration clustering_configuration,
                                                     sensor_msgs::PointCloud2& output_pointcloud2,
                                                     std::vector<Observation>& observations,
                                                     visualization_msgs::MarkerArray& marker_array)
{
  sensor_msgs::PointCloud2 input_pointcloud2;
  projector_.projectLaser(laser_scan, input_pointcloud2);

  if (!input_pointcloud2.data.empty())
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCLPointCloud2 aux_input;
    pcl_conversions::toPCL(input_pointcloud2, aux_input);
    pcl::fromPCLPointCloud2(aux_input, *input_cloud_pcl);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(input_cloud_pcl);

    std::vector < pcl::PointIndices > cluster_indices;

    pcl::EuclideanClusterExtraction < pcl::PointXYZ > ec;
    ec.setClusterTolerance(clustering_configuration.clustering_tolerance);
    ec.setMinClusterSize(2);
    //ec.setMaxClusterSize(max_num_points);
    ec.setSearchMethod(tree);
    ec.setInputCloud(input_cloud_pcl);
    ec.extract(cluster_indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    int observation_id = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      float x_accum = 0.0;
      float y_accum = 0.0;
      float z_accum = 0.0;
      int count = 0;

      for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
      {
        pcl::PointXYZ point;

        point.x = input_cloud_pcl->points[*pit].x;
        point.y = input_cloud_pcl->points[*pit].y;
        point.z = input_cloud_pcl->points[*pit].z;

        cloud_cluster->points.push_back(point);

        x_accum += point.x;
        y_accum += point.y;
        z_accum += point.z;
        count++;
      }
      if (count > 0)
      {
        float x_centroid = x_accum / count;
        float y_centroid = y_accum / count;
        float z_centroid = z_accum / count;
        float max_distance = 0.0;

        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
        {
          float dx = x_centroid - input_cloud_pcl->points[*pit].x;
          float dy = y_centroid - input_cloud_pcl->points[*pit].y;
          float dz = z_centroid - input_cloud_pcl->points[*pit].z;

          float distance = sqrt(dx * dx + dy * dy + dz * dz);
          if (distance > max_distance)
            max_distance = distance;
        }

        if (max_distance > clustering_configuration.min_cluster_radius)
        {
          *filtered_cloud += *cloud_cluster;

          float length = 0.0;
          float width = 0.0;

          float theta = 0.0;
          float central_point[3];

          float mean_squared_error = 0.0;

          findBestOrientedRectangleContainingTheCluster(*cloud_cluster, lidar_configuration, clustering_configuration,
                                                        central_point, width, length, theta, mean_squared_error);
          Observation observation;
          observation.id = observation_id;
          observation.x = central_point[0];
          observation.y = central_point[1];
          observation.length_side_a = width;
          observation.length_side_b = length;
          observation.theta = theta;
          observation.mean_squared_error = mean_squared_error;

          observations.push_back(observation);

          observation_id++;

          std::cout << observation.id << std::endl;
          std::cout << observation.x << std::endl;
          std::cout << observation.y << std::endl;
          std::cout << observation.theta << std::endl;
          std::cout << observation.length_side_a << std::endl;
          std::cout << observation.length_side_b << std::endl;
          std::cout << observation.mean_squared_error << std::endl;


          visualization_msgs::Marker marker;
          marker.ns = "rectangle";
          marker.id = observation_id;

          tf::Quaternion qt;

          marker.type = visualization_msgs::Marker::CUBE;

          marker.scale.x = observation.length_side_b;
          marker.scale.y = observation.length_side_a;
          marker.scale.z = 0.1;

          qt.setRPY(0.0, 0.0, theta);

          geometry_msgs::Quaternion qt_msg;
          tf::quaternionTFToMsg(qt, qt_msg);
          marker.pose.orientation = qt_msg;

          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = observation.x;
          marker.pose.position.y = observation.y;
          marker.pose.position.z = 0.0;

          marker.color.a = 0.7; //0.5; // Don't forget to set the alpha!
          marker.color.r = 0;
          marker.color.g = 255;
          marker.color.b = 0;

          marker.header.frame_id = laser_scan.header.frame_id;
          marker.header.stamp    = laser_scan.header.stamp;

          marker_array.markers.push_back(marker);

        }
        else
        {
          //std::cout << "Cluster discarded, radius = " << max_distance << std::endl;
        }
      }
      else
      {
        std::cout << "Warning! in ReactiveHokuyoAlgorithm::eliminateSmallClusters, cluster with zero points!!"
            << std::endl;
      }
    }

    pcl::PCLPointCloud2 aux_output;
    toPCLPointCloud2(*filtered_cloud, aux_output);
    pcl_conversions::fromPCL(aux_output, output_pointcloud2);
  }
  else
  {
    output_pointcloud2 = input_pointcloud2;
  }
}

// RectangleDetectorAlgorithm Public API
