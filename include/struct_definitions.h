#ifndef _struct_definitions_h_
#define _struct_definitions_h_

struct Observation
{
  float x;
  float y;
  float theta;
  float mean_squared_error;

  float length_side_a;
  float length_side_b;

  int id;
};

struct SensorConfiguration
{
  float max_horizontal_angle;
  float min_horizontal_angle;

  float grid_horizontal_angular_resolution;

  int num_of_horizontal_cells;
  int num_of_vertical_cells;
};

struct ClusteringConfiguration
{
  double clustering_tolerance;
  int min_num_points;
  int max_num_points;
  double min_cluster_radius;
};

#endif
