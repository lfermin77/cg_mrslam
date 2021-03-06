#include "occ_grid/frequency_map.h"

//ROS
#include "nav_msgs/GetMap.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"

//tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//g2o
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"

#include "g2o/solvers/csparse/linear_solver_csparse.h"

#include "g2o/types/data/robot_laser.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"





using namespace std;
using namespace Eigen;
using namespace g2o;




class Graph2RosMap
{
 public:
	Graph2RosMap();
	Graph2RosMap(string laser_frame, string fixed_frame, string odom_frame);
	int graph_2_occ( SparseOptimizer *graph) ;
	tf::Transform update_transform(g2o::SE2 optimized, g2o::SE2 odom );
	int publish_markers(  SparseOptimizer *graph) ;
	g2o::SE2 listen_tf_odom();
	
	void init_laser_id(string new_frame_id){ laser_frame_id = new_frame_id;}
	
	void GT_callback(const nav_msgs::Odometry::ConstPtr& msg);
	void publish_ground_truth();


 protected:
  ros::NodeHandle _nh;
  ros::Publisher map_pub_;
  ros::Publisher markers_pub_;
  ros::Publisher GT_trajectory_pub_;
  
  ros::Subscriber GT_sub_;
  
  string laser_frame_id;
  string fixed_frame_id;
  string odom_frame_id;
  
  int marker_seq;
  int map_seq;
  
  geometry_msgs::Point current_position;
  geometry_msgs::Point* first_position;
  
  std::vector<geometry_msgs::Point> GT_trajectory;


};




