#include "occ_grid/frequency_map.h"

//ROS
#include "nav_msgs/GetMap.h"
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

#include "visualization_msgs/Marker.h"



using namespace std;
using namespace Eigen;
using namespace g2o;




class Graph2RosMap
{
 public:
	Graph2RosMap();
	int graph_2_occ(nav_msgs::OccupancyGrid &map_msg, SparseOptimizer *graph) ;
	tf::Transform update_transform(g2o::SE2 optimized, g2o::SE2 odom );
	int publish_markers( visualization_msgs::Marker &marker, SparseOptimizer *graph) ;

 protected:
  ros::NodeHandle _nh;
  ros::Publisher map_pub_;
  ros::Publisher markers_pub_;
  string laser_frame_id;
  string fixed_frame_id;


};




