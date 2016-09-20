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
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseWithCovariance.h"



using namespace std;
using namespace Eigen;
using namespace g2o;




class GraphUncertainty
{
 public:
	GraphUncertainty();
	GraphUncertainty(string laser_frame, string fixed_frame, string odom_frame);

	g2o::SE2 listen_tf_odom();
	
	void init_laser_id(string new_frame_id){ laser_frame_id = new_frame_id;}

	void PoseArrayCallback(const geometry_msgs::PoseArray& pose_array_msg);

 protected:
  ros::NodeHandle _nh;
  ros::Publisher Uncertainty_pub_;
  ros::Subscriber pose_array_sub_;
  
  string laser_frame_id;
  string fixed_frame_id;
  string odom_frame_id;
  
  int marker_seq;
  int map_seq;


};




