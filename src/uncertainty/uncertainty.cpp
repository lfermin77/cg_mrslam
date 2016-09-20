#include "uncertainty.h"



using namespace std;
using namespace Eigen;
using namespace g2o;


GraphUncertainty::GraphUncertainty(SparseOptimizer* graph){

	_graph = graph;
	Uncertainty_pub_ =  _nh.advertise<geometry_msgs::PoseWithCovariance>("query_Uncertainty", 10);
	pose_array_sub_  =  _nh.subscribe("query_Poses", 10, &GraphUncertainty::PoseArrayCallback, this);
	laser_frame_id = "base_laser_link";
	fixed_frame_id = "map";
	odom_frame_id  = "odom";
	
	marker_seq=0;
    map_seq=0;
}






/////////////////////////
void GraphUncertainty::PoseArrayCallback(const geometry_msgs::PoseArray& pose_array_msg){
	geometry_msgs::PoseWithCovariance pepe;
//	Uncertainty_pub_.publish(pepe);
	std::cout << "Im in" << std::endl;
	std::vector< std::pair<int,int> > retrievalList_Big;
	
//	g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv_Big(idx_Big.data(),idy_Big.data(),Trayectory_size*Trayectory_size , Trayectory_size*Trayectory_size);
	g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv_Big;

						
	_graph->solver()->computeMarginals(spinv_Big,retrievalList_Big);
	
}



// Linear Transformation
Eigen::Matrix3d CalculateRelativeJacobian1( double from[], double to[]){
	Eigen::Matrix3d JacobianRelative1;
	float s1=sin(from[2]);
	float c1=cos(from[2]);
	
	float deltaX= to[0]-from[0];
	float deltaY= to[1]-from[1];
	
	JacobianRelative1(0,0) = -c1;
	JacobianRelative1(0,1) =  s1;
	JacobianRelative1(0,2) = -deltaX*s1 - deltaY*c1;
	
	JacobianRelative1(1,0) = -s1;
	JacobianRelative1(1,1) = -c1;
	JacobianRelative1(1,2) = deltaX*c1 - deltaY*s1;
	
	JacobianRelative1(2,0) =  0;
	JacobianRelative1(2,1) =  0;
	JacobianRelative1(2,2) = -1;
	
	
	return JacobianRelative1;
		}

Eigen::Matrix3d CalculateRelativeJacobian2( double from[], double to[]){
	Eigen::Matrix3d JacobianRelative2;
	float s1=sin(from[2]);
	float c1=cos(from[2]);
	
	JacobianRelative2(0,0) =  c1;
	JacobianRelative2(0,1) = -s1;
	JacobianRelative2(0,2) =   0;
		
	JacobianRelative2(1,0) =  s1;
	JacobianRelative2(1,1) =  c1;
	JacobianRelative2(1,2) =   0;
		
	JacobianRelative2(2,0)=0;
	JacobianRelative2(2,1)=0;
	JacobianRelative2(2,2)=1;
	return JacobianRelative2;
	}
	





