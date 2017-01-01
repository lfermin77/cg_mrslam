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
		//	int idx_Big[Trayectory_size*Trayectory_size] ;	
		//	int idy_Big[Trayectory_size*Trayectory_size];
		
		int Trayectory_size=2;
		
		std::vector<int> idx_Big;
		idx_Big.resize(Trayectory_size*Trayectory_size);
		
		std::vector<int> idy_Big;
		idy_Big.resize(Trayectory_size*Trayectory_size);
		
		
		for(int i=0 ; i < Trayectory_size ; i ++){
			for (int j=0 ; j < Trayectory_size ; j ++){
				idx_Big[i+Trayectory_size*j] = 2;
				idy_Big[i+Trayectory_size*j] = 3;
		
			}
		}		
		for(int i=0 ; i < Trayectory_size*Trayectory_size ; i ++)
			retrievalList_Big.push_back( std::pair<int,int>(idx_Big[i],idy_Big[i]));

		g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv_Big(idx_Big.data(),idy_Big.data(),Trayectory_size*Trayectory_size , Trayectory_size*Trayectory_size);

						
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
	





void GraphUncertainty::calculate_every_uncertainty(int fixed_vertex){

	int node_size = _graph->vertices().size();
	
	std::vector< std::pair<int,int> > retrievalList;
	
	std::vector<int> idx, idy;

	
	//*
	int k=0;
	for(int i=0 ; i < node_size ; i ++){
		if(i != fixed_vertex){
			retrievalList.push_back( std::pair<int,int>(k,k));
			k++;
			idx.push_back(i+1);
			idy.push_back(i+1);
		}
	}
//*/

/*
	node_size --;
	idx.resize(node_size);
	idy.resize(node_size);


	for(int i=0 ; i < node_size ; i ++)
		retrievalList.push_back( std::pair<int,int>(i,i));

	for(int i=0 ; i < node_size ; i ++){
			idx[i] = i+1;
			idy[i] = i+1;
	}	
	//*/
	
	for(int i=0 ; i < idx.size() ; i ++){
		std::cerr << " idx " << idx[i] << std::endl;
	}	
	for(int i=0 ; i < retrievalList.size() ; i ++){
 		std::cerr << " retrievalList " << retrievalList[i].first << ","<<retrievalList[i].second << std::endl;
	}	



	g2o::SparseBlockMatrix<Eigen::MatrixXd> spinv(idx.data(),idy.data(), idx.size(), idx.size() );

	std::cerr << " Marginals before " << spinv << std::endl;
					
	_graph->solver()->computeMarginals(spinv,retrievalList);

	std::cerr << " Marginals calculated " << spinv << std::endl;
	
}
	
	
	
	
	
	
	
	
	
	
	
	
