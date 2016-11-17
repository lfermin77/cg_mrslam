#include "g2o_2_occ.h"



using namespace std;
using namespace Eigen;
using namespace g2o;



Graph2RosMap::Graph2RosMap(){
	markers_pub_     =  _nh.advertise<visualization_msgs::Marker>( "SLAM_Graph", 10 );
	map_pub_         =  _nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
	laser_frame_id = "base_laser_link";
	fixed_frame_id = "map";
	odom_frame_id  = "odom";
	
	marker_seq=0;
    map_seq=0;
    
	GT_sub_ = _nh.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &Graph2RosMap::GT_callback, this);
	first_position=NULL;
	
	GT_trajectory_pub_ =  _nh.advertise<visualization_msgs::Marker>("GT_poses", 10);
}

Graph2RosMap::Graph2RosMap(string laser_frame, string fixed_frame, string odom_frame){
	markers_pub_     =  _nh.advertise<visualization_msgs::Marker>( "SLAM_Graph", 10 );
	map_pub_         =  _nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

	laser_frame_id = laser_frame;
	fixed_frame_id = fixed_frame;
	odom_frame_id  = odom_frame;
	
	marker_seq=0;
    map_seq=0;
    
	GT_sub_ = _nh.subscribe<nav_msgs::Odometry>("base_pose_ground_truth", 1000, &Graph2RosMap::GT_callback, this);
	first_position=NULL;
	
	GT_trajectory_pub_ =  _nh.advertise<visualization_msgs::Marker>("GT_poses", 10);
}


g2o::SE2 Graph2RosMap::listen_tf_odom(){
	
	tf::TransformListener listener;
	tf::StampedTransform transform;
	
	try{
		listener.waitForTransform(odom_frame_id, laser_frame_id, ros::Time(0), ros::Duration(3.0));
		listener.lookupTransform (odom_frame_id, laser_frame_id, ros::Time(0), transform);
	}
	catch (tf::TransformException &ex) {
	  ROS_ERROR("listen  stageros transform %s",ex.what());
	  ros::Duration(1.0).sleep();
	}
	
	
	g2o::SE2 current_odom(transform.getOrigin().x(), transform.getOrigin().y(), tf::getYaw(transform.getRotation() ));
	
	
	return current_odom;
}


int Graph2RosMap::publish_markers( SparseOptimizer *graph) {
	
	visualization_msgs::Marker marker;

	marker.header.frame_id = fixed_frame_id;
	marker.header.seq = marker_seq;
	marker.header.stamp = ros::Time();
	marker_seq++;
	

	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0;
	marker.color.g = 0.5;
	marker.color.b = 0.5;

	
//	marker.points.resize(graph->edges().size());
	marker.points.clear();
	
	OptimizableGraph::EdgeSet eset = graph->edges();

	for(OptimizableGraph::EdgeSet::iterator it = eset.begin();  it!= eset.end(); it++){		
		geometry_msgs::Point point_from, point_to;
		
		EdgeSE2* e_in=dynamic_cast<EdgeSE2*>(*it);
		
		VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e_in->vertices()[0]);
		VertexSE2* vto  =dynamic_cast<VertexSE2*>(e_in->vertices()[1]);
		
		point_from.x = vfrom->estimate().translation().x();
		point_from.y = vfrom->estimate().translation().y();
		point_from.z = double(vfrom->id())/1000;

		
		point_to.x = vto->estimate().translation().x();
		point_to.y = vto->estimate().translation().y();
		point_to.z = double(vto->id())/1000;

		marker.points.push_back(point_from);
		marker.points.push_back(point_to);
		
	}
	
//	EdgeSE2* e=dynamic_cast<EdgeSE2*>(*eset.begin());

	publish_ground_truth();// probably with an "if"
	
	markers_pub_.publish( marker );	

	
	
	return 0;
}


int Graph2RosMap::graph_2_occ( SparseOptimizer *graph) {
	/************************************************************************
	*                          Input handling                              *
	************************************************************************/
	float rows, cols, gain, square_size;
	float resolution, max_range, usable_range, angle, threshold;
//			string g2oFilename, mapFilename;
	
	
	resolution = 0.05f;
	threshold =  0.8;//-1.0;
	rows = cols = 0;
	//	rows = 	cols = 1000;
	max_range = -1.0;
	//	usable_range =  -1.0;
	usable_range =  10;
	gain =  1;
	square_size= 1;
	angle =  0;//M_PI/2;


	



	int size_edges = graph->edges().size();
	ROS_INFO("Size of edges %d", size_edges);
	
	
	
	

				
	// Sort verteces
	vector<int> vertexIds(graph->vertices().size());
	int k = 0;
	for(OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
		vertexIds[k++] = (it->first);
	}  
	sort(vertexIds.begin(), vertexIds.end());
	
	/************************************************************************
	*                          Compute map size                            *
	************************************************************************/
	// Check the entire graph to find map bounding box
	Eigen::Matrix2d boundingBox = Eigen::Matrix2d::Zero();
	std::vector<RobotLaser*> robotLasers;
	std::vector<SE2> robotPoses;
	double xmin=std::numeric_limits<double>::infinity();
	double xmax=-std::numeric_limits<double>::infinity();
	double ymin=std::numeric_limits<double>::infinity();
	double ymax=-std::numeric_limits<double>::infinity();
	
	
	SE2 baseTransform(0,0,angle);
	
	for(size_t i = 0; i < vertexIds.size(); ++i) {
		OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
		VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
		if(!v) { continue; }
		v->setEstimate(baseTransform*v->estimate());
		OptimizableGraph::Data *d = v->userData();
		
		while(d) {
			RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
			if(!robotLaser) {
				d = d->next();
				continue;
			}
			robotLasers.push_back(robotLaser);
			robotPoses.push_back(v->estimate());
			double x = v->estimate().translation().x();
			double y = v->estimate().translation().y();
			
//					cout <<"Point"<< i<<"; x " << x<< ", y "<< y << " and angle "<< v->estimate().rotation().angle() <<endl;
			
			xmax = xmax > x+usable_range ? xmax : x+usable_range;
			ymax = ymax > y+usable_range ? ymax : y+usable_range;
			xmin = xmin < x-usable_range ? xmin : x-usable_range;
			ymin = ymin < y-usable_range ? ymin : y-usable_range;
			
			d = d->next();
		}
	}
	
	boundingBox(0,0)=xmin;
	boundingBox(0,1)=xmax;
	boundingBox(1,0)=ymin;
	boundingBox(1,1)=ymax;
	
	std::cout << "Found " << robotLasers.size() << " laser scans"<< std::endl;
	std::cout << "Bounding box: " << std::endl << boundingBox << std::endl; 
	
	std::cout << " xmin " << xmin<< " ymin " << ymin<< " xmax " << xmax<< " ymax " << ymax << std::endl; 
	
	
	if(robotLasers.size() == 0)  {
		std::cout << "No laser scans found ... quitting!" << std::endl;
		return 0;
	}
	
	/************************************************************************
	*                          Compute the map                             *
	************************************************************************/
	// Create the map
	Eigen::Vector2i size;
	std::cout << " xlength " << (xmax-xmin)/resolution<< " ylength " << (ymax-ymin)/resolution  << std::endl; 
	if(rows != 0 && cols != 0) { size = Eigen::Vector2i(rows, cols); }
	else {
		size = Eigen::Vector2i((boundingBox(0, 1) - boundingBox(0, 0))/ resolution,
			   (boundingBox(1, 1) - boundingBox(1, 0))/ resolution);
	} 
	//  std::cout << "Map size: " << size.transpose() << std::endl;
	if(size.x() == 0 || size.y() == 0) {
		std::cout << "Zero map size ... quitting!" << std::endl;
		return 0;
	}
	
	
	//Eigen::Vector2f offset(-size.x() * resolution / 2.0f, -size.y() * resolution / 2.0f);
	Eigen::Vector2f offset(boundingBox(0, 0),boundingBox(1, 0));
	//  Eigen::Vector2f offset(-18, -36);
	//  Eigen::Vector2f offset(  (xmin+xmax)/2- 25, (ymin+ymax)/2 - 25);
	//  Eigen::Vector2f offset(  (xmin+xmax)/2- size.x()*resolution/2, (ymin+ymax)/2 - size.y()*resolution/2);
	//  Eigen::Vector2f offset(  xmax, ymax);
	FrequencyMapCell unknownCell;
	
	
	
	FrequencyMap map = FrequencyMap(resolution, offset, size, unknownCell);
	
	
	for(size_t i = 0; i < vertexIds.size(); ++i) {
		OptimizableGraph::Vertex *_v = graph->vertex(vertexIds[i]);
		VertexSE2 *v = dynamic_cast<VertexSE2*>(_v);
		if(!v) { continue; }
		
		OptimizableGraph::Data *d = v->userData();
		SE2 robotPose = v->estimate();
		
		while(d) {
			RobotLaser *robotLaser = dynamic_cast<RobotLaser*>(d);
			if(!robotLaser) {
				d = d->next();
				continue;
			}      
		map.integrateScan(robotLaser, robotPose, max_range, usable_range, gain, square_size);
		d = d->next();
		}
	}
	
	
	/************************************************************************
	*                          Save map Occupancy Grid                              *
	************************************************************************/

	nav_msgs::OccupancyGrid map_msg;
	
	map_msg.header.frame_id = fixed_frame_id;	
	map_msg.header.seq = map_seq;
	map_msg.header.stamp = ros::Time();
	map_seq++;	
	
	map_msg.info.resolution = resolution;
	map_msg.info.width = map.rows();
	map_msg.info.height = map.cols();
	
	map_msg.info.origin.position.x = boundingBox(0, 0);
	map_msg.info.origin.position.y = boundingBox(1, 0);
	
	
	std::vector<signed char> data_vector;
	
	for(int c = 0; c < map.cols(); c++) {
		for(int r = 0; r < map.rows(); r++) {
			int value;
			if(map(r, c).misses() == 0 && map(r, c).hits() == 0) {
				value=255;
				data_vector.push_back((char)value);
				  } 
			else {
				float fraction = (float)map(r, c).hits()/(float)(map(r, c).hits()+map(r, c).misses());
				float val = 100*fraction;
				data_vector.push_back((char)val);
	
			}
		}
	}
	
	map_msg.data = data_vector;
	
	
/////////////////////	
	map_pub_.publish(map_msg);
//////////////////////	
	return 0;
}


tf::Transform Graph2RosMap::update_transform(g2o::SE2 optimized, g2o::SE2 odom){	
	
	SE2 adjust = optimized * odom.inverse();
	
	tf::Quaternion q;
	q.setRPY(0, 0, adjust.rotation().angle() );
	
	tf::Transform tf_adjust = tf::Transform(q, tf::Vector3(adjust.translation().x(), adjust.translation().y(), 0));
	
	return tf_adjust;
//	return tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));
}



void Graph2RosMap::GT_callback(const nav_msgs::Odometry::ConstPtr& msg){
	if (first_position==NULL){
		first_position= new geometry_msgs::Point;
		first_position->x = msg->pose.pose.position.x;
		first_position->y = msg->pose.pose.position.y;
		first_position->z = msg->pose.pose.position.z;
		
		geometry_msgs::Point zero;
		zero.x=zero.y=zero.z=0;
//		GT_trajectory.push_back(zero);
	}
	else{
		current_position.y = -(msg->pose.pose.position.x - first_position->x);
		current_position.x =   msg->pose.pose.position.y - first_position->y ;
		current_position.z = 0;

	}
	
//	std::cout <<"GT received "<< std::endl;
		
}



void Graph2RosMap::publish_ground_truth(){
	visualization_msgs::Marker marker;

	marker.header.frame_id = fixed_frame_id;
	marker.header.seq = marker_seq;
	marker.header.stamp = ros::Time();
	marker_seq++;
	

	marker.ns = "my_namespace";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.05;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.5;
	marker.color.g = 0.5;
	marker.color.b = 0.0;
	
	GT_trajectory.push_back(current_position);


	marker.points = GT_trajectory;
	GT_trajectory_pub_.publish( marker );	
}





