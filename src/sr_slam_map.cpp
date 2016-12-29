// Copyright (c) 2013, Maria Teresa Lazaro Grañon/Leonardo Fermin
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this
//   list of conditions and the following disclaimer in the documentation and/or
//   other materials provided with the distribution.
//
//   Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "g2o/stuff/command_args.h"

#include <string>
#include <sstream> 

#include "mrslam/mr_graph_slam.h"


#include "graph_ros_publisher.h"
#include "ros_handler.h"

//Additional components
#include "occ_grid/g2o_2_occ.h"
#include "uncertainty/uncertainty.h"
#include "uncertainty/graph_distance.hpp"

//tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//ROS
#include "nav_msgs/GetMap.h"

using namespace g2o;

//int find_new_edges( SparseOptimizer *graph, int last_vertex) {
int find_new_edges(   MRGraphSLAM &gslam_, int window_size) {
	int a=1;
	
	OptimizableGraph::VertexIDMap VertexMap = gslam_.graph()->vertices();
	
	int last_edge_id = gslam_.lastVertex()->id();
//	std::cerr << "Last vertex is  "<<   last_edge_id << std::endl;
//	std::cerr << "window_size is  "<<   window_size << std::endl;
	
	int search_window = std::min(last_edge_id+1, window_size);
	
	std::cerr << "The last vertices are  "<<   search_window << std::endl;
	for(int i=0; i < search_window; i ++){
//		std::cerr << last_edge_id-i << " ";
		VertexSE2* vcurrent = dynamic_cast<VertexSE2*>(VertexMap[last_edge_id-i]);
		
		OptimizableGraph::EdgeSet eset = vcurrent->edges();
		for(OptimizableGraph::EdgeSet::iterator it = eset.begin();  it!= eset.end(); it++){					
			EdgeSE2* e_in=dynamic_cast<EdgeSE2*>(*it);
			
			VertexSE2* vfrom=dynamic_cast<VertexSE2*>(e_in->vertices()[0]);
			VertexSE2* vto  =dynamic_cast<VertexSE2*>(e_in->vertices()[1]);
			
			float from_x = vfrom->estimate().translation().x();
			float from_y = vfrom->estimate().translation().y();
			
			float to_x = vto->estimate().translation().x();
			float to_y = vto->estimate().translation().y();
			
			float distance = std::sqrt(  (from_x-to_x)*(from_x-to_x) + (from_y-to_y)*(from_y-to_y) );
			
			std::cerr << "("<< vfrom->id() << ","<< vto->id() << ") with distance: " << distance << std::endl;		
			
		}
		std::cerr << std::endl;
		
	}

	std::vector< std::pair<int, float> > edges_in_new_node;
//	central_vertex_label = Distances.insert_new_node(label_new_node,  edges_in_new_node);

	
	/*
	std::cerr << "The vertices are ";
	for(OptimizableGraph::VertexIDMap::iterator it = VertexMap.begin();  it!= VertexMap.end(); it++){		
		int current_id = it->first;
		VertexSE2* vcurrent = dynamic_cast<VertexSE2*>(it->second);		
		std::cerr << " "<<current_id;		
	}
	//*/
	std::cerr <<std::endl;
	




	return a;
}



int main(int argc, char **argv)
{

  CommandArgs arg;
  double resolution;
  double maxScore, maxScoreMR;
  double kernelRadius;
  int  minInliers, minInliersMR;
  int windowLoopClosure, windowMRLoopClosure;
  double inlierThreshold;
  int idRobot;
  int nRobots;
  std::string outputFilename;
  std::string odometryTopic, scanTopic, fixedFrame, odomFrame;
  
//  ros::Publisher map_pub_ =  n.advertise<nav_msgs::OccupancyGrid>("map", 10);

  arg.param("resolution",  resolution, 0.025, "resolution of the matching grid");
  arg.param("maxScore",    maxScore, 0.15,     "score of the matcher, the higher the less matches");
  arg.param("kernelRadius", kernelRadius, 0.2,  "radius of the convolution kernel");
  arg.param("minInliers",    minInliers, 5,     "min inliers");
  arg.param("windowLoopClosure",  windowLoopClosure, 10,   "sliding window for loop closures");
  arg.param("inlierThreshold",  inlierThreshold, 2.,   "inlier threshold");
  arg.param("idRobot", idRobot, 0, "robot identifier" );
  arg.param("nRobots", nRobots, 1, "number of robots" );
  arg.param("maxScoreMR",    maxScoreMR, 0.15,  "score of the intra-robot matcher, the higher the less matches");
  arg.param("minInliersMR",    minInliersMR, 5,     "min inliers for the intra-robot loop closure");
  arg.param("windowMRLoopClosure",  windowMRLoopClosure, 10,   "sliding window for the intra-robot loop closures");
  arg.param("odometryTopic", odometryTopic, "odom", "odometry ROS topic");
  arg.param("scanTopic", scanTopic, "base_scan", "scan ROS topic");
  arg.param("fixedFrame", fixedFrame, "map", "fixed frame to visualize the graph with ROS Rviz");
  arg.param("odomFrame", odomFrame, "odom", "frame reference for odometry");
  arg.param("o", outputFilename, "", "file where to save output");
  arg.parseArgs(argc, argv);

  ros::init(argc, argv, "real_srslam_occ");

  RosHandler rh(idRobot, nRobots, REAL_EXPERIMENT);
  rh.setOdomTopic(odometryTopic);
  rh.setScanTopic(scanTopic);
  rh.useOdom(true);
  rh.useLaser(true);


  rh.init();   //Wait for initial odometry and laserScan
  rh.run();

  tf::Transform  g2o_transform = tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0));	
 
  //For estimation
  SE2 currEst = rh.getOdom();
  std::cout << "My initial position is: " << currEst.translation().x() << " " << currEst.translation().y() << " " << currEst.rotation().angle() << std::endl;
  SE2 odomPosk_1 = currEst;
  std::cout << "My initial odometry is: " << odomPosk_1.translation().x() << " " << odomPosk_1.translation().y() << " " << odomPosk_1.rotation().angle() << std::endl;

  //Graph building
  MRGraphSLAM gslam;
  gslam.setIdRobot(idRobot);
  int baseId = 10000;
  gslam.setBaseId(baseId);
  gslam.init(resolution, kernelRadius, windowLoopClosure, maxScore, inlierThreshold, minInliers);
  gslam.setInterRobotClosureParams(maxScoreMR, minInliersMR, windowMRLoopClosure);

  RobotLaser* rlaser;// = rh.getLaser();

  gslam.setInitialData(odomPosk_1, rlaser);

  GraphRosPublisher graphPublisher(gslam.graph(), fixedFrame);
  Graph_Distance distance_graph;

  Graph2RosMap g2map(rh.laser().header.frame_id, fixedFrame, odomFrame);
  GraphUncertainty uncertain(gslam.graph());

  ros::Rate loop_rate(10);
  
  bool first_publish=true;
  int cycles=0;


  while (ros::ok()){
    ros::spinOnce();
	

//    SE2 odomPosk = rh.getOdom(); //current odometry
    SE2 odomPosk = g2map.listen_tf_odom(); //current odometry
    SE2 relodom = odomPosk_1.inverse() * odomPosk;
    currEst *= relodom;

    odomPosk_1 = odomPosk;

    if((distanceSE2(gslam.lastVertex()->estimate(), currEst) > 0.25) || 
       (fabs(gslam.lastVertex()->estimate().rotation().angle()-currEst.rotation().angle()) > M_PI_4)){
      //Add new data
      RobotLaser* laseri = rh.getLaser();

      gslam.addDataSM(odomPosk, laseri);
      gslam.findConstraints();
      gslam.findInterRobotConstraints();

//	  std::cerr << "Last vertex " << gslam.lastVertex()->id()<< std::endl ;
//	  find_new_edges(gslam.graph(), gslam.lastVertex()->id());
	  find_new_edges(gslam, windowLoopClosure);
		std::cout << " after finding "<<std::endl;


	  
	  
      gslam.optimize(5);
	  
      currEst = gslam.lastVertex()->estimate();

	  g2map.graph_2_occ(gslam.graph());
      g2map.publish_markers(gslam.graph());
      
      g2o_transform = g2map.update_transform(currEst, odomPosk);
      

      //Publish graph to visualize it on Rviz
      graphPublisher.publishGraph();
      
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
//      gslam.saveGraph(buf);
    }



   		static tf::TransformBroadcaster tf_broadcaster;
		tf::StampedTransform map_transform = tf::StampedTransform(g2o_transform, ros::Time::now(), fixedFrame, odomFrame);
		tf_broadcaster.sendTransform(map_transform);

	///Improve
	if( (cycles >5) && first_publish){
		g2map.graph_2_occ(gslam.graph());
		graphPublisher.publishGraph();
	}
	else if (first_publish) cycles++;
    ///
    
    loop_rate.sleep();
  }
  
  return 0;
}
