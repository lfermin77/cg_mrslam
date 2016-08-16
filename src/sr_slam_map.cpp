// Copyright (c) 2013, Maria Teresa Lazaro Grañon
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
#include "mrslam/graph_comm.h"

#include "graph_ros_publisher.h"
#include "ros_handler.h"
#include "occ_grid/g2o_2_occ.h"

//tf
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//ROS
#include "nav_msgs/GetMap.h"

using namespace g2o;

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
  std::string odometryTopic, scanTopic, fixedFrame;
  
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

  RobotLaser* rlaser = rh.getLaser();

  gslam.setInitialData(odomPosk_1, rlaser);

  GraphRosPublisher graphPublisher(gslam.graph(), fixedFrame);

  Graph2RosMap g2map;

  ////////////////////
  //Setting up network
  std::string base_addr = "192.168.0.";
  GraphComm gc(&gslam, idRobot, nRobots, base_addr, REAL_EXPERIMENT);
  gc.init_network(&rh);

  ros::Rate loop_rate(10);

  while (ros::ok()){
    ros::spinOnce();

    SE2 odomPosk = rh.getOdom(); //current odometry
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

      gslam.optimize(5);
      
      nav_msgs::OccupancyGrid map_msg;	
	  map_msg.header = rh.laser().header;
	  
	  g2map.graph_2_occ(map_msg, gslam.graph());

      currEst = gslam.lastVertex()->estimate();
      
      g2o_transform = g2map.update_transform(currEst, odomPosk);
      
      visualization_msgs::Marker marker;
	  marker.header = rh.laser().header;
      g2map.publish_markers(marker,gslam.graph());

      //Publish graph to visualize it on Rviz
      graphPublisher.publishGraph();
      
      char buf[100];
      sprintf(buf, "robot-%i-%s", idRobot, outputFilename.c_str());
//      gslam.saveGraph(buf);


      

    }

   		static tf::TransformBroadcaster tf_broadcaster;
		tf::StampedTransform map_transform = tf::StampedTransform(g2o_transform, ros::Time::now(), "map", "odom");
		tf_broadcaster.sendTransform(map_transform);
    
    loop_rate.sleep();
  }
  
  return 0;
}
