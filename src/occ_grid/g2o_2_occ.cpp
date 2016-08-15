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
#include "g2o/types/slam2d/parameter_se2_offset.h"

#include "g2o/stuff/command_args.h"



using namespace std;
using namespace Eigen;
using namespace g2o;


int read_and_publish(nav_msgs::OccupancyGrid &map_msg, SparseOptimizer *graph) {
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
	
	
	
	
	
	return 0;
}

