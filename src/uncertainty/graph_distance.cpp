#include "graph_distance.hpp"


///////////////////

void Graph_Distance::clean_class(){
	for(std::unordered_map<int, Vertex*>::iterator map_iter = Vertices_map.begin(); map_iter != Vertices_map.end(); map_iter ++){
		Vertex* ptr = map_iter->second;
	    delete  ptr;	
	}
	Vertices_map.clear();

	for(std::map<std::set<int>, Arc*>::iterator map_iter = Arcs_map.begin(); map_iter != Arcs_map.end(); map_iter ++){
		Arc* ptr = map_iter->second;
	    delete  ptr;	
	}
	Arcs_map.clear();
}
//
Graph_Distance::~Graph_Distance(){
	clean_class();
}


std::ostream& operator<<(std::ostream& os, Graph_Distance& Graph){
	os << "\n";	
	os << "Number of Nodes "<< Graph.Vertices_map.size() << "\n";
	os << "Number of Edges "<< Graph.Arcs_map.size()     << "\n";

/*
	for(std::unordered_map <int, std::unordered_map <int,float> >::iterator map_map_iter = Graph.map_distance_matrix.begin(); map_map_iter != Graph.map_distance_matrix.end(); map_map_iter ++){
		int label_1 = map_map_iter->first;
		std::unordered_map <int,float>  sub_map = map_map_iter->second;
		for(std::unordered_map <int,float>::iterator  map_iter = sub_map.begin(); map_iter != sub_map.end(); map_iter ++){
			int label_2 = map_iter->first;
			float current_distance = map_iter->second;
			os << "("<< label_1<<","<<label_2<<")-> "<<  current_distance << ";\t";
		}		
	}
//*/


    return os;
}




///////////////////////////////////////



///////
void Graph_Distance::insert_first_edge(int label_first, int label_second, float distance){
	clean_class();
	Vertex* first_vertex = new Vertex;
	Vertex* second_vertex = new Vertex;
	
	Vertices_map[label_first]  = first_vertex;
	Vertices_map[label_second] = second_vertex;
		
	first_vertex->label  = label_first;
	second_vertex->label = label_second;

	Arc* first_arc = new Arc;

	first_arc->labels={label_first, label_second};
	first_arc->distance =distance;
	
	first_vertex ->connections.push_back(first_arc);
	second_vertex->connections.push_back(first_arc);
	
	Arcs_map[first_arc->labels] = first_arc;


	
}
//////


int Graph_Distance::insert_new_node(int label, std::vector< std::pair<int, float> > Connections_label_distance ){
	std::unordered_map<int, Vertex*>::iterator found_ptr =  Vertices_map.find(label);
	if(found_ptr != Vertices_map.end() ){
		std::cout << "Vertex label already in graph" << std::endl;
		return -1;
	}
	

	Vertex* vertex_new = new Vertex;
	Vertices_map[label]  = vertex_new;
	
	vertex_new->label=label;
	//Insert all edges
	for(int i=0; i < Connections_label_distance.size(); i++){
		Arc* current_arc = new Arc;
		int TO_label = Connections_label_distance[i].first;
		
		current_arc->labels={label, TO_label};
		current_arc->distance = Connections_label_distance[i].second;
		
		vertex_new            ->connections.push_back(current_arc);
		Vertices_map[TO_label]->connections.push_back(current_arc);		
		
		Arcs_map[current_arc->labels] = current_arc;
	}
	////
	return update_distances( label );
}
//////


int Graph_Distance::insert_new_edge(int label_1, int label_2,  float distance ){
	std::unordered_map<int, Vertex*>::iterator found_ptr =  Vertices_map.find(label_1);
	if(found_ptr == Vertices_map.end() ){
		std::cout << "Vertex 1 label NOT in graph" << std::endl;
		return -1;
	}
	found_ptr =  Vertices_map.find(label_2);
	if(found_ptr == Vertices_map.end() ){
		std::cout << "Vertex 2 label NOT in graph" << std::endl;
		return -1;
	}
	
	Arc* current_arc = new Arc;
	
	current_arc->labels={label_1, label_2};
	current_arc->distance = distance;
	
	Vertices_map[label_1]->connections.push_back(current_arc);		
	Vertices_map[label_2]->connections.push_back(current_arc);		
	
	Arcs_map[current_arc->labels] = current_arc;
	
	return update_distances( label_1 );
}




int Graph_Distance::update_distances(	int label_start_node ){
	

	
	std::unordered_map<int, Vertex*> unvisited_vertices = Vertices_map; //List to work
	// Set initial conditions
	for(std::unordered_map<int, Vertex*>::iterator map_iter = unvisited_vertices.begin(); map_iter != unvisited_vertices.end(); map_iter ++){
		map_iter->second->distance_in_this_iter = std::numeric_limits<float>::infinity();
	}

	Vertices_map[label_start_node]->distance_in_this_iter = 0;
	unvisited_vertices[label_start_node]->distance_in_this_iter = 0;

	while(unvisited_vertices.size() > 0){
		std::unordered_map<int, Vertex*>::iterator eliminate_iter = unvisited_vertices.begin();

		//Find minimum
		float min_distance = std::numeric_limits<float>::infinity();
		for(std::unordered_map<int, Vertex*>::iterator map_iter = unvisited_vertices.begin(); map_iter != unvisited_vertices.end(); map_iter ++){
			if( map_iter->second->distance_in_this_iter < min_distance ){
				eliminate_iter = map_iter;
				min_distance = map_iter->second->distance_in_this_iter;
			}
		}
		
		Vertex* current_vertex = eliminate_iter->second;
		unvisited_vertices.erase(eliminate_iter); //remove from unvisited
		
		//Expand the edges
		for(int i=0; i < current_vertex->connections.size();i++){
			float new_distance = current_vertex->distance_in_this_iter  +  current_vertex->connections[i]->distance;

			int destination_label = *current_vertex->connections[i]->labels.begin();
			if(destination_label == current_vertex->label)   destination_label = *current_vertex->connections[i]->labels.rbegin();

			if(new_distance < Vertices_map[destination_label] -> distance_in_this_iter ){
				Vertices_map[destination_label] -> distance_in_this_iter = new_distance;
			}
			//
		}
		//
	}

	return update_distance_matrix(label_start_node);
}



int Graph_Distance::update_distance_matrix(int label_new_node){
	//Create rows and columns (if any)
	for(std::unordered_map<int, Vertex*>::iterator map_iter = Vertices_map.begin(); map_iter != Vertices_map.end(); map_iter ++){
		
		int lower_label =  (label_new_node >= map_iter->first)? map_iter->first : label_new_node; 
		int upper_label =  (lower_label == label_new_node)? map_iter->first :  label_new_node; 

		map_distance_matrix [lower_label][upper_label] = map_iter->second->distance_in_this_iter;		
	}
	
	
	//Update the rest
	for(std::unordered_map <int, std::unordered_map <int,float> >::iterator map_map_iter = map_distance_matrix.begin(); map_map_iter != map_distance_matrix.end(); map_map_iter ++){
		int lower_label = map_map_iter->first;
		std::unordered_map <int,float>  sub_map = map_map_iter->second;
		
		for(std::unordered_map <int,float>::iterator  map_iter = sub_map.begin(); map_iter != sub_map.end(); map_iter ++){
			int upper_label = map_iter->first;
			float current_distance = map_iter->second;

			int first_low  =  (label_new_node >= lower_label)? lower_label : label_new_node; 
			int first_high =  (lower_label == label_new_node)? lower_label :  label_new_node; 
			
			int second_low  =  (label_new_node >= upper_label)? upper_label : label_new_node; 
			int second_high =  (lower_label == label_new_node)? upper_label :  label_new_node; 

			
			float distance_alternative = map_distance_matrix [first_low][first_high]  +   map_distance_matrix [second_low][second_high];
			if (distance_alternative < map_distance_matrix [lower_label][upper_label]){
				map_distance_matrix [lower_label][upper_label]  =  distance_alternative;
			}
			//
		}				
		//
	}
	////
	return  extract_central_vertex_label();
	
}



int Graph_Distance::extract_central_vertex_label(){

	std::unordered_map<int, float> max_dist_map;
	
	for(std::unordered_map<int, Vertex*>::iterator map_iter = Vertices_map.begin(); map_iter != Vertices_map.end(); map_iter ++){
		int label_1 = map_iter->first;
		max_dist_map[label_1] = -1;
		
		for(std::unordered_map<int, Vertex*>::iterator map_iter_2 = Vertices_map.begin(); map_iter_2 != Vertices_map.end(); map_iter_2 ++){
			int label_2 = map_iter_2->first;
			
			int lower_label =  (label_1 >= label_2)? label_2 : label_1; 
			int upper_label =  (lower_label == label_1)? label_2 :  label_1; 
			
			float distance = map_distance_matrix [lower_label][upper_label];
			if(distance >max_dist_map[label_1]) max_dist_map[label_1]=distance;
			
		}
	}
	////
//	std::cout << "     Largest Distances: " <<   std::endl;
	float min_dist = std::numeric_limits<float>::infinity();
	int central_label=-1;
	for(std::unordered_map<int, float>::iterator map_iter = max_dist_map.begin(); map_iter != max_dist_map.end(); map_iter ++){
//		std::cout << "     node: "<<  map_iter->first <<",  distance "<< map_iter->second <<   std::endl;
		if(map_iter->second < min_dist){
			central_label = map_iter->first;
			min_dist = map_iter->second;
		}
	}
//	std::cout << "     Central vertex: "<< central_label <<   std::endl;
	
	return central_label;
}



bool Graph_Distance::is_node_in_graph(int label){
	std::unordered_map<int, Vertex*>::iterator vertex_iter =  Vertices_map.find(label) ;
	bool inside = (vertex_iter == Vertices_map.end() )? false : true;
	return inside;
}





bool Graph_Distance::is_edge_in_graph(std::set<int> labels){
	std::map<std::set<int>, Arc*>::iterator arc_iter = Arcs_map.find(labels);
	bool inside = (arc_iter == Arcs_map.end() )? false : true;
	return inside;
}






