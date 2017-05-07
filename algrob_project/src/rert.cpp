#include <algrob_project/rert.h>

bool rapid_random_tree::grow_and_find(
	const std::vector<double> & init,
	const std::vector<double> & goal,
	std::vector<std::vector<double> > & output_traj
){
	
	std::vector<std::vector<double> > init_node_cluster = std::vector<std::vector<double> >();
	std::vector<std::vector<double> > goal_node_cluster = std::vector<std::vector<double> >();
	init_node_cluster.push_back(init);
	goal_node_cluster.push_back(goal);
	std::vector<std::vector<long unsigned int> > init_edge_cluster = std::vector<std::vector<long unsigned int> >();
	std::vector<std::vector<long unsigned int> > goal_edge_cluster = std::vector<std::vector<long unsigned int> >();

	for(int k = 0; k < expansion_timeout; k++){
		//Should we expand randomly, or point ourselves towards the goal?
		if((float)(rand()) / (float)(RAND_MAX) >= goal_seeking_ratio){
			//Generate a random point:
			std::vector<double> r_config = std::vector<double>();
			r_config.resize(14);
			for(int i = 0; i < 7; i++){
				r_config[i]	= (((double)(rand()) / (double)(RAND_MAX)) * (joint_lims_max[i] - joint_lims_min[i])) + joint_lims_min[i];
				r_config[i + 7]	= (((double)(rand()) / (double)(RAND_MAX)) * (joint_lims_max[i] - joint_lims_min[i])) + joint_lims_min[i];
			}

			//Find the closest point thereto:
			long unsigned int closest_ind = 0;
			double closest_dst = rapid_random_tree::dist(init_node_cluster[0], r_config);
			for(int i = 1; i < init_node_cluster.size(); i++){
				double candidate_dst = rapid_random_tree::dist(init_node_cluster[i], r_config);
				if(candidate_dst < closest_dst){
					closest_dst = candidate_dst;
					closest_ind = i;
				}
			}
			
			//Check our path for collisions recursively and get as far as we can.
			std::vector<double> init_point = init_node_cluster[closest_ind];
			std::vector<double> best_effort = std::vector<double>();
			recursive_best_point_finder(init_point, r_config, best_effort);
			init_node_cluster.push_back(best_effort);
			init_edge_cluster.push_back({closest_ind, init_node_cluster.size() - 1});

			//Rinse and repeat for the goal side:
			r_config = std::vector<double>();
			r_config.resize(14);
			for(int i = 0; i < 7; i++){
				r_config[i]		= (((double)(rand()) / (double)(RAND_MAX)) * (joint_lims_max[i] - joint_lims_min[i])) + joint_lims_min[i];
				r_config[i + 7]	= (((double)(rand()) / (double)(RAND_MAX)) * (joint_lims_max[i] - joint_lims_min[i])) + joint_lims_min[i];
			}
			closest_ind = 0;
			closest_dst = rapid_random_tree::dist(goal_node_cluster[0], r_config);
			for(int i = 1; i < goal_node_cluster.size(); i++){
				double candidate_dst = rapid_random_tree::dist(goal_node_cluster[i], r_config);
				if(candidate_dst < closest_dst){
					closest_dst = candidate_dst;
					closest_ind = i;
				}
			}

			//Check our path for collisions recursively and get as far as we can.
			init_point = goal_node_cluster[closest_ind];
			best_effort = std::vector<double>();
			recursive_best_point_finder(init_point, r_config, best_effort);
			goal_node_cluster.push_back(best_effort);
			goal_edge_cluster.push_back({closest_ind, goal_node_cluster.size() - 1});

		} else{
			//Advance towards our goal.
			//First, find the pair of points in the start and goal trees that are closest together
			long unsigned int gtree_index = 0;
			long unsigned int itree_index = 0;
			double min_dst = rapid_random_tree::dist(init_node_cluster[0], goal_node_cluster[0]);
			for(int i = 0; i < init_node_cluster.size(); i++){
				for(int j = 0; j < goal_node_cluster.size(); j++){
					double can_dst = rapid_random_tree::dist(init_node_cluster[i], goal_node_cluster[j]);
					if(can_dst < min_dst){
						itree_index = i;
						gtree_index = j;
						min_dst = can_dst;
					}
				}
			}
			//Project along a line towards them.
			std::vector<double> init_point = init_node_cluster[itree_index];
			std::vector<double> goal_point = goal_node_cluster[gtree_index];
			
			std::vector<double> best_effort = std::vector<double>();
			bool collided = recursive_best_point_finder(init_point, goal_point, best_effort);
			if(collided){
				init_node_cluster.push_back(best_effort);
				init_edge_cluster.push_back({itree_index, init_node_cluster.size() - 1});
			}
			else{
				for(int i = 0; i < init_edge_cluster.size(); i++){
				}
				for(int i = 0; i < goal_edge_cluster.size(); i++){
				}
				solve(
					init_node_cluster,
					goal_node_cluster,
					init_edge_cluster,
					goal_edge_cluster,
					itree_index,
					gtree_index,
					output_traj
				);
				return true;
			}
		}
	}

	printf("Tree timed out.");
	return false;
}

bool rapid_random_tree::recursive_best_point_finder(
	const std::vector<double> & init_point,
	const std::vector<double> & goal_point,
	std::vector<double> & our_best_point
){
	//Terminal case 1: No collisions all along our path.
	//Return the goal point (as we can get to it unhindered) and a successful result.
	if(!collide(init_point, goal_point)){
		our_best_point = goal_point;
		return false;
	}
	//Terminal case 2: Collision, but we are at minimum step size.
	//Return our start point and an unsuccessful result.
	bool within_step_size = true;
	for(int i = 0; i < init_point.size(); i++){
		if(abs(goal_point[i] - init_point[i]) > step_size[i]){
			within_step_size = false;
			break;
		}
	}
	if(within_step_size){
		our_best_point = init_point;
		return true;
	}
	
	//Otherwise, split our result into halves. 
	std::vector<double> halfway_point = std::vector<double>();
	halfway_point.resize(14);
	for(int i = 0; i < 14; i++){
		halfway_point[i] = (init_point[i] + goal_point[i]) / 2.0;
	}
	
	//Get whether there is a collision in the first half and how far we got.
	std::vector<double> best_first = std::vector<double>();
	best_first.resize(14);
	bool collision_first = recursive_best_point_finder(init_point, halfway_point, best_first);
	//If there IS a collision, return that point.
	if(collision_first){
		our_best_point = best_first;
		return true;
	}
	//Otherwise, return how far we got on the other end.
	return recursive_best_point_finder(halfway_point, goal_point, our_best_point);		
}

double rapid_random_tree::dist(const std::vector<double> & p1, const std::vector<double> & p2){
	double sq_sum = 0;
	for(int i = 0; i < 14; i++){
		sq_sum = sq_sum + pow(p1[i] - p2[i], 2);
	}
	return sqrt(sq_sum);
}

bool rapid_random_tree::collide(const std::vector<double> & config_init, const std::vector<double> & config_goal){
	std::vector<std::vector<Eigen::Vector3d> > a1_part1_squares = coarse_model::getAllFaces(config_init, true);
	std::vector<std::vector<Eigen::Vector3d> > a2_part1_squares = coarse_model::getAllFaces(config_init, false);
	std::vector<std::vector<Eigen::Vector3d> > a1_part2_squares = coarse_model::getAllFaces(config_goal, true);
	std::vector<std::vector<Eigen::Vector3d> > a2_part2_squares = coarse_model::getAllFaces(config_goal, false);
	
	//Convert the squares into the relevant triangles
	std::vector<std::vector<Eigen::Vector3d> > a1_triangles = faces_to_triangles(a1_part1_squares, a1_part2_squares);
	std::vector<std::vector<Eigen::Vector3d> > a2_triangles = faces_to_triangles(a2_part1_squares, a2_part2_squares);
	
	for(int i = 0; i < a1_triangles.size(); i++){
		for(int j = i; j < a2_triangles.size(); j++){
			if(triangle_intersection(a1_triangles[i], a2_triangles[i])){
				return true;
			}
		}
	}
	return false;
}

std::vector<std::vector<Eigen::Vector3d> > rapid_random_tree::faces_to_triangles(
	const std::vector<std::vector<Eigen::Vector3d> > & one,
	const std::vector<std::vector<Eigen::Vector3d> > & two
){
	std::vector<std::vector<Eigen::Vector3d> > triangles = std::vector<std::vector<Eigen::Vector3d> >();
	for(int i = 0; i < one.size(); i++){
		std::vector<Eigen::Vector3d> triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][0]);
		triangle.push_back(two[i][0]);
		triangle.push_back(two[i][2]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][0]);
		triangle.push_back(two[i][2]);
		triangle.push_back(one[i][2]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][0]);
		triangle.push_back(two[i][0]);
		triangle.push_back(two[i][1]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][0]);
		triangle.push_back(one[i][1]);
		triangle.push_back(two[i][1]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][1]);
		triangle.push_back(two[i][1]);
		triangle.push_back(two[i][3]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][1]);
		triangle.push_back(one[i][3]);
		triangle.push_back(two[i][3]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][2]);
		triangle.push_back(two[i][2]);
		triangle.push_back(two[i][3]);
		triangles.push_back(triangle);
		
		triangle = std::vector<Eigen::Vector3d>();
		triangle.push_back(one[i][2]);
		triangle.push_back(one[i][3]);
		triangle.push_back(two[i][3]);
		triangles.push_back(triangle);
	}
	return triangles;
}

bool rapid_random_tree::triangle_intersection(
	const std::vector<Eigen::Vector3d> & t1,
	const std::vector<Eigen::Vector3d> & t2
){
	double v1 [3] = {t1[0][0], t1[0][1], t1[0][2]};
	double v2 [3] = {t1[1][0], t1[1][1], t1[1][2]};
	double v3 [3] = {t1[2][0], t1[2][1], t1[2][2]};
	
	double u1 [3] = {t2[0][0], t2[0][1], t2[0][2]};
	double u2 [3] = {t2[1][0], t2[1][1], t2[1][2]};
	double u3 [3] = {t2[2][0], t2[2][1], t2[2][2]};
	return tri_intersect::tri_tri_intersect(v1, v2, v3, u1, u2, u3) == 1;
}

void rapid_random_tree::solve(
	const std::vector<std::vector<double> > & init_node_cluster,
	const std::vector<std::vector<double> > & goal_node_cluster,
	const std::vector<std::vector<long unsigned int> > & init_edge_cluster,
	const std::vector<std::vector<long unsigned int> > & goal_edge_cluster,
	const int itree_link_index,
	const int gtree_link_index,
	std::vector<std::vector<double> > & output_traj
){
	//Convert the graph into a more easily searchable form.
	//Consolidate the nodes.
	std::vector<node> all_nodes = std::vector<node>();
	for(int i = 0; i < init_node_cluster.size(); i++){
		node n;
		n.config_pointer = & (init_node_cluster[i]);
		all_nodes.push_back(n);
	}
	all_nodes[0].dist = 0;
	node goal;
	goal.config_pointer = & (goal_node_cluster[0]);
	goal.goal = true;
	all_nodes.push_back(goal);
	for(int i = 1; i < goal_node_cluster.size(); i++){
		node n;
		n.config_pointer = & (goal_node_cluster[i]);
		all_nodes.push_back(n);
	}
	//Add the edges.
	for(int i = 0; i < init_edge_cluster.size(); i++){
		int ni_1 = init_edge_cluster[i][0];
		int ni_2 = init_edge_cluster[i][1];
		all_nodes[ni_1].neighbors.push_back(ni_2);
		all_nodes[ni_2].neighbors.push_back(ni_1);
	}
	for(int i = 0; i < goal_edge_cluster.size(); i++){
		int ni_1 = goal_edge_cluster[i][0] + init_node_cluster.size();
		int ni_2 = goal_edge_cluster[i][1] + init_node_cluster.size();
		all_nodes[ni_1].neighbors.push_back(ni_2);
		all_nodes[ni_2].neighbors.push_back(ni_1);
	}
	all_nodes[itree_link_index].neighbors.push_back(
		gtree_link_index + init_node_cluster.size()
	);
	all_nodes[gtree_link_index + init_node_cluster.size()].neighbors.push_back(
		itree_link_index
	);
	
	//Use Dijkstra's algorithm to find a path of nodes between start and end.
	std::vector<node *> q = std::vector<node *>();
	for(int i = 0; i < all_nodes.size(); i++){
		q.push_back(&(all_nodes[i]));
	}
	while(q.size() > 0){
		node * candidate = q[0];
		q.erase(q.begin());
		int alt = candidate->dist + 1;
		for(int i = 0; i < candidate->neighbors.size(); i++){
			node * neighbor = &(all_nodes[candidate->neighbors[i]]);
			if(alt < neighbor->dist){
				neighbor->dist = alt;
				neighbor->previous = candidate;
			}
		}
	}
	
	//Back-trace the nodes and use them to produce a path.
	node * descendent = &(all_nodes[init_node_cluster.size()]);
	while(descendent != NULL){
		output_traj.push_back(*(descendent->config_pointer));
		descendent = descendent->previous;
	}
}
