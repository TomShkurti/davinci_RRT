#include <vector>
#include <cstdio>
#include <stdlib.h>
#include <math.h>

#include <algrob_project/joint_angles.h>
#include <algrob_project/graph.h>
#include <algrob_project/davinci_model.h>
#include <algrob_project/tr_intersect.h>

#ifndef ALGROB_RERT
#define ALGROB_RERT

namespace rapid_random_tree{
	//Tunable parameters
	const int expansion_timeout = 4000;
	const int repetition_timeout = 10;
	const float goal_seeking_ratio = 0.15;
	const float step_size [7] = {0.0001, 0.0001, 0.000001, 0.01, 0.01, 0.01, 0.01};

///////////////////MAIN FUNCs///////////////////
	bool grow_and_find(
		const std::vector<double> & init,
		const std::vector<double> & goal,
		std::vector<std::vector<double> > & output_traj
	);
	void solve(
		const std::vector<std::vector<double> > & init_node_cluster,
		const std::vector<std::vector<double> > & goal_node_cluster,
		const std::vector<std::vector<long unsigned int> > & init_edge_cluster,
		const std::vector<std::vector<long unsigned int> > & goal_edge_cluster,
		const int itree_link_index,
		const int gtree_link_index,
		std::vector<std::vector<double> > & output_traj
	);

///////////////////UTIL FUNCS///////////////////
	double dist(const std::vector<double> & p1, const std::vector<double> & p2);
	bool recursive_best_point_finder(
		const std::vector<double> & init_point,
		const std::vector<double> & goal_point,
		std::vector<double> & our_best_point
	);
	bool collide(
		const std::vector<double> & config_init,
		const std::vector<double> & config_goal
	);
	bool triangle_intersection(
		const std::vector<Eigen::Vector3d> & t1,
		const std::vector<Eigen::Vector3d> & t2
	);
	
	std::vector<std::vector<Eigen::Vector3d> > faces_to_triangles(
		const std::vector<std::vector<Eigen::Vector3d> > & one,
		const std::vector<std::vector<Eigen::Vector3d> > & two
	);
}

#endif
