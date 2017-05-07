#include <vector>
#include <climits>

#ifndef ALGROB_GRAPH
#define ALGROB_GRAPH

struct node{
	std::vector<int> neighbors = std::vector<int>();
	node * previous = NULL;
	int dist = INT_MAX - 1;//Setting to INT_MAX itself would cause overflows when +1.
	const std::vector<double> * config_pointer = NULL;
	bool marked = false;
	bool goal = false;
};

#endif
