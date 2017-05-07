#include <algrob_project/rert.h>

//MAGIC NUMBERS
const std::vector<double> start_pos = {
	0.8,	0.5,	0.20,	0.0,	0.0,	0.0,	0.0,
	-0.8,	0.0,	0.20,	0.0,	0.0,	0.0,	0.0
};
const std::vector<double> ending_pos = {
	0.8,	-0.5,	0.20,	0.0,	0.0,	0.0,	0.0,
	-0.8,	0.0,	0.20,	0.0,	0.0,	0.0,	0.0
};


int main(int argc, char **argv) {

	//Set the RNG at execution, for repeatability.
	srand(321);

	for(int i = 0; i < rapid_random_tree::repetition_timeout; i++){
		std::vector<std::vector<double> > candidate_path = std::vector<std::vector<double> >();
		if(rapid_random_tree::grow_and_find(start_pos, ending_pos, candidate_path)){
			std::printf("Found our trajectory.\n");
			for(int j = 0; j < candidate_path.size(); j++){
				for(int k = 0; k < candidate_path[i].size(); k++){
					std::printf("%f, ", candidate_path[j][k]);
				}
				std::printf("\n");
			}
			return 0;
		}
	}

	std::printf("Algorithm ran out of tries before it could find a path. Exiting in shame.\n");
	return 0;
}
