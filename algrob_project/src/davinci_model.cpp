#include <algrob_project/davinci_model.h>

std::vector<std::vector<Eigen::Vector3d> > coarse_model::getAllFaces(const std::vector<double> & j_angles, bool psm_1){

	//Perform forward kinematics transforms on the arm.
	//The commented-out print statements produce point outputs already formatted in matlab for use with the plotter.m script.
	
	Eigen::Affine3d root_transformation_chain;
	std::vector<std::vector<Eigen::Vector3d> > square_points = std::vector<std::vector<Eigen::Vector3d> >();

	long unsigned int offset;
	int num;
	if(psm_1){
		offset = 0;
		num = 1;
		root_transformation_chain = coarse_model::base__to__outer_yaw_joint__1;
	} else{
		offset = 7;
		num = 2;
		root_transformation_chain = coarse_model::base__to__outer_yaw_joint__2;
	}
	
	//Red Box
	Eigen::Affine3d red_box_chain =
		root_transformation_chain *
		Eigen::AngleAxis<double>(j_angles[offset + 0], Eigen::Vector3d::UnitY()) *
		base_pivot_point__to__red_box_center
	;
	
	std::vector<Eigen::Vector3d> red_box_transformed = std::vector<Eigen::Vector3d>();
	//printf("%%Red Box\n");
	for(int i = 0; i < 8; i++){
		red_box_transformed.push_back(red_box_chain * coarse_model::red_box[i]);
		//printf("p%d_r%d = [%f, %f, %f];\n", i+1, num, red_box_transformed[i][0], red_box_transformed[i][1], red_box_transformed[i][2]);
	}
	square_points.push_back(red_box_transformed);
	
	//Orange Box
	Eigen::Affine3d orange_box_chain =
		red_box_chain *
		coarse_model::red_box_center__to__orange_box_center
	;
	std::vector<Eigen::Vector3d> orange_box_transformed = std::vector<Eigen::Vector3d>();
	//printf("%%Orange Box\n");
	for(int i = 0; i < 8; i++){
		orange_box_transformed.push_back(orange_box_chain * coarse_model::orange_box[i]);
		//printf("p%d_o%d = [%f, %f, %f];\n", i+1, num, orange_box_transformed[i][0], orange_box_transformed[i][1], orange_box_transformed[i][2]);
	}
	square_points.push_back(orange_box_transformed);
	
	//Yellow Box
	Eigen::Affine3d yellow_box_chain =
		orange_box_chain *
		coarse_model::orange_center__to__yellow_pivot_point * 
		Eigen::AngleAxis<double>(j_angles[offset + 1], Eigen::Vector3d::UnitX()) *
		coarse_model::yellow_pivot_point__to__yellow_center
	;
	std::vector<Eigen::Vector3d> yellow_box_transformed = std::vector<Eigen::Vector3d>();
	//printf("%%Yellow Box\n");
	for(int i = 0; i < 8; i++){
		yellow_box_transformed.push_back(yellow_box_chain * coarse_model::yellow_box[i]);
		//printf("p%d_y%d = [%f, %f, %f];\n", i+1, num, yellow_box_transformed[i][0], yellow_box_transformed[i][1], yellow_box_transformed[i][2]);
	}
	square_points.push_back(yellow_box_transformed);
	
	//Green Box
	Eigen::Affine3d green_box_chain =
		yellow_box_chain *
		coarse_model::yellow_box_center__to__green_box_center *
		Eigen::Translation3d(0.0, 0.0, -j_angles[offset + 2])
	;
	
	std::vector<Eigen::Vector3d> green_box_transformed = std::vector<Eigen::Vector3d>();
	//printf("%%Green Box\n");
	for(int i = 0; i < 8; i++){
		green_box_transformed.push_back(green_box_chain * coarse_model::green_box[i]);
		//printf("p%d_g%d = [%f, %f, %f];\n", i+1, num, green_box_transformed[i][0], green_box_transformed[i][1], green_box_transformed[i][2]);
	}
	square_points.push_back(green_box_transformed);
	
	
	//Once we have all of these transformed points, convert them into triangles and add them to one big vector.
	std::vector<std::vector<Eigen::Vector3d> > squares = std::vector<std::vector<Eigen::Vector3d> >();
	for(int i = 0; i < square_points.size(); i++){
		std::vector<Eigen::Vector3d> square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][0]);
		square.push_back(square_points[i][1]);
		square.push_back(square_points[i][2]);
		square.push_back(square_points[i][3]);
		squares.push_back(square);
		
		square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][1]);
		square.push_back(square_points[i][3]);
		square.push_back(square_points[i][4]);
		square.push_back(square_points[i][7]);
		squares.push_back(square);
		
		square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][0]);
		square.push_back(square_points[i][2]);
		square.push_back(square_points[i][4]);
		square.push_back(square_points[i][6]);
		squares.push_back(square);
		
		square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][2]);
		square.push_back(square_points[i][3]);
		square.push_back(square_points[i][6]);
		square.push_back(square_points[i][7]);
		squares.push_back(square);
		
		square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][0]);
		square.push_back(square_points[i][1]);
		square.push_back(square_points[i][4]);
		square.push_back(square_points[i][6]);
		squares.push_back(square);
		
		square = std::vector<Eigen::Vector3d>();
		square.push_back(square_points[i][4]);
		square.push_back(square_points[i][5]);
		square.push_back(square_points[i][6]);
		square.push_back(square_points[i][7]);
		squares.push_back(square);
	}
	return squares;
}
