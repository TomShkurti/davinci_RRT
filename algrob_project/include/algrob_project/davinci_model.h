#include <Eigen/Eigen>
#include <Eigen/Geometry>

#ifndef ALGROB_MODEL
#define ALGROB_MODEL

namespace coarse_model{
	//Universal offsets
	const Eigen::Affine3d base__to__outer_yaw_joint__1 =
	
		//World offset 1
		(Eigen::Translation3d(-0.20427, 0, 0.50105) *
			Eigen::AngleAxis<double>(-0.014010,	Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxis<double>(0.122206,	Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxis<double>(3.087896,	Eigen::Vector3d::UnitZ())
		)
		
		*
		
		//psm base link
		Eigen::Translation3d(0.039, -0.40788, -0.07879)
	;
	const Eigen::Affine3d base__to__outer_yaw_joint__2 =
		//World offset 2
		(Eigen::Translation3d(0.20427, 0, 0.49895) *
			Eigen::AngleAxis<double>(-0.003491,	Eigen::Vector3d::UnitX()) *
			Eigen::AngleAxis<double>(-0.128246,	Eigen::Vector3d::UnitY()) *
			Eigen::AngleAxis<double>(3.020854,	Eigen::Vector3d::UnitZ())
		)
		
		*
		
		//psm base link
		Eigen::Translation3d(0.039, -0.40788, -0.07879)
	;
		

	//Red Chain Things
	const Eigen::Affine3d base_pivot_point__to__red_box_center =
		Eigen::Translation3d(0.0365, 0.05, 0.075) *
		Eigen::AngleAxis<double>(0,	Eigen::Vector3d::UnitX())
	;
	const std::vector<Eigen::Vector3d> red_box = {
		Eigen::Vector3d(-0.075,	0.1,	-0.15	),
		Eigen::Vector3d(0.075,	0.1,	-0.15	),
		Eigen::Vector3d(-0.075,	0.1,	0.15	),
		Eigen::Vector3d(0.075,	0.1,	0.15	),
		Eigen::Vector3d(-0.075,	-0.1,	-0.15	),
		Eigen::Vector3d(0.075,	-0.1,	-0.15	),
		Eigen::Vector3d(-0.075,	-0.1,	0.15	),
		Eigen::Vector3d(0.075,	-0.1,	0.15	),
	};
	
	//Orange Chain Things
	const Eigen::Affine3d red_box_center__to__orange_box_center =
		Eigen::Translation3d(0, 0.4, 0.1) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
	;
	const std::vector<Eigen::Vector3d> orange_box = {
		Eigen::Vector3d(-0.05,	0.3,	-0.025	),
		Eigen::Vector3d(0.05,	0.3,	-0.025	),
		Eigen::Vector3d(-0.05,	0.3,	0.025	),
		Eigen::Vector3d(0.05,	0.3,	0.025	),
		Eigen::Vector3d(-0.05,	-0.3,	-0.025	),
		Eigen::Vector3d(0.05,	-0.3,	-0.025	),
		Eigen::Vector3d(-0.05,	-0.3,	0.025	),
		Eigen::Vector3d(0.05,	-0.3,	0.025	),
	};
	
	//Yellow Chain Things
	const Eigen::Affine3d orange_center__to__yellow_pivot_point = 
		Eigen::Translation3d(0, 0.35, -0.1) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
	;
	const Eigen::Affine3d yellow_pivot_point__to__yellow_center = 
		Eigen::Translation3d(0, 0, 0.3) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
	;
	const std::vector<Eigen::Vector3d> yellow_box = {
		Eigen::Vector3d(-0.05,	0.025,	-0.25	),
		Eigen::Vector3d(0.05,	0.025,	-0.25	),
		Eigen::Vector3d(-0.05,	0.025,	0.25	),
		Eigen::Vector3d(0.05,	0.025,	0.25	),
		Eigen::Vector3d(-0.05,	-0.025,	-0.25	),
		Eigen::Vector3d(0.05,	-0.025,	-0.25	),
		Eigen::Vector3d(-0.05,	-0.025,	0.25	),
		Eigen::Vector3d(0.05,	-0.025,	0.25	),
	};

	//Green Chain Things
	const Eigen::Affine3d yellow_box_center__to__green_box_center =
		Eigen::Translation3d(0, 0.04, -0.1) *
		Eigen::AngleAxis<double>(0, Eigen::Vector3d::UnitX())
	;
	const std::vector<Eigen::Vector3d> green_box = {
		Eigen::Vector3d(-0.00375,	0.00375,	-0.25	),
		Eigen::Vector3d(0.00375,	0.00375,	-0.25	),
		Eigen::Vector3d(-0.00375,	0.00375,	0.25	),
		Eigen::Vector3d(0.00375,	0.00375,	0.25	),
		Eigen::Vector3d(-0.00375,	-0.00375,	-0.25	),
		Eigen::Vector3d(0.00375,	-0.00375,	-0.25	),
		Eigen::Vector3d(-0.00375,	-0.00375,	0.25	),
		Eigen::Vector3d(0.00375,	-0.00375,	0.25	),
	};

	std::vector<std::vector<Eigen::Vector3d> > getAllFaces(const std::vector<double> & j_angles, bool psm_1);
}

#endif
