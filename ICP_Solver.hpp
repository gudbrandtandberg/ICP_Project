//
//  ICP_Solver.hpp
//  icp_project
//
//  Created by Gudbrand Tandberg on 07/12/16.
//
//

#ifndef ICP_Solver_hpp
#define ICP_Solver_hpp

#include <iostream>
#include <map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "/usr/local/include/nanoflann/nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd, 3, nanoflann::metric_L1> kd_tree_t;

class ICP_Solver {
public:
	Eigen::MatrixXd data_verts; size_t N_data;
	Eigen::MatrixXd model_verts;
	std::map<int, int> point_correspondence;
	std::map<int, double> weights;
	
	Eigen::Vector3d translation, final_translation = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rotation, final_rotation = Eigen::Matrix3d::Identity();
	bool iteration_has_converged = false;
	
private:
	kd_tree_t *model_kd_tree;
	
	double error = MAXFLOAT;
	double old_error = 0;
	int iter_counter = 0;
	const size_t max_it = 50;
	const double tolerance = 0.000001;
	const float sampling_quotient = 1.0;
	
public:
	ICP_Solver();
	ICP_Solver(Eigen::MatrixXd data_verts, Eigen::MatrixXd model_verts);
	void build_tree();
	
	/*
	 * The main entry point for ICP alignment of the loaded meshes.
	 * Iterates until convergence or maximum number of iterations is reached.
	 */

	bool step();
	
	bool perform_icp();
	
private:
	void compute_closest_points();
	
	void compute_registration(Eigen::Vector3d &translation,
							  Eigen::Matrix3d &rotation);
	
	double compute_rms_error(Eigen::Vector3d translation,
							 Eigen::Matrix3d rotation);
	
	void quaternion_to_matrix(Eigen::Vector4d q, Eigen::Matrix3d &R);

};

#endif /* ICP_Solver_hpp */
