//
//  ICP_Solver.hpp
//  icp_project
//
//  Created by Gudbrand Tandberg on 07/12/16.
//
//

#ifndef ICP_Solver_hpp
#define ICP_Solver_hpp

#include <stdio.h>
#include <map>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include "/usr/local/include/nanoflann/nanoflann.hpp"

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;

class ICP_Solver {
	
public:
	
	Eigen::MatrixXd data_verts;
	Eigen::MatrixXd model_verts;
	std::map<int, int> point_correspondence;
	const size_t max_it = 10;
	
public:
	
	/*
	 * The main entry point for ICP alignment of the loaded meshes.
	 * Iterates until convergence or maximum number of iterations is reached.
	 */
	
	void icp_align();
	
	
private:
	void compute_closest_points(Eigen::MatrixXd &data_vertices,
								kd_tree_t &model_tree,
								std::map<int, int> &point_correspondences);
	
	void compute_registration(Eigen::Vector3d &translation,
							  Eigen::Matrix3d &rotation,
							  std::map<int, int> pc);
	
	
	double compute_rms_error(Eigen::Vector3d translation,
							 Eigen::Matrix3d rotation,
							 std::map<int, int> pc);
	
	void quaternion_to_matrix(Eigen::Vector4d q, Eigen::Matrix3d &R);
	
public:
	ICP_Solver();
	ICP_Solver(Eigen::MatrixXd data_verts, Eigen::MatrixXd model_verts);
	ICP_Solver& operator=(ICP_Solver arg);

};
	









#endif /* ICP_Solver_hpp */
