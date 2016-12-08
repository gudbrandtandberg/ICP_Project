//
//  ICP_Solver.cpp
//  icp_project
//
//  Created by Gudbrand Tandberg on 07/12/16.
//
//

#include "ICP_Solver.hpp"
#include <iostream>

ICP_Solver::ICP_Solver() { }

ICP_Solver::ICP_Solver(Eigen::MatrixXd d, Eigen::MatrixXd m) :
data_verts(d), model_verts(m)
{ }

ICP_Solver& ICP_Solver::operator=(ICP_Solver arg)
{
	std::swap(data_verts, arg.data_verts);
	std::swap(model_verts, arg.model_verts);
	
	return *this;
}

void ICP_Solver::icp_align() {
	
	int iter_counter = 0;
	
	std::map<int, int> point_correspondence;
	Eigen::Vector3d translation, final_translation = Eigen::Vector3d::Zero();
	Eigen::Matrix3d rotation, final_rotation = Eigen::Matrix3d::Identity();
	
	/* Build a kd-tree of the model mesh */
	kd_tree_t model_tree(3 /*dim*/, model_verts, 10 /* max leaf */ );
	model_tree.index->buildIndex();
	double error = MAXFLOAT;
	size_t N_data = data_verts.rows();
	double tolerance = 0.01;
	
	while (iter_counter < max_it && error >= tolerance) {
		
		compute_closest_points(data_verts, model_tree, point_correspondence);
		
		//for (int i=0; i<N_data; i++)
		//	point_correspondence[i] = i;
		
		translation = Eigen::Vector3d::Zero();
		rotation = Eigen::Matrix3d::Zero();
		
		compute_registration(translation, rotation, point_correspondence);
		
		/* Transform the data mesh */
		
		final_rotation = rotation*final_rotation;
		final_translation += translation;
		
		//Eigen::MatrixXd data_COM = data_verts.colwise().sum().replicate(N_data, 1);
		//data_COM /= data_verts.rows();
		
		data_verts = data_verts * rotation.transpose();
		data_verts = data_verts + translation.transpose().replicate(N_data, 1);
		
		/* Save the error */
		error = compute_rms_error(translation, rotation, point_correspondence);
		
		std::cout << "Iteration: " << iter_counter << ", Error: " << error << std::endl;
		
		iter_counter++;
	}
	
	if (iter_counter == max_it) {
		std::cout << "Iteration did not converge.." << std::endl;
	} else {
		std::cout << "Iteration converged!" << std::endl;
	}
	
	std::cout << "Final rotation\n" << final_rotation << std::endl;
	std::cout << "Final translation\n" << final_translation << std::endl;
	
	/*Eigen::EigenSolver<Eigen::Matrix3d> eigen_solver(final_rotation);
	 std::cout << eigen_solver.eigenvalues() << std::endl;
	 std::cout << eigen_solver.eigenvectors() << std::endl;*/
	
	//igl::writeOBJ(MESH_DIRECTORY + "mesh_aligned.obj", data_verts, data_faces);
	
}

void ICP_Solver::compute_closest_points(Eigen::MatrixXd &data_vertices,
							kd_tree_t &model_tree,
							std::map<int, int> &point_correspondences) {
	
	/* do a 1-nn search */
	const size_t num_results = 1;
	std::vector<size_t> ret_indices(num_results);
	std::vector<double> out_dists_sqr(num_results);
	
	nanoflann::KNNResultSet<double> resultSet(num_results);
	
	/* downsample */
	
	
	for (int i=0; i<data_vertices.rows(); i++) {
		//std::vector<double> query_pt(data_vertices.row(i).data(),
		//							 data_vertices.row(i).data() + 3);
		
		std::vector<double> query_pt = {data_vertices(i, 0),
			data_vertices(i, 1), data_vertices(i, 2)};
			
		
		std::cout << "Query pt: \n" << "(" << query_pt[0] << ", "
		<< query_pt[1] << ", " << query_pt[2] << ")" << std::endl;
		
		resultSet.init(&ret_indices[0], &out_dists_sqr[0]);
		model_tree.index->findNeighbors(resultSet, &query_pt[0],
										nanoflann::SearchParams(10));
		
		point_correspondences[i] = ret_indices[0];
		
		std::cout << "Closest pt: \n" << model_verts.row(point_correspondences[i]) << std::endl;
	}
	
}

void ICP_Solver::compute_registration(Eigen::Vector3d &translation,
						  Eigen::Matrix3d &rotation,
						  std::map<int, int> pc) {
	
	size_t N_data = data_verts.rows();
	size_t N_model = model_verts.rows();
	
	/* Centres-of-mass */
	Eigen::Vector3d data_COM = Eigen::Vector3d::Zero();
	Eigen::Vector3d model_COM = Eigen::Vector3d::Zero();
	
	for (int i=0; i<N_data; i++) {
		data_COM += data_verts.row(i);
	} data_COM /= N_data;
	
	for (int i=0; i<N_model; i++) {
		model_COM += model_verts.row(i);
	} model_COM /= N_model;
	
	/* Construct covariance matrix */
	Eigen::Matrix3d covariance_matrix = Eigen::Matrix3d::Zero();
	for (int i=0; i<N_data; i++) {
		covariance_matrix += (data_verts.row(i).transpose() * model_verts.row(pc[i]));
	} covariance_matrix /= N_data;
	covariance_matrix -= (data_COM * model_COM.transpose());
	
	//std::cout << covariance_matrix << std::endl;
	
	/* Construct Q-matrix */
	Eigen::Matrix3d A = covariance_matrix - covariance_matrix.transpose();
	Eigen::Vector3d delta;
	delta << A(1, 2), A(2, 0), A(0, 1);
	
	Eigen::Matrix4d Q;
	double Q_trace = covariance_matrix.trace();
	Q(0, 0) = Q_trace;
	Q.block(1, 0, 3, 1) = delta;
	Q.block(0, 1, 1, 3) = delta.transpose();
	Q.block(1, 1, 3, 3) = covariance_matrix
	+ covariance_matrix.transpose()
	- Q_trace * Eigen::MatrixXd::Identity(3,3);
	
	//std::cout << "delta: \n" << delta << std::endl;
	//std::cout << "Q: \n" << Q << std::endl;
	
	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(Q);
	Eigen::MatrixXd::Index max_ev_index;
	eigen_solver.eigenvalues().real().cwiseAbs().maxCoeff(&max_ev_index);
	Eigen::Vector4d q_optimal = eigen_solver.eigenvectors().real().col(max_ev_index);
	
	quaternion_to_matrix(q_optimal, rotation);
	
	translation = model_COM - rotation * data_COM;
	
}

double ICP_Solver::compute_rms_error(Eigen::Vector3d translation,
						 Eigen::Matrix3d rotation,
						 std::map<int, int> pc) {
	size_t N_data = data_verts.rows();
	Eigen::Vector3d diff;
	double sum = 0;
	for (int i=0; i<N_data; i++) {
		diff = model_verts.row(pc[i]).transpose() - rotation*data_verts.row(i).transpose() - translation;
		sum += diff.norm();
	} sum /= N_data;
	
	return sum;
}

void ICP_Solver::quaternion_to_matrix(Eigen::Vector4d q, Eigen::Matrix3d &R) {
	
	R(0, 0) = q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3];
	R(1, 0) = 2*(q[1]*q[2] + q[0]*q[3]);
	R(2, 0) = 2*(q[1]*q[3] - q[0]*q[2]);
	
	R(0, 1) = 2*(q[1]*q[2] - q[0]*q[3]);
	R(1, 1) = q[0]*q[0] - q[1]*q[1] + q[2]*q[2] - q[3]*q[3];
	R(2, 1) = 2*(q[2]*q[3] + q[0]*q[1]);
	
	R(0, 2) = 2*(q[1]*q[3] + q[0]*q[2]);
	R(1, 2) = 2*(q[2]*q[3] - q[0]*q[1]);
	R(2, 2) = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
	
	//R.transposeInPlace();
	
}
