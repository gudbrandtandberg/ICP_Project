#define LIBIGL_VIEWER_WITH_NANOGUI

#include <igl/viewer/Viewer.h>
#include <igl/readOBJ.h>
#include <string>
#include "/usr/local/include/nanoflann/nanoflann.hpp"

std::string MESH_DIRECTORY = "/Users/gudbrand/Documents/C++/ICP_Project/mesh/";


typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXi> Mesh;

typedef nanoflann::KDTreeEigenMatrixAdaptor<Eigen::MatrixXd> kd_tree_t;

int iterations = 1;

enum mesh_selection {
	moon_surface = 0,
	bunny_scan
};

void init_viewer();
void reload_mesh();
Mesh concat_meshes(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
				   Eigen::MatrixXd VB, Eigen::MatrixXi FB);
bool setup_icp_ui(igl::viewer::Viewer& viewer);

void compute_closest_points(Eigen::MatrixXd &data_vertices,
							kd_tree_t &model_tree,
							std::vector<int> &point_correspondences);
void compute_registration(Eigen::VectorXd &registration_quaternion, std::vector<int> pc);

void icp_align();

/* currently selected mesh */
mesh_selection selected_mesh = moon_surface;

/* The libigl-viewer  */
igl::viewer::Viewer viewer;

// Target datastructure
Eigen::MatrixXd model_verts;
Eigen::MatrixXi model_faces;

// Data datastructure
Eigen::MatrixXd data_verts;
Eigen::MatrixXi data_faces;


int main(int argc, char *argv[])
{
	init_viewer();
	
}

void icp_align() {
	
	
	int iter_counter = 0;
	
	std::vector<int> point_correspondences(data_verts.rows());
	Eigen::VectorXd registration_quaternion;
	registration_quaternion.resize(8);
	
	
	/* Build a kd-tree of the model mesh */
	kd_tree_t model_tree(3 /*dim*/, model_verts, 10 /* max leaf */ );
	model_tree.index->buildIndex();
	
	while (iter_counter < iterations) {
		
		compute_closest_points(data_verts, model_tree, point_correspondences);
		
		compute_registration(registration_quaternion, point_correspondences);
		
		iter_counter++;
	}
	
}

void compute_closest_points(Eigen::MatrixXd &data_vertices,
							kd_tree_t &model_tree,
							std::vector<int> &point_correspondences) {
	
	
	
	// do a knn search
	const size_t num_results = 1;
	std::vector<size_t> ret_indexes(num_results);
	std::vector<double> out_dists_sqr(num_results);
	
	nanoflann::KNNResultSet<double> resultSet(num_results);
	
	
	for (int i=0; i<data_vertices.rows(); i++) {
		std::vector<double> query_pt(data_vertices.row(i).data(),
									 data_vertices.row(i).data() + 3);
		
		resultSet.init(&ret_indexes[0], &out_dists_sqr[0]);
		model_tree.index->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(10));
		
		point_correspondences[i] = ret_indexes[0];
		
	}
	
}

void compute_registration(Eigen::VectorXd &registration_quaternion,
						  std::vector<int> pc) {
	
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
	
	
	
	Eigen::EigenSolver<Eigen::Matrix4d> eigen_solver(Q);
	Eigen::MatrixXd::Index max_ev_index;
	eigen_solver.eigenvalues().real().maxCoeff(&max_ev_index);
	Eigen::Vector4d q_optimal = eigen_solver.eigenvectors().real().col(max_ev_index);
	
	std::cout << q_optimal << std::endl;
	
}

void init_viewer() {
	// Extend viewer menu
	viewer.callback_init = setup_icp_ui;
	
	reload_mesh();
	
	viewer.data.set_face_based(true);
	viewer.core.show_lines = false;
	
	viewer.launch();
}


/*
 * Configures the ICP-section of the UI
 */

bool setup_icp_ui(igl::viewer::Viewer& viewer) {
	// Add new group
	viewer.ngui->addGroup("ICP");

	viewer.ngui->addVariable("Iterations", iterations)->setSpinnable(true);
	
	viewer.ngui->addVariable("Select model", selected_mesh, true)
	->setItems({"Moon", "Bunny"});
	
	// Add a button
	viewer.ngui->addButton("Align", icp_align);
	viewer.ngui->addButton("Reset", reload_mesh);
	
	// call to generate menu
	viewer.screen->performLayout();
	return false;
}

/*
 * Load the selected meshes from file and add them to the viewer
 */

void reload_mesh() {
	std::string model_name;
	std::string target_name;
	
	switch (selected_mesh) {
		case moon_surface:
			model_name = "Moon_Terrain_Transformed.obj";
			target_name = "Moon_Terrain1.obj";
			break;
		case bunny_scan:
			model_name = "bun000.obj";
			target_name = "bun315.obj";
		default:
			break;
	}
	
	// Load obj-files
	igl::readOBJ(MESH_DIRECTORY + target_name, model_verts, model_faces);
	igl::readOBJ(MESH_DIRECTORY + model_name, data_verts, data_faces);
	
	Mesh concat_mesh = concat_meshes(model_verts, model_faces, data_verts, data_faces);
	
	
	viewer.data.clear();
	viewer.data.set_mesh(concat_mesh.first, concat_mesh.second);
	viewer.core.align_camera_center(concat_mesh.first, concat_mesh.second);
	

	// blue color for faces of first mesh, orange for second
	Eigen::MatrixXd C(concat_mesh.second.rows(), 3);
	C << Eigen::RowVector3d(0.2,0.3,0.8).replicate(model_faces.rows(),1),
		 Eigen::RowVector3d(1.0,0.7,0.2).replicate(data_faces.rows(),1);
	
	viewer.data.set_colors(C);
	
}

Mesh concat_meshes(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
				   Eigen::MatrixXd VB, Eigen::MatrixXi FB) {
	
	
	//Found this way of concatenating meshes in the libigl github comments
	// Concatenate (Target_V,Target_F) and (Model_V,Model_F) into (V,F)
	Eigen::MatrixXd V(VA.rows() + VB.rows(), VA.cols());
	V << VA, VB;
	
	Eigen::MatrixXi F(FA.rows() + FB.rows(), FA.cols());
	F << FA, (FB.array() + VA.rows());
	
	return std::make_pair(V, F);
}

