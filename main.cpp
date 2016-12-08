#define LIBIGL_VIEWER_WITH_NANOGUI

#include <igl/viewer/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>

#include <nanogui/progressbar.h>
#include "/usr/local/include/nanoflann/nanoflann.hpp"

#include <string>

#include "ICP_Solver.hpp"

std::string MESH_DIRECTORY = "/Users/gudbrand/Documents/C++/ICP_Project/mesh/";

typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXi> Mesh;

enum mesh_selection {
	moon_surface = 0,
	bunny_scan,
	diamond
};

void init_viewer();

void load_mesh();

bool setup_icp_ui(igl::viewer::Viewer& viewer);

void perform_icp();

Mesh concat_meshes(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
				   Eigen::MatrixXd VB, Eigen::MatrixXi FB);

void set_mesh(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
			  Eigen::MatrixXd VB, Eigen::MatrixXi FB);


/* currently selected mesh */
mesh_selection selected_mesh = diamond;

/* The libigl-viewer  */
igl::viewer::Viewer viewer;

/* "Target" datastructure */
Eigen::MatrixXd model_verts;
Eigen::MatrixXi model_faces;

/* "Data" datastructure */
Eigen::MatrixXd data_verts;
Eigen::MatrixXi data_faces;

/* The ICP solver */
ICP_Solver solver;

/*
 * Initialize ui and solver and start the mainloop
 */

int main(int argc, char *argv[])
{
	load_mesh();
	solver = ICP_Solver(data_verts, model_verts);
	init_viewer();
}

void init_viewer() {
	viewer.data.set_face_based(true);
	viewer.core.show_lines = false;
	viewer.callback_init = setup_icp_ui;
	viewer.launch();
}


/*
 * Configures the ICP-section of the UI and hooks up callbacks
 */

bool setup_icp_ui(igl::viewer::Viewer& viewer) {
	// Add new group
	viewer.ngui->addGroup("ICP");
	
	viewer.ngui->addVariable("Select model", selected_mesh, true)
	->setItems({"Moon", "Bunny", "Diamond"});
	
	// Add buttons and callbacks
	viewer.ngui->addButton("Align", perform_icp);
	viewer.ngui->addButton("Reset", load_mesh);

	
	nanogui::Window *window = new nanogui::Window(viewer.screen, "Progress");
	window->setPosition(Eigen::Vector2i(125, 100));
	
	nanogui::ProgressBar *mProgress;
	mProgress = new nanogui::ProgressBar(window);
	mProgress->setValue(0.5);
	
	// call to generate menu
	viewer.screen->performLayout();
	return false;
}

void perform_icp() {
	
	// align the meshes
	solver.icp_align();
	
	// show the aligned meshes in the viewer
	set_mesh(model_verts, model_faces, solver.data_verts, data_faces);
	
}

/*
 * Load the selected meshes from file and add them to the viewer
 */

void load_mesh() {
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
			break;
		case diamond:
			//model_name = "diamond_10yrot.obj";
			model_name = "diamond_transformed.obj";
			target_name = "diamond.obj";
			break;
		default:
			break;
	}
	
	// load obj-files
	igl::readOBJ(MESH_DIRECTORY + target_name, model_verts, model_faces);
	igl::readOBJ(MESH_DIRECTORY + model_name, data_verts, data_faces);
	
	// reset the solver-meshes
	solver.data_verts = data_verts;
	solver.model_verts = model_verts;
	
	// update the viewed mesh
	set_mesh(model_verts, model_faces, data_verts, data_faces);
}

void set_mesh(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
			  Eigen::MatrixXd VB, Eigen::MatrixXi FB) {
	
	Mesh concat_mesh = concat_meshes(VA, FA, VB, FB);
	
	
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

