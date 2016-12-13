#define LIBIGL_VIEWER_WITH_NANOGUI

#include <igl/viewer/Viewer.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>


#include <nanogui/screen.h>
#include <nanogui/window.h>

#include "/usr/local/include/nanoflann/nanoflann.hpp"

#include <string>

#include "ICP_Solver.hpp"

std::string MESH_DIRECTORY = "/Users/gudbrand/Documents/C++/ICP_Project/mesh/";

typedef std::pair<Eigen::MatrixXd, Eigen::MatrixXi> Mesh;

enum mesh_selection {
	moon_surface = 0,
	bunny_scan,
	diamond,
	camel,
	camel2
};

void init_viewer();

void load_mesh();

bool setup_icp_ui(igl::viewer::Viewer& viewer);

void perform_icp();

void set_mesh();

Mesh concat_meshes(Eigen::MatrixXd VA, Eigen::MatrixXi FA,
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

/* The concatinated mesh for viewing */
Mesh concat_mesh;

/*
 * Initialize ui and and start the mainloop
 */

int main(int argc, char *argv[])
{
	load_mesh();
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
	
	viewer.ngui->addGroup("ICP");
	
	viewer.ngui->addVariable("Select model", selected_mesh, true)
	->setItems({"Moon", "Bunny", "Diamond", "Camel", "Camel2"});
	
	// Add buttons and callbacks
	viewer.ngui->addButton("Align", perform_icp);
	viewer.ngui->addButton("Reset", load_mesh);

	
	// call to generate menu
	viewer.screen->performLayout();
	return false;
}

/*
 * Callback for when the 'align' button is clicked
 * Uses an ICP_Solver object to align the meshes
 */

void perform_icp() {
	
	ICP_Solver solver = ICP_Solver(data_verts, model_verts);
	solver.perform_icp();
	
	// show the aligned meshes in the viewer
	concat_mesh.first.block(0, 0, data_verts.rows(), 3) = solver.data_verts;
	set_mesh();
	
	std::cout << "Final rotation\n" << solver.final_rotation << std::endl;
	std::cout << "Final translation\n" << solver.final_translation << std::endl;
	
}

/*
 * Load the selected meshes from file and add them to the viewer
 */

void load_mesh() {
	std::string model_name;
	std::string target_name;
	
	switch (selected_mesh) {
		case moon_surface:
			model_name = "Moon_Terrain2.obj";
			target_name = "Moon_Terrain_1z10x.obj";
			break;
		case bunny_scan:
			//model_name = "bun000_new2.obj";
			model_name = "bun045_init_align_to_315__.obj";
			target_name = "bun315.obj";
			break;
		case diamond:
			//model_name = "diamond_10yrot.obj";
			model_name = "diamond_transformed.obj";
			target_name = "diamond.obj";
			break;
		case camel:
			model_name = "noisy_translated_camel.obj";
			target_name = "camel.obj";
			break;
		case camel2:
			target_name = "camel.obj";
			model_name = "camel_headless.obj";
			break;
		default:
			break;
	}
	
	// load obj-files
	igl::readOBJ(MESH_DIRECTORY + target_name, model_verts, model_faces);
	igl::readOBJ(MESH_DIRECTORY + model_name, data_verts, data_faces);
	
	// update the viewed mesh
	concat_mesh = concat_meshes(data_verts, data_faces, model_verts, model_faces);
	set_mesh();
	
}

void set_mesh() {
	
	viewer.data.clear();
	viewer.data.set_mesh(concat_mesh.first, concat_mesh.second);
	viewer.core.align_camera_center(concat_mesh.first, concat_mesh.second);
	
	
	// blue color for faces of first mesh, orange for second
	Eigen::MatrixXd C(concat_mesh.second.rows(), 3);
	C << Eigen::RowVector3d(1.0,0.7,0.2).replicate(data_faces.rows(),1),
	Eigen::RowVector3d(0.2,0.3,0.8).replicate(model_faces.rows(),1);
	
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

