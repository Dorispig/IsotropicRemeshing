#include "surfacemeshprocessing.h"
//#include <QtWidgets/QApplication>
#include <OpenMesh/Core/Geometry/VectorT.hh>
//#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include "MeshViewer/MeshDefinition.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriConnectivity.hh>
#include "AABB_Tree/AABB_Tree.h"
#include"remeshing/isotropic remeshing.h"

using namespace std;

AABB_Tree* abtree;
Mesh mesh;

void split_long_edges(double target_length) {
	int NE = mesh.n_edges();
	Mesh::EdgeIter e_end = mesh.edges_end();
	//for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != e_end; ++e_it) {
	for (int i = 0; i < NE; i++) {
		Mesh::EdgeHandle e = mesh.edge_handle(i);
		if (mesh.calc_edge_length(e) > 4. / 3. * target_length) {
			Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(e, 0);
			Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(e, 1);
			Mesh::Point v = mesh.calc_centroid(h0);
			Mesh::VertexHandle vh = mesh.add_vertex(v);
			mesh.split_edge_copy(e,vh);//only split the e to two edges and copy the properties, not create the faces
			Mesh::HalfedgeHandle ph0 = mesh.prev_halfedge_handle(h0);
			Mesh::HalfedgeHandle nh1 = mesh.next_halfedge_handle(h1);
			if (!mesh.is_boundary(h0)) {
				Mesh::VertexHandle v0 = mesh.to_vertex_handle(mesh.next_halfedge_handle(h0));
				Mesh::HalfedgeHandle new_e0 = mesh.new_edge(v0, vh);

				mesh.set_next_halfedge_handle(new_e0, h0);
				mesh.set_next_halfedge_handle(mesh.next_halfedge_handle(h0), new_e0);

				mesh.set_next_halfedge_handle(ph0, mesh.opposite_halfedge_handle(new_e0));
				mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(new_e0), mesh.prev_halfedge_handle(ph0));

				Mesh::FaceHandle face0 = mesh.face_handle(h0);
				mesh.set_halfedge_handle(face0, h0);
				mesh.set_face_handle(new_e0, face0);

				Mesh::FaceHandle new_face0 = mesh.new_face();
				mesh.set_halfedge_handle(new_face0, ph0); 
				mesh.set_face_handle(ph0, new_face0);
				mesh.set_face_handle(mesh.opposite_halfedge_handle(new_e0), new_face0);
				mesh.set_face_handle(mesh.prev_halfedge_handle(ph0), new_face0);

				mesh.set_halfedge_handle(v0, new_e0);
				mesh.adjust_outgoing_halfedge(v0);
			}
			if (!mesh.is_boundary(h1)) {	
				Mesh::VertexHandle v1 = mesh.from_vertex_handle(mesh.prev_halfedge_handle(h1));
				Mesh::HalfedgeHandle new_e1 = mesh.new_edge(v1, vh);

				mesh.set_next_halfedge_handle(h1, mesh.opposite_halfedge_handle(new_e1));
				mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(new_e1),mesh.prev_halfedge_handle(h1));

				mesh.set_next_halfedge_handle(mesh.next_halfedge_handle(nh1), new_e1);
				mesh.set_next_halfedge_handle(new_e1, nh1);		

				Mesh::FaceHandle face1 = mesh.face_handle(h1);
				mesh.set_halfedge_handle(face1, h1);
				mesh.set_face_handle(mesh.opposite_halfedge_handle(new_e1), face1);

				Mesh::FaceHandle new_face1 = mesh.new_face();
				mesh.set_halfedge_handle(new_face1, nh1);
				mesh.set_face_handle(nh1, new_face1);
				mesh.set_face_handle(new_e1, new_face1);
				mesh.set_face_handle(mesh.next_halfedge_handle(nh1), new_face1);

				mesh.set_halfedge_handle(v1, new_e1);
				mesh.adjust_outgoing_halfedge(v1);

			}
		}
	}
}

void collapse_short_edges(double target_length) {
	int NE = mesh.n_edges();
	//for (Mesh::EdgeIter e_it = mesh.edges_end(); e_it != mesh.edges_sbegin(); --e_it) {
	for (int i = NE - 1; i >= 0; i--) {
		if (i > mesh.n_edges() - 1)continue;
		Mesh::EdgeHandle e = mesh.edge_handle(i);
		//Mesh::EdgeHandle e = *e_it;
		Mesh::HalfedgeHandle he = mesh.halfedge_handle(e);
		if (!mesh.is_collapse_ok(he)) continue;
		Mesh::VertexHandle vfrom = mesh.from_vertex_handle(he);
		Mesh::VertexHandle vto = mesh.to_vertex_handle(he);
		if (mesh.is_boundary(vfrom) || mesh.is_boundary(vto))continue;
		double length = mesh.calc_edge_length(e);
		double low = 4. / 5. * target_length, high = 4. / 3. * target_length;
		if (length < low) {
			bool is_collapse = true;
			Mesh::Point pfrom = mesh.point(vfrom);
			Mesh::Point pto = mesh.point(vto);
			for (Mesh::VertexVertexIter vv_it = mesh.vv_iter(vfrom); vv_it.is_valid(); ++vv_it) {
				Mesh::VertexHandle v = *vv_it;
				double L = norm(mesh.point(v) - pto);
				if (L > high) {
					is_collapse = false;
					break;
				}
			}
			if (is_collapse) {
				mesh.collapse(he);//not delete the face which Square=0
			}
		}
	}
	mesh.garbage_collection();
	
}

void equalize_valences() {
	int deviation_pre, deviation_post;
	std::vector<int> target_valence;
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (mesh.is_boundary(v_it)) target_valence.push_back(4);
		else target_valence.push_back(6);
	}
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		Mesh::EdgeHandle e = mesh.edge_handle((*e_it).idx());
		if (mesh.is_boundary(e)||!is_flip_ok_openmesh(e, mesh))continue;
		Mesh::HalfedgeHandle he0 = mesh.halfedge_handle(e, 0);
		Mesh::HalfedgeHandle he1 = mesh.halfedge_handle(e, 1);
		Mesh::VertexHandle v0 = mesh.from_vertex_handle(he0);
		Mesh::VertexHandle v1 = mesh.to_vertex_handle(he0);

		Mesh::HalfedgeHandle he2 = mesh.next_halfedge_handle(he0);
		Mesh::VertexHandle v2 = mesh.to_vertex_handle(he2);
		Mesh::HalfedgeHandle he3 = mesh.next_halfedge_handle(he1);
		Mesh::VertexHandle v3 = mesh.from_vertex_handle(he3);

		deviation_pre = abs(int(mesh.valence(v0) - target_valence[v0.idx()]))
			+ abs(int(mesh.valence(v1) - target_valence[v1.idx()]))
			+ abs(int(mesh.valence(v2) - target_valence[v2.idx()]))
			+ abs(int(mesh.valence(v3) - target_valence[v3.idx()]));

		deviation_post = abs(int(mesh.valence(v0) - 1 - target_valence[v0.idx()]))
			+ abs(int(mesh.valence(v1) - 1 - target_valence[v1.idx()]))
			+ abs(int(mesh.valence(v2) + 1 - target_valence[v2.idx()]))
			+ abs(int(mesh.valence(v3) + 1 - target_valence[v3.idx()]));
		if (deviation_pre > deviation_post) flip_openmesh(e, mesh);
	}
}

void tangential_relaxation() {
	
	mesh.update_normals();
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (mesh.is_boundary(*v_it)) continue;
		double count = 0.0;
		Mesh::Point p = mesh.point(*v_it);
		Mesh::Point q(0, 0, 0);
		for (Mesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
		{
			q += mesh.point(*vv_it);
			++count;
		}
		q /= count;
		Mesh::Normal n = mesh.normal(*v_it);
		n.normalize();
		mesh.set_point(*v_it, q + (n.dot(p - q)) * n);
	}
}

void get_AABB_tree() {
	std::vector<Vector3f> point_set;
	point_set.clear();
	for (Mesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it){
		for (Mesh::FaceVertexIter fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it)
		{
			Mesh::VertexHandle fv = *fv_it;
			Mesh::Point fp = mesh.point(fv);
			Vector3f p;
			p[0] = float(fp.data()[0]);
			p[1] = float(fp.data()[1]);
			p[2] = float(fp.data()[2]);
			point_set.push_back(p);
		}
	}
	abtree = new AABB_Tree(point_set);
}

void project_to_surface() {
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		Mesh::VertexHandle fv = *v_it;
		Mesh::Point fp = mesh.point(fv);
		Vector3f p;
		p[0] = float(fp.data()[0]);
		p[1] = float(fp.data()[1]);
		p[2] = float(fp.data()[2]);
		Vector3f ab_nearst_point;
		abtree->findNearstPoint(p, ab_nearst_point);
		Mesh::Point new_point;
		new_point[0] = double(ab_nearst_point[0]);
		new_point[1] = double(ab_nearst_point[1]);
		new_point[2] = double(ab_nearst_point[2]);
		mesh.set_point(fv, new_point);
	}
}

double calTargetLength() {
	double target_edge_length = 0.0;
	std::cout << mesh.n_edges() << std::endl;
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it){
		target_edge_length += mesh.calc_edge_length(*e_it);
	}
	target_edge_length /= mesh.n_edges();
	return target_edge_length;
}

int main(int argc, char *argv[])
{
	//QApplication app(argc, argv);

	//SurfaceMeshProcessing mainWin;
	///*mainWin.setGeometry(100,100,mainWin.sizeHint().width(),mainWin.sizeHint().height());
	//mainWin.resize( mainWin.sizeHint() );*/
	//mainWin.showMaximized();

	//if( argc > 1 )
	//{
	//	mainWin.open_mesh_from_main(argv[1]);
	//}

	//return app.exec();
	//hardcase  horse
	std::string input_file = "../data/hardcase.obj";
	std::string output_file = "../data/hardcase_remeshing.obj";
	/*std::string input_file = "../data/hardcase_up.obj";
	std::string output_file = "../data/hardcase_up_remeshing.obj";*/
	//std::string input_file = "../data/horse.obj";
	//std::string output_file = "../data/horse_remeshing.obj";
	//std::string input_file = "../data/cubic.obj";
	//std::string output_file = "../data/cubic_remeshing.obj";

	bool read_OK = OpenMesh::IO::read_mesh(mesh, input_file);
	if (!read_OK){
		std::cerr << "read error\n";
		exit(1);
	}
	else std::cerr << "read success\n";

	IsotropicRemeshing remesh(mesh);

	//set para
	remesh.para.iter = 10;

	
	//remesh.para.clean_flag = false;
	remesh.para.target_length_ratio = 0.5;
	//remesh.para.split_flag = false;
	//remesh.para.collapse_flag = false;
	//remesh.para.flip_flag = false;
	//remesh.para.smooth_flag = false;
	//remesh.para.project_flag = false;
	
	remesh.para.adapt_flag = true;
	remesh.para.angle_Threshold = 30;
	remesh.para.tolerance = 0.05;
	

	//isotropic remeshing
	remesh.isotropic_remeshing();

	mesh = remesh.get_mesh();
	
	bool write_OK = OpenMesh::IO::write_mesh(mesh, output_file);
	if (!write_OK) {
		std::cerr << "write error\n";
		exit(1);
	}
	else std::cerr << "write success\n";
	
	return 1;
}





















