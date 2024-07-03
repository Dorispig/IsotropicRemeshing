#include "isotropic remeshing.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
//mesh.ccw_rotated_halfedge_handle : ccw 逆时针,cw 顺时针

void IsotropicRemeshing::isotropic_remeshing() {
	update_para();
	mesh.add_property(v_feature_edge_num,"v:feature_edge_num");        
	mesh.add_property(v_target_length, "v:min_target_length");
	mesh.add_property(e_min_target_length, "e:min_target_length");
	mesh.add_property(e_max_target_length, "e:max_target_length");
	mesh.add_property(is_crease, "e:is_crease");
	mesh.add_property(v_is_boundary, "v:is_crease");
	mesh.add_property(e_is_boundary, "e:is_crease");
	mesh.add_property(f_is_boundary, "f:is_crease");
	clean_mesh(0.01);
	OpenMesh::IO::write_mesh(mesh, "../data/hardcase_remeshing_cleaned.obj");

	update_target_length();
	generate_AABB_tree();
	std::cout << "target_length=" << para.target_length << "\n";
	std::cout << "Activate Adaptive isotropic_remeshing\n";
	update_feature();
	bug_test();
	std::cout << "num of vertices:" << mesh.n_vertices() << "\n";
	std::cout << "num of edges:" << mesh.n_edges() << "\n";
	std::cout << "num of faces:" << mesh.n_faces() << "\n";
	std::cout << "iter:" << para.iter << "\n";
	string fiter= "../data/remeshing_test_iter";
	//std::cout << "pre split start\n";
	//for (int i = 0; i < para.pre_split_iter - 1; i++) {
	//	if (para.split_flag) {
	//		split_long_edges();
	//		std::cout << "split_long_edge --success\n";
	//		//bug_test();
	//		//tangential_relaxation();
	//		//std::cout << "tangential_relaxation --success\n";
	//		//bug_test();
	//	}
	//}
	//std::cout << "pre split finish\n";
	int it = 0;
	//repair_BadFaces(0.01);
	for (int i = 0; i < para.iter; i++)
	{
		if (i > max(4, para.iter - 2))clean_mesh(0.001);
		std::cout << "\n*** iter " << i << " ***\n";
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test0.obj");
		}
		if (para.split_flag) {
			split_long_edges(); 
			std::cout << "split_long_edge --success\n";
			bug_test();
		}
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test1.obj");
		}
		if (para.collapse_flag) {
			//update_target_length();
			collapse_short_edges();
			std::cout << "collapse_short_edges --success\n";
			collapse_cross();
			std::cout << "collapse_cross --success\n";

			//repair_BadFaces(0.01);
			//std::cout << "repair_BadFaces --success\n";
			bug_test();
		}
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test2.obj");
		}
		if (para.flip_flag) {
			equalize_valences(); 
			std::cout << "equalize_valences --success\n";
			bug_test();
		}
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test3.obj");
		}
		if (para.smooth_flag) {
			tangential_relaxation(); 
			std::cout << "tangential_relaxation --success\n";
			bug_test();
		}
		/*if (i == 0) {
			OpenMesh::IO::write_mesh(mesh, "../data/hardcase_remeshing_test4.obj");
		}*/
		//repair_BadFaces(0.01);
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test4.obj");
		}
		if (para.project_flag) {
			project_to_surface(); 
			std::cout << "project_to_surface --success\n";
			bug_test();
			/*removeColinearFaces();
			std::cout << "removeColinearFaces --success\n";
			mesh.garbage_collection();
			bug_test();*/
		}
		if (i == it) {
			OpenMesh::IO::write_mesh(mesh, "../data/remeshing_test5.obj");
		}
		/*std::cout << "num of vertices:" << mesh.n_vertices() << "\n";
		std::cout << "num of edges:" << mesh.n_edges() << "\n";
		std::cout << "num of faces:" << mesh.n_faces() << "\n";*/
		/*repair_BadFaces(0.01);*/
		//clean_mesh();
		//bug_test();
		string f;
		f = f.append(fiter);
		f = f.append(to_string(i));
		f = f.append(".obj");
		OpenMesh::IO::write_mesh(mesh, f);
	}
	//clean_mesh();
	bug_test();
}

void IsotropicRemeshing::bug_test() {
	std::cout << "-----bug test\n";
	std::cout << "num of vertices:" << mesh.n_vertices() << "\n";
	std::cout << "num of edges:" << mesh.n_edges() << "\n";
	std::cout << "num of faces:" << mesh.n_faces() << "\n";
	int num_area0 = 0;
	for (Mesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++) {
		Mesh::FaceHandle fh = *f_it;
		if (mesh.status(fh).deleted())continue;
		Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
		Mesh::VertexHandle vh0 = mesh.from_vertex_handle(heh);
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh);
		Mesh::VertexHandle vh2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
		Mesh::Point p0 = mesh.point(vh0);
		Mesh::Point p1 = mesh.point(vh1);
		Mesh::Point p2 = mesh.point(vh2);
		Mesh::Point p01 = p1-p0;
		Mesh::Point p02 = p2-p0;
		
		if (norm(p01.cross(p02)) == 0 ){
			num_area0++;//&& norm(p01) != 0 && norm(p02) != 0
			if (norm(p01) == 0 || norm(p02) == 0||norm(p1-p2)==0)std::cout << "bug重合点\n";
			else std::cout << "bug非重合点\n";
			/*std::cout << "vh0 feature:" << (mesh.status(vh0).feature() || mesh.status(vh0).locked()) << "， ";
			std::cout << "vh1 feature:" << (mesh.status(vh1).feature() || mesh.status(vh1).locked()) << "， ";
			std::cout << "vh2 feature:" << (mesh.status(vh2).feature() || mesh.status(vh2).locked()) << "\n";*/
		}
	}
	std::cout << "nums of 0 Area = " << num_area0 << "\n";
	int num_non_fe_v = 0, num_feature_v = 0, num_corner = 0, num_non_cor_v = 0 ,num_2edge = 0;
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); v_it++) {
		Mesh::VertexHandle vh = *v_it;
		if (mesh.status(vh).deleted())continue;
		if(mesh.status(vh).feature()){
			num_feature_v++;
			int k = 0;
			for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_begin(vh); voh_it.is_valid(); voh_it++) {
				Mesh::EdgeHandle eh = mesh.edge_handle(*voh_it);
				if (mesh.status(eh).feature())k++;
			}
			if (k != 2) {
				num_non_fe_v++;
			}
			num_2edge += 2;
		}
		if (mesh.status(vh).locked()) {
			num_corner++;
			int k = 0;
			for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_begin(vh); voh_it.is_valid(); voh_it++) {
				Mesh::EdgeHandle eh = mesh.edge_handle(*voh_it);
				if (mesh.status(eh).feature())k++;
			}
			if (k != 1 && k < 3) num_non_cor_v++;
			num_2edge += mesh.property(v_feature_edge_num, vh);
			//num_2edge += k;
		}
	}
	std::cout << "nums of illegal feature_vertice = " << num_non_fe_v << "\n";
	std::cout << "nums of illegal corner_vertice = " << num_non_cor_v << "\n";
	int num_feature_edge = 0;
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); e_it++) {
		Mesh::EdgeHandle eh = *e_it;
		if (mesh.status(eh).deleted()) continue;
		if (mesh.status(eh).feature()) {
			num_feature_edge++;
		}
	}
	std::cout << "nums of feature_edge = " << num_feature_edge << "\n";
	std::cout << "nums of feature vertice = " << num_feature_v << "\n";
	std::cout << "nums of corner vertice = " << num_corner << "\n";
	std::cout << "nums of 2*edge(nums of halfedge) = " << num_2edge << "\n";
}

int IsotropicRemeshing::cal_vertice_feature_edges(Mesh::VertexHandle vh) {
	int k = 0;
	for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_begin(vh); voh_it.is_valid(); voh_it++) {
		Mesh::EdgeHandle eh = mesh.edge_handle(*voh_it);
		if (mesh.status(eh).feature())k++;
	}
	return k;
}

bool IsotropicRemeshing::check_legal_feature_corner(Mesh::VertexHandle vh) {
	if (mesh.status(vh).feature()) {
		if (cal_vertice_feature_edges(vh) != 2)return false;
	}
	if (mesh.status(vh).locked()) {
		int num = cal_vertice_feature_edges(vh);
		if (num < 3 && num != 1)return false;
	}
	return true;
}

bool IsotropicRemeshing::check_legal_feature_corner(Mesh::VertexHandle vh, int num_feature_edges) {
	if (mesh.status(vh).feature()) {
		if (num_feature_edges != 2)return false;
	}
	if (mesh.status(vh).locked()) {
		if (num_feature_edges < 3 && num_feature_edges != 1)return false;
		//if (num_feature_edges != cal_vertice_feature_edges(vh))return false;//不用判断这个？
	}
	return true;
}

void IsotropicRemeshing::generate_AABB_tree() {
	std::vector<Vector3f> point_set;
	point_set.clear();
	for (Mesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
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
	para.aabb_flag = true;
}

double IsotropicRemeshing::calTargetLength() {
	double target_edge_length = 0.0;
	std::cout << "num of edges:" << mesh.n_edges() << std::endl;
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		double l = mesh.calc_edge_length(*e_it);
		para.min_length = min(para.min_length, l);
		para.max_length = max(para.max_length, l);
		para.max_edge_length = max(para.max_edge_length, l);
		target_edge_length += l;
	}
	target_edge_length /= mesh.n_edges();
	return target_edge_length;
}

bool IsotropicRemeshing::if_split_ok(Mesh::EdgeHandle eh, Mesh::VertexHandle vh) {
	Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(eh, 0);
	Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(eh, 1);
	Mesh::Point p = mesh.point(vh);
	Mesh::Point p_h1from = mesh.point(mesh.from_vertex_handle(h1));
	Mesh::Point p_h1to = mesh.point(mesh.to_vertex_handle(h1));
	Mesh::VertexHandle v0 = mesh.to_vertex_handle(mesh.next_halfedge_handle(h0));
	Mesh::VertexHandle v1 = mesh.from_vertex_handle(mesh.prev_halfedge_handle(h1));
	Mesh::Point p0 = mesh.point(v0);
	double area0 = cal_3p_face_2Area(p0, p_h1from, p_h1to);
	Mesh::Point p1 = mesh.point(v1);
	double area1 = cal_3p_face_2Area(p1, p_h1from, p_h1to);
	if (area0 < para.min_area || area1 < para.min_area) { return false; }
	if (distPointLine(p0, p_h1from, p_h1to) == 0 || distPointLine(p1, p_h1from, p_h1to) == 0) { return false; }
	if (norm(p - p0) == 0 || norm(p - p1) == 0) {  return false; }
	return true;
}

void IsotropicRemeshing::split_edge(Mesh::EdgeHandle eh, Mesh::VertexHandle vh) {
	Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(eh, 0);
	Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(eh, 1);
	Mesh::Point p = mesh.point(vh);
	Mesh::Point p_h1from = mesh.point(mesh.from_vertex_handle(h1));
	Mesh::Point p_h1to = mesh.point(mesh.to_vertex_handle(h1));
	Mesh::VertexHandle v0 = mesh.to_vertex_handle(mesh.next_halfedge_handle(h0));
	Mesh::VertexHandle v1 = mesh.from_vertex_handle(mesh.prev_halfedge_handle(h1));

	mesh.split_edge_copy(eh, vh);//only split the e to two edges and copy the properties(don't define), not create the faces
	if (mesh.status(eh).feature()) {
		mesh.property(v_feature_edge_num, vh) = 2;
		mesh.status(vh).set_feature(mesh.status(eh).feature());
	}
	Mesh::HalfedgeHandle ph0 = mesh.prev_halfedge_handle(h0);
	Mesh::HalfedgeHandle nh1 = mesh.next_halfedge_handle(h1);
	Mesh::EdgeHandle eh_new = mesh.edge_handle(ph0);
	mesh.status(eh_new).set_feature(mesh.status(eh).feature());
	//mesh.property(v_target_length, vh) = min(mesh.property(v_target_length, v0), mesh.property(v_target_length, v1));
	//mesh.property(v_target_length, vh) = (mesh.property(v_target_length, v0) + mesh.property(v_target_length, v1))/2;
	//update_vertice_target_length(vh);
	//update_edge_target_length(eh_new);
	if (!mesh.is_boundary(h0)) {
		
		Mesh::HalfedgeHandle new_eh0 = mesh.new_edge(v0, vh);

		mesh.set_next_halfedge_handle(new_eh0, h0);
		mesh.set_next_halfedge_handle(mesh.next_halfedge_handle(h0), new_eh0);

		mesh.set_next_halfedge_handle(ph0, mesh.opposite_halfedge_handle(new_eh0));
		mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(new_eh0), mesh.prev_halfedge_handle(ph0));

		Mesh::FaceHandle face0 = mesh.face_handle(h0);
		mesh.set_halfedge_handle(face0, h0);
		mesh.set_face_handle(new_eh0, face0);

		Mesh::FaceHandle new_face0 = mesh.new_face();
		mesh.set_halfedge_handle(new_face0, ph0);
		mesh.set_face_handle(ph0, new_face0);
		mesh.set_face_handle(mesh.opposite_halfedge_handle(new_eh0), new_face0);
		mesh.set_face_handle(mesh.prev_halfedge_handle(ph0), new_face0);
		/*if (cal_3p_face_2Area(new_face0) == 0) {
			std::cout << "newface0 = 0\n";
			Mesh::Point p = mesh.point(vh);
			std::cout << setprecision(10) << "p:" << p[0] << "," << p[1] << " " << p[2] << "\n";
			std::cout << setprecision(10) << "p0:" << p0[0] << "," << p0[1] << " " << p0[2] << "\n";
			std::cout << setprecision(10) << norm(p - p0) << "\n";
			std::cout << setprecision(10) << "原三角形两倍面积：" << cal_3p_face_2Area(p, p_h1from, p_h1to) << "\n";
			std::cout << setprecision(10) << "pleft:" << p_h1from[0] << "," << p_h1from[1] << " " << p_h1from[2] << "\n";
			std::cout << setprecision(10) << "pright:" << p_h1to[0] << "," << p_h1to[1] << " " << p_h1to[2] << "\n";
			std::cout << setprecision(10) << norm(p_h1from - p_h1to) << "\n";
		}*/
		//mesh.set_halfedge_handle(v0, new_eh0);
		//update_vertice_target_length(v0);
		//update_edge_target_length(mesh.edge_handle(new_eh0));
		mesh.adjust_outgoing_halfedge(v0);
	}
	if (!mesh.is_boundary(h1)) {
		Mesh::HalfedgeHandle new_eh1 = mesh.new_edge(v1, vh);

		mesh.set_next_halfedge_handle(h1, mesh.opposite_halfedge_handle(new_eh1));
		mesh.set_next_halfedge_handle(mesh.opposite_halfedge_handle(new_eh1), mesh.prev_halfedge_handle(h1));

		mesh.set_next_halfedge_handle(mesh.next_halfedge_handle(nh1), new_eh1);
		mesh.set_next_halfedge_handle(new_eh1, nh1);

		Mesh::FaceHandle face1 = mesh.face_handle(h1);
		mesh.set_halfedge_handle(face1, h1);
		mesh.set_face_handle(mesh.opposite_halfedge_handle(new_eh1), face1);

		Mesh::FaceHandle new_face1 = mesh.new_face();
		mesh.set_halfedge_handle(new_face1, nh1);
		mesh.set_face_handle(nh1, new_face1);
		mesh.set_face_handle(new_eh1, new_face1);
		mesh.set_face_handle(mesh.next_halfedge_handle(nh1), new_face1);

		/*if (cal_3p_face_2Area(new_face1) == 0) {
			std::cout << "newface1 = 0\n";
			std::cout << setprecision(10) << "p:" << p[0] << "," << p[1] << " " << p[2] << "\n";
			std::cout << setprecision(10) << "p0:" << p1[0] << "," << p1[1] << " " << p1[2] << "\n";
			std::cout << setprecision(10) << norm(p - p1) << "\n";
			std::cout << setprecision(10) << "原三角形两倍面积：" <<cal_3p_face_2Area(p,p_h1from,p_h1to)<< "\n";
			std::cout << setprecision(10) << "pleft:" << p_h1from[0] << "," << p_h1from[1] << " " << p_h1from[2] << "\n";
			std::cout << setprecision(10) << "pright:" << p_h1to[0] << "," << p_h1to[1] << " " << p_h1to[2] << "\n";
			std::cout << setprecision(10) << norm(p_h1from - p_h1to) << "\n";
		}*/
		//mesh.set_halfedge_handle(v1, new_e1);
		//update_vertice_target_length(v1);
		//update_edge_target_length(mesh.edge_handle(new_eh1));
		mesh.adjust_outgoing_halfedge(v1);

	}
	//update_vertice_target_length(vh);
	//update_edge_target_length(eh_new);
	//update_edge_target_length(eh);
	mesh.adjust_outgoing_halfedge(vh);
}

//TODO:对接近线性的三角形split！！（对边判断）
void IsotropicRemeshing::split_long_edges() {
	update_target_length();
	int NE = mesh.n_edges();
	for (int i = 0; i < NE; i++) {
		double target_L = para.target_length;
		double max_target_L = para.max_target_length;
		double min_target_L = para.min_target_length;
		//Mesh::EdgeHandle e = *e_it;
		Mesh::EdgeHandle e = mesh.edge_handle(i);
		Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(e, 0);
		Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(e, 1);
		//if (para.adapt_flag) {
		//	double adp_L = calAdaptTargetlength(e);
		//	//if (adp_L > 0.5 * target_L && adp_L < target_L) target_L = min(adp_L, target_L);
		//	//max_target_L = min(max_target_L, max(min_target_L,adp_L * 4 / 3));
		//	if (adp_L < target_L) {
		//		max_target_L = max(min_target_L, adp_L * 4 / 3);//adp_L > min_target_L && 
		//	}
		//}
		max_target_L = mesh.property(e_max_target_length, e);
		if (mesh.calc_edge_length(e) > max_target_L || !check_angle_max(e)) {
			Mesh::Point v = mesh.calc_centroid(h0);
			Mesh::VertexHandle vh = mesh.add_vertex(v);
			if (if_split_ok(e, vh)) {
				split_edge(e, vh);
			}
			else { mesh.status(vh).set_deleted(true); }
		}
	}
}

void IsotropicRemeshing::collapse_short_edges() {
	//update_target_length();
	int NE = mesh.n_edges();
	double min_target_L = para.min_target_length;
	double target_L = para.target_length;
	double max_target_L = para.max_target_length;
	int count = 0, iter = 0, iter_num = 10;
	do {
		count = 0;
		update_target_length();
		for (int i = 0; i < NE; i++) {
			if (i > mesh.n_edges() - 1)break;
			Mesh::EdgeHandle e = mesh.edge_handle(i);
			if (mesh.status(e).deleted())continue;
			Mesh::HalfedgeHandle he = mesh.halfedge_handle(e);
			Mesh::VertexHandle vfrom = mesh.from_vertex_handle(he);
			bool he_clp_ok = true;
			if (!mesh.is_collapse_ok(he)) {  he_clp_ok = false; continue; }
			if (mesh.is_boundary(vfrom)) {  he_clp_ok = false; continue; }
			if (mesh.status(vfrom).locked()) {  he_clp_ok = false; continue; }
			if (!check_collapse_Condition(he)) {  he_clp_ok = false; continue; }
			if (he_clp_ok) {
				mesh.collapse(he);
				count++;
				//std::cout << "collapse a edge/*/*/*/*/*/\n";
				//bug_test();
				continue;
			}
			Mesh::HalfedgeHandle he_opp = mesh.opposite_halfedge_handle(he);
			Mesh::VertexHandle vto = mesh.to_vertex_handle(he);
			bool he_opp_clp_ok = true;
			if (!mesh.is_collapse_ok(he_opp)) { he_opp_clp_ok = false; continue; }
			if (mesh.is_boundary(vto)) {  he_opp_clp_ok = false; continue; }
			if (mesh.status(vto).locked()) {  he_opp_clp_ok = false; continue; }
			if (!check_collapse_Condition(he_opp)) { he_opp_clp_ok = false; continue; }
			if (he_opp_clp_ok) {
				mesh.collapse(he_opp);
				count++;
				//std::cout << "collapse a edge/*/*/*/*/*/\n";
				//bug_test();
				continue;
			}
		}
		iter++;
	} while (count > mesh.n_edges() / 100 && iter < iter_num);// while (count != 0 && iter < 10);//while (count > mesh.n_edges()/1000 && iter < 10)
	mesh.garbage_collection();
}

void IsotropicRemeshing::collapse_cross() {
	//update_target_length();
	int NV = mesh.n_vertices();
	int count = 0, iter = 0, iter_num = 10;
	do{
		count = 0;
		update_target_length();
		for (int i = 0; i < NV; i++) {
			if (i > mesh.n_vertices() - 1)break;
			Mesh::VertexHandle vh = mesh.vertex_handle(i);
			if (mesh.status(vh).deleted())continue;
			Mesh::HalfedgeHandle voheh_min = vertices_min_Out_Halfedge(vh);
			double voheh_min_len = mesh.calc_edge_length(voheh_min);
			if (voheh_min_len < mesh.property(e_min_target_length, mesh.edge_handle(voheh_min))) {
				if (mesh.is_collapse_ok(voheh_min) && check_collapse_Condition(voheh_min)) {
					mesh.collapse(voheh_min);
					count++;
					continue;
				}
				else {
					Mesh::HalfedgeHandle voheh_min_opp = mesh.opposite_halfedge_handle(voheh_min);
					if (mesh.is_collapse_ok(voheh_min_opp) && check_collapse_Condition(voheh_min_opp)) {
						mesh.collapse(voheh_min_opp);
						count++;
						continue;
					}
				}
			}
			Mesh::HalfedgeHandle voheh_max = vertices_max_Out_Halfedge(vh);
			double voheh_max_len = mesh.calc_edge_length(voheh_max);
			if (voheh_max_len / voheh_min_len >= 1.8) {
				if (mesh.is_collapse_ok(voheh_min) && check_collapse_Condition(voheh_min)) {
					mesh.collapse(voheh_min);
					count++;
					continue;
				}
				else {
					Mesh::HalfedgeHandle voheh_min_opp = mesh.opposite_halfedge_handle(voheh_min);
					if (mesh.is_collapse_ok(voheh_min_opp) && check_collapse_Condition(voheh_min_opp)) {
						mesh.collapse(voheh_min_opp);
						count++;
						continue;
					}
				}
			}
			if (mesh.status(vh).locked() || mesh.is_boundary(vh))continue;
			double valence = mesh.valence(vh);
			if (valence == 3 || valence == 4 || valence > 7) {
				for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_begin(vh); voh_it.is_valid(); voh_it++) {
					Mesh::HalfedgeHandle heh = *voh_it;
					if (mesh.is_collapse_ok(heh) && check_collapse_Condition(heh)) {
						mesh.collapse(heh);
						count++;
						break;
					}
					else {
						Mesh::HalfedgeHandle heh_opp = mesh.opposite_halfedge_handle(heh);
						if (mesh.is_collapse_ok(heh_opp) && check_collapse_Condition(heh_opp)) {
							mesh.collapse(heh_opp);
							count++;
							break;
						}
					}
				}
			}
		}
		iter++;
	}while ((count > mesh.n_vertices() / 100 && iter < iter_num));//count != 0 && iter<10
	mesh.garbage_collection();
}

void IsotropicRemeshing::equalize_valences() {
	update_crease();
	int deviation_pre, deviation_post;
	std::vector<int> target_valence;
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (mesh.is_boundary(v_it)) target_valence.push_back(4);
		else target_valence.push_back(6);
	}
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		Mesh::EdgeHandle e = mesh.edge_handle((*e_it).idx());

		if (mesh.status(e).feature() || mesh.property(is_crease, e))continue;// 
		if (mesh.is_boundary(e) || !is_flip_ok_openmesh(e, mesh))continue;
		if (!check_flip_Condition(e)) continue;
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

void IsotropicRemeshing::tangential_relaxation() {
	mesh.update_normals();
	int iter = 3;
	for(int i = 0;i < iter;i++){
		for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
			int kf = 0, k = 0, K = 0;
			Mesh::VertexHandle vh = *v_it;
			if (mesh.status(vh).deleted())continue;
			Mesh::Point p = mesh.point(vh); //if (p != p)std::cout << "p error +";
			bool v_is_feature = mesh.status(vh).feature();
			double Lp = mesh.property(v_target_length, vh);
			Mesh::Point q(0, 0, 0);
			double count = 0.0;
			if (mesh.is_boundary(vh)) continue;
			if (mesh.status(vh).locked()) continue;

			//double radius = DBL_MAX;//Calculate the radius that can be moved
			vector<Mesh::Point> feature_points;
			//if (para.adapt_flag) {
				for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh); voh_it.is_valid(); ++voh_it) {
					K++;
					Mesh::HalfedgeHandle heh0 = *voh_it;
					Mesh::EdgeHandle eh = mesh.edge_handle(heh0);
					Mesh::VertexHandle vh0 = mesh.to_vertex_handle(heh0);
					Mesh::Point p0 = mesh.point(vh0); //if (p0 != p0)std::cout << "p0 error +";
					Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh0)));
					//radius = min(radius, distPointLine(p, p0, p1));//q + (n.dot(p - q)) * n
					//radius = min(radius, distPointLine(p, p0 + (n.dot(p - p0)) * n, p1 + (n.dot(p - p1)) * n));
					if (v_is_feature) {
						//continue;
						if(mesh.status(eh).feature()){
							q += p0;
							feature_points.push_back(p0);
							count++;
							//double L = norm(p0 - p);
							//Mesh::Point qj = (p + p0) / 2.; //if (qj != qj)std::cout << "feature qj error +";
							////double Lqj = calAdaptTargetlength(heh0); //if (Lqj != Lqj)std::cout << "feature Lqj error \n";
							//double Lqj = mesh.property(e_min_target_length, eh) * 5 / 4;
							//q += L * Lqj * qj; //if (q != q)std::cout << " feature q error +";
							//count += L * Lqj;
							//double eh0_quality = quality_edge(mesh.edge_handle(heh0));
							//q += eh0_quality  * qj; //if (q != q)std::cout << " feature q error +";
							//count += eh0_quality ;
							kf++;
						}
					}
					else {
						Mesh::FaceHandle fh = mesh.face_handle(heh0);
						Mesh::HalfedgeHandle heh1 = mesh.ccw_rotated_halfedge_handle(heh0);
						Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh1);
						q += p0;
						++count;
						//Mesh::Point p1 = mesh.point(vh1); //if (p1 != p1)std::cout << "p1 error +";
						//double S = max(norm((p0 - p).cross(p1 - p)) / 2.,para.min_area); //if (S != S)std::cout << "S error +";
						////if (S == 0)continue;
						//Mesh::Point qj = (p + p0 + p1) / 3; //if (qj != qj)std::cout << "qj error +";
						////double Lp0 = calAdaptTargetlength(vh0);
						//double Lp0 = mesh.property(v_target_length, vh0);
						////double Lp1 = calAdaptTargetlength(vh1);
						//double Lp1 = mesh.property(v_target_length, vh1);
						//double Lqj = (Lp + Lp0 + Lp1) / 3.; //if (Lqj != Lqj)std::cout << "Lqj error \n";
						//q += S * Lqj * qj; //if (S * Lqj * qj != S * Lqj * qj)std::cout << "not feature S * Lqj * qj error +";
						//count += S * Lqj; //if (S * Lqj == 0)std::cout << "S=" << S << " Lqj=" << Lqj << "\n";
						//double f_quality = quality_face(fh);
						//q += f_quality  * qj; //if (S * Lqj * qj != S * Lqj * qj)std::cout << "not feature S * Lqj * qj error +";
						//count += f_quality ; //if (S * Lqj == 0)std::cout << "S=" << S << " Lqj=" << Lqj << "\n";
						/*if (q / count != q / count) {
							std::cout <<"S=:"<<S<<" Lqj="<<Lqj<< " q:" << q[0] << " " << q[1] << " " << q[2] << " count=" << count << "\n";
							std::cout << "Lp=" << Lp << " Lp0=" << Lp0 << " Lp1=" << Lp1 << "\n";
						}*/
						k++;
					}
				}
			//}
			//else {
				/*for (Mesh::VertexVertexIter vv_it = mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it)
				{
					q += mesh.point(*vv_it);
					++count;
				}*/
			//}
			//radius = para.tolerance;
			if (v_is_feature) {
				if (count != 2)std::cout << "一个特征点周围不是两条特征边！" << count << "\n";
				assert(count == 2);
				if (norm((feature_points[0] - p).cross(feature_points[1] - p)) != 0)
					continue;
			}
			assert(count != 0);
			q = q / count;
			//q = p + para.smooth_ratio * (q - p);
			//bool is_move_ok = true;
			//for (Mesh::VertexOHalfedgeCCWIter voh_ccwit = mesh.voh_ccwiter(vh); voh_ccwit.is_valid(); ++voh_ccwit) {
			//	Mesh::HalfedgeHandle heh0 = *voh_ccwit;
			//	Mesh::HalfedgeHandle heh0_next = mesh.next_halfedge_handle(heh0);
			//	Mesh::VertexHandle vh0 = mesh.to_vertex_handle(heh0);
			//	Mesh::Point p0 = mesh.point(vh0); //if (p0 != p0)std::cout << "p0 error +";
			//	Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh0_next));
			//	//Mesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh0_next)));

			//}
			//radius *= 0.9;
			int it = 0;
			//if (!check_relocate_Normal(vh, q)) {
			while (!check_relocate_Normal(vh, q) && it<5) {
				q = p + para.smooth_ratio * (q - p);
				it++;
			}
			if (it == 5)continue;
				/*if (norm(q - p) > radius && radius != 0) {
					q = p + radius * (q - p) / norm(q - p);
				}*/
			//}
			if (!mesh.status(vh).feature()) {
				Mesh::Normal n = mesh.calc_vertex_normal(*v_it);//if (n != n)std::cout << "n error +";//= mesh.normal(*v_it); 
				q = q + (n.dot(p - q)) * n;
			}
			// || mesh.property(v_plane_bound_flag, vh)
			//if (!mesh.property(v_plane_flag, vh)) {
			//
			//}
			mesh.set_point(*v_it, q);
		}
	}
}

void IsotropicRemeshing::project_to_surface() {
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
	{
		Mesh::VertexHandle fv = *v_it;
		if (mesh.status(fv).locked()|| mesh.status(fv).feature())continue;
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

void IsotropicRemeshing::update_bound() {
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		if (mesh.is_boundary(*v_it)) mesh.property(v_is_boundary, *v_it) = true;
	}
}

void IsotropicRemeshing::update_edge_feature(Mesh::EdgeHandle eh){
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh);
	Mesh::HalfedgeHandle heh_opp = mesh.opposite_halfedge_handle(heh);
	bool f = mesh.is_estimated_feature_edge(heh, para.angle_Threshold_Rad);
	mesh.status(eh).set_feature(f);//&& length > 0
	mesh.status(heh).set_feature(f);
	mesh.status(heh_opp).set_feature(f);
	if (f) {//&& length>0
		Mesh::VertexHandle v0 = mesh.from_vertex_handle(heh);
		Mesh::VertexHandle v1 = mesh.to_vertex_handle(heh);
		mesh.property(v_feature_edge_num, v0)++;
		mesh.property(v_feature_edge_num, v1)++;
	}
}

void IsotropicRemeshing::update_edges_feature(){
	int num = 0;
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		update_edge_feature(*e_it);
		if (mesh.status(*e_it).feature()) num++;
	}
	std::cout << "nums of feature edges:" << num << "\n";
}

void IsotropicRemeshing::update_vertice_feature(Mesh::VertexHandle vh){
	int num = mesh.property(v_feature_edge_num, vh);
	mesh.status(vh).set_feature((num == 2));
	mesh.status(vh).set_locked((num == 1 || num >= 3));
}

void IsotropicRemeshing::update_vertices_feature(){
	int num_f = 0, num_c = 0, num=0;
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		update_vertice_feature(*v_it);
		if (mesh.status(*v_it).feature()) {
			num += 2;
			num_f++;
		}
		if (mesh.status(*v_it).locked()) {
			num+=mesh.property(v_feature_edge_num, *v_it);//
			num_c++;
		}
	}
	std::cout << "nums of feature verts:" << num_f << "\n";
	std::cout << "nums of corner verts:" << num_c << "\n";
	std::cout << "nums of :2*edges:" << num << "\n";
}

void IsotropicRemeshing::update_feature(){
	update_edges_feature();
	update_vertices_feature();
	std::cout << "The features of the mesh have been labelled.. \n";
}

void IsotropicRemeshing::update_crease() {
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		Mesh::HalfedgeHandle heh = mesh.halfedge_handle(*e_it);
		bool f = mesh.is_estimated_feature_edge(heh, 0);
		mesh.property(is_crease, *e_it) = f;
	}
}

void IsotropicRemeshing::update_target_length() {
	update_vertices_target_length();
	update_edges_target_length();
}

void IsotropicRemeshing::update_vertice_target_length(Mesh::VertexHandle vh) {
	if (para.adapt_flag && !mesh.is_boundary(vh)) {
		mesh.property(v_target_length, vh) = calAdaptTargetlength(vh);
	}
	else {
		mesh.property(v_target_length, vh) = para.target_length;
	}
}

void IsotropicRemeshing::update_vertices_target_length() {
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		update_vertice_target_length(*v_it);
	}
}

void IsotropicRemeshing::update_edge_target_length(Mesh::EdgeHandle eh) {
	double min_L = para.min_target_length;
	double max_L = para.max_target_length;
	if (para.adapt_flag) {
		Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(eh, 0);
		Mesh::VertexHandle vh0 = mesh.from_vertex_handle(heh0);
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh0);
		double vh0_len = mesh.property(v_target_length, vh0);
		double vh1_len = mesh.property(v_target_length, vh1);
		double eh_tar_L = min(vh0_len, vh1_len);
		//double eh_tar_L = (vh0_len + vh1_len)/2;
		//double eh_tar_L = sqrt(vh0_len * vh1_len);
		max_L = min(eh_tar_L * 4 / 3, para.max_target_length);
		min_L = max_L * 3 / 5;
	}
	mesh.property(e_min_target_length, eh) = min_L;
	mesh.property(e_max_target_length, eh) = max_L;
}

void IsotropicRemeshing::update_edges_target_length() {
	for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
		Mesh::EdgeHandle eh = *e_it;
		update_edge_target_length(eh);
	}
}

void IsotropicRemeshing::clean_mesh(double quality) {
	std::cout << "clean mesh start\n";
	removeColinearFaces();
	repair_BadFaces(quality);
	//mesh.garbage_collection();
	std::cout << "clean mesh finish\n";
}

void IsotropicRemeshing::removeColinearFaces() {
	std::cout << "removeColinearFaces start\n";
	int iter = 0;
	int count = 0;
	Mesh::FaceIter f_end = mesh.faces_end();
	do{
		count = 0;
		int k = 0;
		update_target_length();
		for (Mesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++) {
			k++;
			Mesh::FaceHandle fh = *f_it;
			if (mesh.status(fh).deleted())continue;
			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
			Mesh::HalfedgeHandle heh_ = heh;
			double area = cal_3p_face_2Area(fh);//
			double q = quality_face(fh);
			if (q <= 0.001 || area <= para.min_area) {
				bool coincide = false;
				Mesh::VertexHandle vfrom = mesh.from_vertex_handle(heh_);
				Mesh::VertexHandle vto = mesh.to_vertex_handle(heh_);
				//判断是否有两点重合
				do {
					if (mesh.calc_edge_length(mesh.edge_handle(heh_)) == 0) {
						//std::cout << "+++++++++重合点" << k << "\n";
						//bug test start
						bool next_feature_flag = mesh.status(mesh.edge_handle(mesh.next_halfedge_handle(heh_))).feature();
						bool prev_feature_flag = mesh.status(mesh.edge_handle(mesh.prev_halfedge_handle(heh_))).feature();
						bool opp_next_feature_flag= mesh.status(mesh.edge_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh_)))).feature();
						bool opp_prev_feature_flag = mesh.status(mesh.edge_handle(mesh.prev_halfedge_handle(mesh.opposite_halfedge_handle(heh_)))).feature();
						/*std::cout << "最短边是否是特征边：" << mesh.status(mesh.edge_handle(heh_)).feature() << "\n";
						std::cout << "最短边两端点是否是特征点：" << mesh.status(vfrom).feature() << mesh.status(vfrom).locked() <<
							" " << mesh.status(vto).feature() << mesh.status(vto).locked() << "\n";
						std::cout << "第三个端点是否是特征点：" << mesh.status(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh_))).feature()
							<<mesh.status(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh_))).locked()<< "\n";
						std::cout << "第四个端点是否是特征点：" << mesh.status(mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh_)))).feature()
							<< mesh.status(mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh_)))).locked() << "\n";
						std::cout << "最短边下一边是否是特征边：" << next_feature_flag << "\n";
						std::cout << "最短边上一边是否是特征边：" << prev_feature_flag << "\n";
						std::cout << "最短边opp下一边是否是特征边：" << opp_next_feature_flag << "\n";
						std::cout << "最短边opp上一边是否是特征边：" << opp_prev_feature_flag << "\n";*/
						//bug test end
						if (mesh.is_collapse_ok(heh_) && (!para.aabb_flag||check_collapse_legal_feature(heh_))) {
							mesh.collapse(heh_);
							count++;
							//std::cout << "已删除重合点-heh\n";
							//bug_test();
						}
						else {
							Mesh::HalfedgeHandle heh_opp = mesh.opposite_halfedge_handle(heh_);
							if (mesh.is_collapse_ok(mesh.opposite_halfedge_handle(heh_))
							&& (!para.aabb_flag || check_collapse_legal_feature(mesh.opposite_halfedge_handle(heh_)))) {
							mesh.collapse(mesh.opposite_halfedge_handle(heh_));
							count++;
							//std::cout << "已删除重合点-hehopp\n";
							//bug_test();
						}
						}
						coincide = true;
						//bug_test();
						break;
					}
					heh_ = mesh.next_halfedge_handle(heh_);
					vfrom = mesh.from_vertex_handle(heh_);
					vto = mesh.to_vertex_handle(heh_);
				} while (heh_ != heh);

				//没有两点重合，取最长边，先判断能否flip；若不能，先split，再collapse。
				//TO MODIFY :如果都是feature 边，则不操作？
				if (!coincide) {
					//std::cout << "+++++++++非重合点" << k << "\n";
					Mesh::HalfedgeHandle longest_heh = heh;
					Mesh::EdgeHandle longest_eh = mesh.edge_handle(longest_heh);
					Mesh::HalfedgeHandle shortest_heh = heh;
					Mesh::EdgeHandle shortest_eh = mesh.edge_handle(shortest_heh);
					heh_ = mesh.next_halfedge_handle(heh);
					do {
						if (mesh.calc_edge_length(mesh.edge_handle(heh_)) > mesh.calc_edge_length(longest_eh)) {
							longest_heh = heh_;
						}
						else shortest_heh = heh_;
						heh_ = mesh.next_halfedge_handle(heh_);
					} while (heh_ != heh);
					longest_eh = mesh.edge_handle(longest_heh);
					shortest_eh = mesh.edge_handle(shortest_heh);
					double len_ratio = mesh.calc_edge_length(longest_eh) / mesh.calc_edge_length(shortest_eh);
					if (len_ratio >= 100 ) {//&& mesh.is_collapse_ok(shortest_heh) && check_collapse_condition(shortest_heh)
						if(mesh.is_collapse_ok(shortest_heh) && (!para.aabb_flag || check_collapse_Condition(shortest_heh))){
							mesh.collapse(shortest_heh);
							count++;
							//std::cout << "clp 100\n";
						}
					}
					if (!mesh.status(longest_eh).feature() && check_flip_Condition(longest_eh)) {
						flip_openmesh(longest_eh, mesh);
						count++;
						//std::cout << "flip\n";
						continue;
					}
					//Mesh::HalfedgeHandle h0 = mesh.halfedge_handle(longest_eh, 0);
					//Mesh::HalfedgeHandle h1 = mesh.halfedge_handle(longest_eh, 1);
					//Mesh::Point p = mesh.calc_centroid(h0);
					//Mesh::VertexHandle v_new = mesh.add_vertex(p);
					//split_edge(longest_eh, v_new); std::cout << "split\n";
					////bug_test();
					////std::cout << "collapse\n";
					//Mesh::HalfedgeHandle ph0 = mesh.prev_halfedge_handle(h0);
					//if(quality_halfedge(ph0)<=0.001){
					////if (mesh.calc_edge_length(mesh.edge_handle(ph0)) == 0) {
					//	std::cout << "ph0\n";
					//	//if (!mesh.has_halfedge_status()) {
					//	//	mesh.collapse(ph0); std::cout << "clp ph0 非重合点已删除\n";
					//	//	//bug_test();
					//	//	count++;
					//	//	continue;
					//	//}
					//	if(mesh.is_collapse_ok(ph0) && check_collapse_condition(ph0)){
					//		mesh.collapse(ph0); std::cout << "clp ph0 非重合点已删除\n";
					//		//bug_test();
					//		count++;
					//	}
					//	else {
					//		Mesh::HalfedgeHandle ph0_opp = mesh.opposite_halfedge_handle(ph0);
					//		if (mesh.is_collapse_ok(ph0_opp) && check_collapse_condition(ph0_opp)){
					//			mesh.collapse(ph0_opp); std::cout << "clp ph0_opp 非重合点已删除\n";
					//			//bug_test();
					//			count++;
					//		}
					//	}
					//}
					//Mesh::HalfedgeHandle nh1 = mesh.next_halfedge_handle(h1);
					//if(quality_halfedge(nh1)<=0.001){
					////if (mesh.calc_edge_length(mesh.edge_handle(nh1)) == 0) {
					//	std::cout << "nh1\n";
					//	//if (!mesh.has_halfedge_status()) {
					//	//	mesh.collapse(nh1); std::cout << "clp nh1 非重合点已删除\n";
					//	//	//bug_test();
					//	//	count++;
					//	//	continue;
					//	//}
					//	if (mesh.is_collapse_ok(nh1) && check_collapse_condition(nh1)) {
					//		mesh.collapse(mesh.opposite_halfedge_handle(nh1)); std::cout << "clp nh0 非重合点已删除\n";
					//		//bug_test();
					//		count++;
					//	}
					//	else {
					//		Mesh::HalfedgeHandle nh1_opp = mesh.opposite_halfedge_handle(nh1);
					//		if (mesh.is_collapse_ok(nh1_opp) && check_collapse_condition(nh1_opp)){
					//			mesh.collapse(nh1_opp); std::cout << "clp nh1_opp 非重合点已删除\n";
					//			//bug_test();
					//			count++;
					//		}
					//	}
					//}
				}
			}
		}
		iter++;
		std::cout << iter << "\n";
	} while (count != 0 && iter < 45);
	mesh.garbage_collection();
	std::cout << "removeColinearFaces finish\n";
}

void IsotropicRemeshing::repair_BadFaces(double quality) {
	std::cout << "repair_BadFaces start\n";
	int iter = 0;
	int count = 0;
	do {
		count = 0;
		int k = 0;
		update_target_length();
		for (Mesh::FaceIter f_it = mesh.faces_begin(); f_it != mesh.faces_end(); f_it++) {
			k++;
			Mesh::FaceHandle fh = *f_it;
			if (mesh.status(fh).deleted())continue;
			Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
			Mesh::HalfedgeHandle heh_ = heh;
			double q = quality_face(fh);
			if (q <= quality) {//||check_angle(fh)
				Mesh::HalfedgeHandle shortest_heh = heh;
				Mesh::EdgeHandle shortest_eh = mesh.edge_handle(shortest_heh);
				heh_ = mesh.next_halfedge_handle(heh);
				do {
					if (mesh.calc_edge_length(mesh.edge_handle(heh_)) < mesh.calc_edge_length(shortest_eh)) {
						shortest_heh = heh_;
						shortest_eh = mesh.edge_handle(shortest_heh);
					}
					heh_ = mesh.next_halfedge_handle(heh_);
				} while (heh_ != heh);
				//flip_openmesh(longest_eh, mesh);
				Mesh::VertexHandle vfrom = mesh.from_vertex_handle(shortest_heh);
				Mesh::VertexHandle vto = mesh.to_vertex_handle(shortest_heh);
				//
				if (mesh.is_boundary(vfrom) || mesh.is_boundary(vto))continue;
				if (!mesh.has_halfedge_status()) {
					mesh.collapse(shortest_heh);
					count++;
				}
				if (mesh.is_collapse_ok(shortest_heh) && check_collapse_Condition(shortest_heh)) {
					if (!mesh.status(vfrom).locked()) {
						mesh.collapse(shortest_heh);
						count++;
						//std::cout << "bad face\n";
					}
				}
				if (!mesh.has_halfedge_status()) {
					mesh.collapse(mesh.opposite_halfedge_handle(shortest_heh));
					count++;
				}
				else if (mesh.is_collapse_ok(mesh.opposite_halfedge_handle(shortest_heh)) && 
					check_collapse_Condition(mesh.opposite_halfedge_handle(shortest_heh))) {
					if (!mesh.status(vto).locked()) {
					mesh.collapse(mesh.opposite_halfedge_handle(shortest_heh));
					count++;
					//std::cout << "bad face\n";
					}
				}
			}
		}
		iter++;
		std::cout << iter << "\n";
	} while (count != 0 && iter < 25);
	std::cout << "repair_BadFaces finish\n";
	mesh.garbage_collection();
}


double IsotropicRemeshing::quality(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2) {
	if (cal_3p_face_2Area(p0, p1, p2) == 0)return 0;
	double a = norm(p0 - p1);
	double b = norm(p1 - p2);
	double c = norm(p2 - p0);
	double p = (a+b+c)/2;
	double Area2 = (p - a) * (p - b) * (p - c) * p;
	return 8 * Area2 / (a * b * c * p);
}

double IsotropicRemeshing::quality_face(Mesh::FaceHandle fh) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
	assert(mesh.is_valid_handle(heh));
	Mesh::Point p0 = mesh.point(mesh.from_vertex_handle(heh));
	Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh));
	Mesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));
	return quality(p0, p1, p2);
}

double IsotropicRemeshing::quality_halfedge(Mesh::HalfedgeHandle heh) {
	Mesh::FaceHandle fh = mesh.face_handle(heh);
	return quality_face(fh);
}

double IsotropicRemeshing::quality_edge(Mesh::EdgeHandle heh) {
	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(heh, 0);
	Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(heh, 1);
	return min(quality_halfedge(heh0), quality_halfedge(heh1));
}



double IsotropicRemeshing::calAdaptTargetlength(Mesh::VertexHandle vh){
	double adp_len = 0;//mesh.ccw_rotated_halfedge_handle : ccw 逆时针,cw 顺时针
	double H, K;
	cal_curvature_And_Area_Voronoi(vh, H, K);
	double H2K = H * H - K;
	H2K = H2K >= 0 ? H2K : 0;
	double max_curvature = H + sqrt(H2K);
	double error = para.tolerance;
	//assert((max_curvature != 0 && max_curvature < 2 / error)||max_curvature==0);
	/*if (max_curvature > 2 / error) adp_len = para.min_length;
	else {
		adp_len = max_curvature == 0 ? para.target_length : sqrt(6 * error / max_curvature - 3 * error * error);
	}*/
	max_curvature = max_curvature < 2 / error ? max_curvature : 2 / error - error;
	adp_len = max_curvature == 0 ? para.target_length : sqrt( 6 * error / max_curvature - 3 * error * error);
	//assert(max_curvature >= 0);
	//if (!isfinite(max_curvature))max_curvature = log(para.target_length / para.min_length) * error;
	//adp_len = para.target_length * exp(-max_curvature / error);
	return adp_len;
}

double IsotropicRemeshing::calAdaptTargetlength(Mesh::HalfedgeHandle heh) {
	Mesh::VertexHandle v0 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle v1 = mesh.to_vertex_handle(heh);
	return min(calAdaptTargetlength(v0), calAdaptTargetlength(v1));
}

double IsotropicRemeshing::calAdaptTargetlength(Mesh::EdgeHandle eh) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh);
	return calAdaptTargetlength(heh);
}

double IsotropicRemeshing::cal_curvature_And_Area_Voronoi(Mesh::VertexHandle vh, double& H, double& K) {
	double Area_Voronoi = 0;
	Mesh::Point sum_vec(0, 0, 0);
	double sum_theta = 0;
	Mesh::Point p = mesh.point(vh);
	int k_H = 0;
	for (Mesh::VertexOHalfedgeIter voh_it=mesh.voh_iter(vh); voh_it.is_valid(); voh_it++) {
		
		Mesh::HalfedgeHandle heh = *voh_it;
		Mesh::HalfedgeHandle heh0 = mesh.cw_rotated_halfedge_handle(heh);
		Mesh::HalfedgeHandle heh1 = mesh.ccw_rotated_halfedge_handle(heh);
		Mesh::FaceHandle face = mesh.face_handle(heh);

		Mesh::Point pto = mesh.point(mesh.to_vertex_handle(heh));
		Mesh::Point p0 = mesh.point(mesh.to_vertex_handle(heh0));
		Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh1));

		Mesh::Point ppto = pto - p;

		Mesh::Point p0p = p - p0, p0pto = pto - p0;
		Mesh::Point p1p = p - p1, p1pto = pto - p1;
		double cot_alphaij = p0p.dot(p0pto) / norm(p0p.cross(p0pto));
		double cot_betaij= p1p.dot(p1pto) / norm(p1p.cross(p1pto));
		sum_vec += (cot_alphaij + cot_betaij) * ppto; k_H++;
		//assert(norm(sum_vec) >= 0);

		Mesh::Point pp1 = p1 - p;
		double costheta = pp1.dot(ppto) / (norm(pp1) * norm(ppto));
		sum_theta += acos(costheta);

		if (costheta < 0) {
			Area_Voronoi += norm(pp1.cross(ppto)) / 2;
			//assert(Area_Voronoi >= 0);
		}
		else if (ppto.dot(p1pto) < 0 || p1p.dot(p1pto) < 0) {
			Area_Voronoi += norm(pp1.cross(ppto)) / 4;
			//assert(Area_Voronoi >= 0);
		}
		else {
			double a = norm(ppto), b = norm(pp1), c = norm(p1pto);
			double s = (a + b + c) / 2;
			double S = sqrt(s * (s - a) * (s - b) * (s - c));
			double R = a * b * c / (4 * S);
			double ha = sqrt(R * R - a * a / 4), hb = sqrt(R * R - b * b / 4);
			Area_Voronoi += (a * ha + b * hb) / 4;
			//assert(Area_Voronoi >= 0);
		}
	}
	H = norm(sum_vec) / (4 * Area_Voronoi);
	//assert(H >= 0);
	/*if (H == 0) {
		std::cout << "k_H=" << k_H << "sunvec:" << sum_vec[0] << sum_vec[1] << sum_vec[2] << "\n";
	}*/
	K = (2 * PI - sum_theta) / Area_Voronoi;
	return Area_Voronoi;

}

//not needed for now
double IsotropicRemeshing::cal_Voronoi_Area_mixed(Mesh::VertexHandle vh) {
	double Area_Voronoi = 0;

	return Area_Voronoi;
}

//not needed for now
Mesh::Point IsotropicRemeshing::cal_face_Circumcenter(Mesh::FaceHandle fh) {
	Mesh::Point Circumcenter(0, 0, 0);

	return Circumcenter;
}

Mesh::Normal IsotropicRemeshing::cal_3p_face_normal(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2) {
	Mesh::Normal p0p1 = p1 - p0;
	Mesh::Normal p0p2 = p2 - p0;
	Mesh::Normal n = p0p1.cross(p0p2);
	n = norm(n) == 0 ? n : n / norm(n);
	//assert(norm(n) == 0 || norm(n) == 1);
	return n;
}

Mesh::Normal IsotropicRemeshing::cal_3p_face_normal(Mesh::FaceHandle fh) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
	Mesh::Point p0 = mesh.point(mesh.from_vertex_handle(heh));
	Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh));
	Mesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));
	return cal_3p_face_normal(p0, p1, p2);
}

double IsotropicRemeshing::cal_3p_face_2Area(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2) {
	Mesh::Normal p0p1 = p1 - p0;
	Mesh::Normal p0p2 = p2 - p0;
	/*if (norm(p0p1.cross(p0p2)) == 0) {
		std::cout <<setprecision(15)<<norm(p0p1)<<" "<<norm(p0p2) << "\n";
	}*/
	return norm(p0p1.cross(p0p2));
}

double IsotropicRemeshing::cal_3p_face_2Area(Mesh::FaceHandle fh) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(fh);
	Mesh::Point p0 = mesh.point(mesh.from_vertex_handle(heh));
	Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh));
	Mesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));
	return cal_3p_face_2Area(p0, p1, p2);
}

bool IsotropicRemeshing::check_angle_max(Mesh::FaceHandle fh) {
	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(fh);
	if (!check_angle_max(heh0)) return false;
	Mesh::HalfedgeHandle heh1 = mesh.next_halfedge_handle(heh0);
	if (!check_angle_max(heh1)) return false;
	Mesh::HalfedgeHandle heh2 = mesh.next_halfedge_handle(heh1);
	if (!check_angle_max(heh2)) return false;
	return true;
}

bool IsotropicRemeshing::check_angle_max(Mesh::HalfedgeHandle heh) {
	Mesh::VertexHandle vh0 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh);
	Mesh::VertexHandle vh2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
	Mesh::Point p0 = mesh.point(vh0);
	Mesh::Point p1 = mesh.point(vh1);
	Mesh::Point p2 = mesh.point(vh2);
	Mesh::Point p2p0 = p0 - p2;
	Mesh::Point p2p1 = p1 - p2;
	double costheta = p2p0.dot(p2p1) / (norm(p2p0) * norm(p2p1));
	if (costheta >= para.cos_max_angle)return true;
	return false;
}

bool IsotropicRemeshing::check_angle_max(Mesh::EdgeHandle eh) {
	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(eh, 0);
	if (!check_angle_max(heh0)) return false;
	Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 1);
	if (!check_angle_max(heh1)) return false;
	return true;
}

bool IsotropicRemeshing::check_angle_min(Mesh::FaceHandle fh) {
	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(fh);
	if (!check_angle_min(heh0)) return false;
	Mesh::HalfedgeHandle heh1 = mesh.next_halfedge_handle(heh0);
	if (!check_angle_min(heh1)) return false;
	Mesh::HalfedgeHandle heh2 = mesh.next_halfedge_handle(heh1);
	if (!check_angle_min(heh2)) return false;
	return true;
}

bool IsotropicRemeshing::check_angle_min(Mesh::HalfedgeHandle heh) {
	Mesh::VertexHandle vh0 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle vh1 = mesh.to_vertex_handle(heh);
	Mesh::VertexHandle vh2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
	Mesh::Point p0 = mesh.point(vh0);
	Mesh::Point p1 = mesh.point(vh1);
	Mesh::Point p2 = mesh.point(vh2);
	Mesh::Point p2p0 = p0 - p2;
	Mesh::Point p2p1 = p1 - p2;
	double costheta = p2p0.dot(p2p1) / (norm(p2p0) * norm(p2p1));
	if (costheta <= para.cos_min_angle)return true;
	return false;
}

bool IsotropicRemeshing::check_angle_min(Mesh::EdgeHandle eh) {
	Mesh::HalfedgeHandle heh0 = mesh.halfedge_handle(eh, 0);
	if (!check_angle_min(heh0)) return false;
	Mesh::HalfedgeHandle heh1 = mesh.halfedge_handle(eh, 1);
	if (!check_angle_min(heh1)) return false;
	return true;
}

bool IsotropicRemeshing::check_angle(Mesh::FaceHandle fh) {
	return check_angle_min(fh) && check_angle_max(fh);
}

bool IsotropicRemeshing::check_angle(Mesh::HalfedgeHandle heh) {
	return check_angle_min(heh) && check_angle_max(heh);
}

bool IsotropicRemeshing::check_angle(Mesh::EdgeHandle eh) {
	return check_angle_min(eh) && check_angle_max(eh);
}

bool IsotropicRemeshing::check_collapse_legal_feature(Mesh::HalfedgeHandle heh) {
	//TODO
	Mesh::VertexHandle vh1 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle vh2 = mesh.to_vertex_handle(heh);
	Mesh::VertexHandle vh3 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
	Mesh::VertexHandle vh4 = mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh)));
	int v1_fes = cal_vertice_feature_edges(vh1);
	int v2_fes = cal_vertice_feature_edges(vh2);
	int v3_fes = cal_vertice_feature_edges(vh3);
	int v4_fes = cal_vertice_feature_edges(vh4);
	bool heh_is_feature = mesh.status(mesh.edge_handle(heh)).feature();
	int r = int(heh_is_feature);
	Mesh::HalfedgeHandle heh_prev = mesh.prev_halfedge_handle(heh);
	bool heh_prev_is_feature = mesh.status(mesh.edge_handle(heh_prev)).feature();
	int s = int(heh_prev_is_feature);
	Mesh::HalfedgeHandle heh_opp = mesh.opposite_halfedge_handle(heh);
	Mesh::HalfedgeHandle heh_opp_next = mesh.next_halfedge_handle(heh_opp);
	bool heh_opp_next_is_feature = mesh.status(mesh.edge_handle(heh_opp_next)).feature();
	int t = int(heh_opp_next_is_feature);
	if (!check_legal_feature_corner(vh2, v2_fes + v1_fes - 2 * r - s - t))return false;
	if (!check_legal_feature_corner(vh3, v3_fes - s))return false;
	if (!check_legal_feature_corner(vh4, v4_fes - t))return false;
	return true;
}

bool IsotropicRemeshing::check_collapse_CanMove(Mesh::HalfedgeHandle heh) {
	Mesh::EdgeHandle e = mesh.edge_handle(heh);
	Mesh::VertexHandle vhb = mesh.from_vertex_handle(heh);
	if (mesh.status(vhb).locked())return false;
	Mesh::VertexHandle vhe = mesh.to_vertex_handle(heh);
	Mesh::Point pb = mesh.point(vhb);
	Mesh::Point pe = mesh.point(vhe);
	double min_target_L = para.min_target_length;
	double target_L = para.target_length;
	double max_target_L = para.max_target_length;
	//if (para.adapt_flag) {
	//	double adp_L = calAdaptTargetlength(e);
	//	if (adp_L < target_L) {
	//		min_target_L = adp_L * 4 / 5;
	//		max_target_L = adp_L * 4 / 3;
	//	}
	//	/*if (adp_L >= min_target_L && adp_L < target_L) {
	//		max_target_L = adp_L * 4 / 3;
	//	}
	//	if (adp_L >= target_L && adp_L < max_target_L) {
	//		min_target_L = adp_L * 4 / 5;
	//	}*/
	//}
	min_target_L = mesh.property(e_min_target_length, e);
	double length = mesh.calc_edge_length(e);
	if (length < min_target_L) {
		bool is_collapse = true;
		for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_begin(vhb); voh_it.is_valid(); voh_it++) {
			Mesh::EdgeHandle e = mesh.edge_handle(*voh_it);
			Mesh::VertexHandle v = mesh.to_vertex_handle(*voh_it);
		//for (Mesh::VertexVertexIter vv_it = mesh.vv_iter(vhb); vv_it.is_valid(); ++vv_it) {
			//Mesh::VertexHandle v = *vv_it;
			double L = norm(mesh.point(v) - pe);
			if (L > mesh.property(e_max_target_length, e)) {
			//if (L > para.max_target_length) {
				is_collapse = false;
				break;
			}
		}
		is_collapse |= (length == 0 && check_collapse_legal_feature(heh));

		if (!is_collapse) return false;
	}
	else return false;
	if (mesh.status(vhb).feature()) {
		//if (!mesh.status(e).feature())return false;
		//Collapsing along one feature edge cannot affect another feature edge
		for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vhb); voh_it.is_valid(); ++voh_it) {
			Mesh::HalfedgeHandle voh = *voh_it;
			if (voh == heh)continue;
			// Feature points can only move along the direction of feature edge
			if (mesh.status(mesh.edge_handle(voh)).feature()) {
				Mesh::HalfedgeHandle he = voh;
				Mesh::Point p0 = mesh.point(mesh.from_vertex_handle(he));//p0==pb
				Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(he));
				//if (norm(pe - pb) == 0) return true;
				Mesh::Normal pbe = (pe - pb).normalized();
				Mesh::Normal p01 = (p1 - p0).normalized();
				if (norm(pbe.cross(p01)) != 0 && norm(p1 - p0) != 0) return false;
			}
		}
	}
	return true;
}

bool IsotropicRemeshing::check_collapse_Quality(Mesh::HalfedgeHandle heh) {
	Mesh::EdgeHandle e = mesh.edge_handle(heh);
	Mesh::VertexHandle vhb = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle vhe = mesh.to_vertex_handle(heh);
	Mesh::Point pb = mesh.point(vhb);
	Mesh::Point pe = mesh.point(vhe);
	for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vhb); voh_it.is_valid(); ++voh_it) {
		Mesh::HalfedgeHandle vohh = *voh_it;
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(vohh);
		Mesh::VertexHandle vh2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(vohh));
		Mesh::Point p1 = mesh.point(vh1);
		Mesh::Point p2 = mesh.point(vh2);

		//The quality of the mesh should not be too bad after collapse
		double newQ = quality(pe, p1, p2);
		//if (newQ < 0.001)return false;
		double oldQ = quality(pb, p1, p2);
		if (newQ <= 0.5 * oldQ && newQ != 0)return false;
	}
	return true;
}

bool IsotropicRemeshing::check_relocate_Normal(Mesh::VertexHandle vh, Mesh::Point q) {
	Mesh::Point p = mesh.point(vh);
	for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vh); voh_it.is_valid(); ++voh_it) {
		Mesh::HalfedgeHandle vohh = *voh_it;
		Mesh::HalfedgeHandle vohh_next = mesh.next_halfedge_handle(vohh);
		Mesh::HalfedgeHandle vohh_next_opp = mesh.opposite_halfedge_handle(vohh_next);
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(vohh);
		Mesh::VertexHandle vh2 = mesh.to_vertex_handle(vohh_next);
		Mesh::Point p1 = mesh.point(vh1);
		Mesh::Point p2 = mesh.point(vh2);
		Mesh::Normal newN = cal_3p_face_normal(q, p1, p2);
		Mesh::Normal vohh_next_opp_N = mesh.calc_face_normal(mesh.face_handle(vohh_next_opp));
		if (!mesh.status(vohh_next).feature()) {
			if (newN.dot(vohh_next_opp_N) < para.cos_angle_Threshold && norm(vohh_next_opp_N)!=0) return false;
		}
		Mesh::Normal oldN = cal_3p_face_normal(p, p1, p2);
		if (newN.dot(oldN) < 0.7f && norm(oldN) != 0)  return false;//0.9
	}
	return true;
}

bool IsotropicRemeshing::check_collapse_Normal(Mesh::HalfedgeHandle heh) {
	Mesh::EdgeHandle e = mesh.edge_handle(heh);
	Mesh::VertexHandle vhb = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle vhe = mesh.to_vertex_handle(heh);
	Mesh::Point pb = mesh.point(vhb);
	Mesh::Point pe = mesh.point(vhe);
	for (Mesh::VertexOHalfedgeIter voh_it = mesh.voh_iter(vhb); voh_it.is_valid(); ++voh_it) {
		Mesh::HalfedgeHandle vohh = *voh_it;
		Mesh::HalfedgeHandle vohh_next = mesh.next_halfedge_handle(vohh);
		Mesh::HalfedgeHandle vohh_opp = mesh.opposite_halfedge_handle(vohh);
		Mesh::HalfedgeHandle vohh_opp_next = mesh.opposite_halfedge_handle(vohh_opp);
		Mesh::VertexHandle vh1 = mesh.to_vertex_handle(vohh);
		Mesh::VertexHandle vh2 = mesh.to_vertex_handle(vohh_next);
		Mesh::VertexHandle vh3 = mesh.to_vertex_handle(vohh_opp_next);
		if (vh1 == vhe || vh2 == vhe || vh3 == vhe)continue;
		Mesh::Point p1 = mesh.point(vh1);
		Mesh::Point p2 = mesh.point(vh2);
		Mesh::Point p3 = mesh.point(vh3);

		//Normal vectors are not excessively deflected after collapse.
		Mesh::Normal vohh_newN = cal_3p_face_normal(pe, p1, p2);
		Mesh::Normal vohh_opp_newN = cal_3p_face_normal(pe, p3, p1);
		if (!mesh.status(vohh).feature()) {
			if (vohh_newN.dot(vohh_opp_newN) < para.cos_angle_Threshold && norm(vohh_opp_newN) * norm(vohh_newN) != 0)
				return false;
		}
		Mesh::Normal vohh_oldN = cal_3p_face_normal(pb, p1, p2);
		if (vohh_oldN.dot(vohh_newN) <= 0.7f && norm(vohh_oldN) != 0) {
			return false;
		}
	}
	return true;
}

bool IsotropicRemeshing::check_collapse_Condition(Mesh::HalfedgeHandle heh) {
	if (!check_collapse_CanMove(heh))return false;
	if (!check_collapse_Normal(heh))return false;
	//add::check_collapse_haussdorf_dist
	//if (!check_collapse_Quality(heh))return false;
	if (para.feature_flag && !check_collapse_legal_feature(heh))return false;
	return true;
}

//可以加：折痕不能flip，后续再加
bool IsotropicRemeshing::check_flip_Condition(Mesh::EdgeHandle eh) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(eh);
	Mesh::Point p0 = mesh.point(mesh.from_vertex_handle(heh));
	Mesh::Point p1 = mesh.point(mesh.to_vertex_handle(heh));
	Mesh::Point p2 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(heh)));
	Mesh::Point p3 = mesh.point(mesh.to_vertex_handle(mesh.next_halfedge_handle(mesh.opposite_halfedge_handle(heh))));

	// check quadrilateral with similar concave shape
	Mesh::Point p01_min, p23_min;
	double dist = distLineLine(p0, p1, p2, p3, p01_min, p23_min);
	if ((p01_min == p0 || p01_min == p1)) {
		return false;
	}
	// flip should not produce triangles that are too small or too angular.
	double old_min_cosAngle = min(min_cosAngle_triangle(p0, p1, p2), min_cosAngle_triangle(p0, p1, p3));
	double old_max_cosAngle = max(max_cosAngle_triangle(p0, p1, p2), max_cosAngle_triangle(p0, p1, p3));
	double new_min_cosAngle = min(min_cosAngle_triangle(p0, p2, p3), min_cosAngle_triangle(p1, p2, p3));
	double new_max_cosAngle = max(max_cosAngle_triangle(p0, p2, p3), max_cosAngle_triangle(p1, p2, p3));
	if (new_min_cosAngle < para.cos_max_angle && new_min_cosAngle < old_min_cosAngle)return false;
	if (new_max_cosAngle > para.cos_min_angle && new_max_cosAngle > old_max_cosAngle)return false;
	return true;
}

Mesh::HalfedgeHandle IsotropicRemeshing::vertices_min_Out_Halfedge(Mesh::VertexHandle vh) {
	Mesh::HalfedgeHandle heh_begin = mesh.halfedge_handle(vh);
	Mesh::HalfedgeHandle heh = heh_begin;
	Mesh::HalfedgeHandle heh_min = heh_begin;
	bool f=true;
	do {
		if (mesh.status(heh).deleted()) f = false;
		if (f && mesh.calc_edge_length(heh) < mesh.calc_edge_length(heh_min)) {
			heh_min = heh;
		}
		heh = mesh.ccw_rotated_halfedge_handle(heh);
		assert(mesh.from_vertex_handle(heh) == vh);
	} while (heh != heh_begin);
	return heh_min;
}

Mesh::HalfedgeHandle IsotropicRemeshing::vertices_max_Out_Halfedge(Mesh::VertexHandle vh) {
	Mesh::HalfedgeHandle heh_begin = mesh.halfedge_handle(vh);
	Mesh::HalfedgeHandle heh = heh_begin;
	Mesh::HalfedgeHandle heh_max = heh_begin;
	do {
		if (mesh.calc_edge_length(heh) > mesh.calc_edge_length(heh_max)) {
			heh_max = heh;
		}
		heh = mesh.ccw_rotated_halfedge_handle(heh);
	} while (heh != heh_begin);
	return heh_max;
}

double IsotropicRemeshing::distLineLine(Mesh::Point p00, Mesh::Point p01, 
	Mesh::Point p10, Mesh::Point p11, Mesh::Point& min_p0, Mesh::Point& min_p1) {

	Mesh::Point kDiff = p00 - p10;
	Mesh::Point d0 = p01 - p00;
	Mesh::Point d1 = p11 - p10;

	double fA00 = d0.sqrnorm();
	double fA01 = -(d0 | d1);
	double fA11 = d1.sqrnorm();
	double fB0 = (kDiff | d0);
	double fC = kDiff.sqrnorm();
	double fDet = fabs(fA00 * fA11 - fA01 * fA01);
	double fB1, fS, fT, fSqrDist, fTmp;

	// fB1 = -(kDiff | d1); 
	// [fA00 fA01][s] = [-fB0]		s = (fA01 * fB1 - fA11 * fB0)/fDet
	// [fA01 fA11][t] = [-fB1]		t = (fA01 * fB0 - fA00 * fB1)/fDet

	if (fDet >= FLT_MIN)
	{
		// line segments are not parallel
		fB1 = -(kDiff | d1);
		fS = fA01 * fB1 - fA11 * fB0;
		fT = fA01 * fB0 - fA00 * fB1;


		if (fS >= 0.0)
		{
			if (fS <= fDet)
			{
				if (fT >= 0.0)
				{
					if (fT <= fDet)  // region 0 (interior)
					{
						// minimum at two interior points of 3D lines
						double fInvDet = 1.0 / fDet;
						fS *= fInvDet;
						fT *= fInvDet;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0 * fB0) +
							fT * (fA01 * fS + fA11 * fT + 2.0 * fB1) + fC;
					}
					else  // region 3 (side)
					{
						fT = 1.0;
						fTmp = fA01 + fB0;
						if (fTmp >= 0.0)
						{
							fS = 0.0;
							fSqrDist = fA11 + 2.0 * fB1 + fC;
						}
						else if (-fTmp >= fA00)
						{
							fS = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB1 + fTmp);
						}
						else
						{
							fS = -fTmp / fA00;
							fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
						}
					}
				}
				else  // region 7 (side)
				{
					fT = 0.0;
					if (fB0 >= 0.0)
					{
						fS = 0.0;
						fSqrDist = fC;
					}
					else if (-fB0 >= fA00)
					{
						fS = 1.0;
						fSqrDist = fA00 + 2.0 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
			}
			else
			{
				if (fT >= 0.0)
				{
					if (fT <= fDet)  // region 1 (side)
					{
						fS = 1.0;
						fTmp = fA01 + fB1;
						if (fTmp >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fA00 + 2.0 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
						}
						else
						{
							fT = -fTmp / fA11;
							fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
						}
					}
					else  // region 2 (corner)
					{
						fTmp = fA01 + fB0;
						if (-fTmp <= fA00)
						{
							fT = 1.0;
							if (fTmp >= 0.0)
							{
								fS = 0.0;
								fSqrDist = fA11 + 2.0 * fB1 + fC;
							}
							else
							{
								fS = -fTmp / fA00;
								fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
							}
						}
						else
						{
							fS = 1.0;
							fTmp = fA01 + fB1;
							if (fTmp >= 0.0)
							{
								fT = 0.0;
								fSqrDist = fA00 + 2.0 * fB0 + fC;
							}
							else if (-fTmp >= fA11)
							{
								fT = 1.0;
								fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
							}
							else
							{
								fT = -fTmp / fA11;
								fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
							}
						}
					}
				}
				else  // region 8 (corner)
				{
					if (-fB0 < fA00)
					{
						fT = 0.0;
						if (fB0 >= 0.0)
						{
							fS = 0.0;
							fSqrDist = fC;
						}
						else
						{
							fS = -fB0 / fA00;
							fSqrDist = fB0 * fS + fC;
						}
					}
					else
					{
						fS = 1.0;
						fTmp = fA01 + fB1;
						if (fTmp >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fA00 + 2.0 * fB0 + fC;
						}
						else if (-fTmp >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB0 + fTmp);
						}
						else
						{
							fT = -fTmp / fA11;
							fSqrDist = fTmp * fT + fA00 + 2.0 * fB0 + fC;
						}
					}
				}
			}
		}
		else
		{
			if (fT >= 0.0)
			{
				if (fT <= fDet)  // region 5 (side)
				{
					fS = 0.0;
					if (fB1 >= 0.0)
					{
						fT = 0.0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1.0;
						fSqrDist = fA11 + 2.0 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
				else  // region 4 (corner)
				{
					fTmp = fA01 + fB0;
					if (fTmp < 0.0)
					{
						fT = 1.0;
						if (-fTmp >= fA00)
						{
							fS = 1.0;
							fSqrDist = fA00 + fA11 + fC + 2.0 * (fB1 + fTmp);
						}
						else
						{
							fS = -fTmp / fA00;
							fSqrDist = fTmp * fS + fA11 + 2.0 * fB1 + fC;
						}
					}
					else
					{
						fS = 0.0;
						if (fB1 >= 0.0)
						{
							fT = 0.0;
							fSqrDist = fC;
						}
						else if (-fB1 >= fA11)
						{
							fT = 1.0;
							fSqrDist = fA11 + 2.0 * fB1 + fC;
						}
						else
						{
							fT = -fB1 / fA11;
							fSqrDist = fB1 * fT + fC;
						}
					}
				}
			}
			else   // region 6 (corner)
			{
				if (fB0 < 0.0)
				{
					fT = 0.0;
					if (-fB0 >= fA00)
					{
						fS = 1.0;
						fSqrDist = fA00 + 2.0 * fB0 + fC;
					}
					else
					{
						fS = -fB0 / fA00;
						fSqrDist = fB0 * fS + fC;
					}
				}
				else
				{
					fS = 0.0;
					if (fB1 >= 0.0)
					{
						fT = 0.0;
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fT = 1.0;
						fSqrDist = fA11 + 2.0 * fB1 + fC;
					}
					else
					{
						fT = -fB1 / fA11;
						fSqrDist = fB1 * fT + fC;
					}
				}
			}
		}
	}
	else
	{
		// line segments are parallel
		if (fA01 > 0.0)
		{
			// direction vectors form an obtuse angle
			if (fB0 >= 0.0)
			{
				fS = 0.0;
				fT = 0.0;
				fSqrDist = fC;
			}
			else if (-fB0 <= fA00)
			{
				fS = -fB0 / fA00;
				fT = 0.0;
				fSqrDist = fB0 * fS + fC;
			}
			else
			{
				fB1 = -(kDiff | d1);
				fS = 1.0;
				fTmp = fA00 + fB0;
				if (-fTmp >= fA01)
				{
					fT = 1.0;
					fSqrDist = fA00 + fA11 + fC + 2.0 * (fA01 + fB0 + fB1);
				}
				else
				{
					fT = -fTmp / fA01;
					fSqrDist = fA00 + 2.0 * fB0 + fC + fT * (fA11 * fT + 2.0 * (fA01 + fB1));
				}
			}
		}
		else
		{
			// direction vectors form an acute angle
			if (-fB0 >= fA00)
			{
				fS = 1.0;
				fT = 0.0;
				fSqrDist = fA00 + 2.0 * fB0 + fC;
			}
			else if (fB0 <= 0.0)
			{
				fS = -fB0 / fA00;
				fT = 0.0;
				fSqrDist = fB0 * fS + fC;
			}
			else
			{
				fB1 = -(kDiff | d1);
				fS = 0.0;
				if (fB0 >= -fA01)
				{
					fT = 1.0;
					fSqrDist = fA11 + 2.0 * fB1 + fC;
				}
				else
				{
					fT = -fB0 / fA01;
					fSqrDist = fC + fT * (2.0 * fB1 + fA11 * fT);
				}
			}
		}
	}


	min_p0 = p00 + fS * d0;
	min_p1 = p10 + fT * d1;

	return fabs(fSqrDist);
}

double IsotropicRemeshing::distPointLine(Mesh::Point p, Mesh::Point p0, Mesh::Point p1) {
	return norm((p0 - p).cross(p1 - p)) / norm(p0 - p1);
}

double IsotropicRemeshing::min_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2) {
	Mesh::Point p01 = p1 - p0;
	Mesh::Point p02 = p2 - p0;
	Mesh::Point p12 = p2 - p1;
	double cos201 = p01.dot(p02) / (norm(p01) * norm(p02));
	double cos012 = -p01.dot(p12) / (norm(p01) * norm(p12));
	double cos120 = p02.dot(p12) / (norm(p02) * norm(p12));
	double min_cos = min(cos201, min(cos012, cos120));
	return min_cos;
}
double IsotropicRemeshing::min_cosAngle_triangle(Mesh::FaceHandle f) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(f);
	Mesh::VertexHandle v0 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle v1 = mesh.to_vertex_handle(heh);
	Mesh::VertexHandle v2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
	Mesh::Point p0 = mesh.point(v0);
	Mesh::Point p1 = mesh.point(v1);
	Mesh::Point p2 = mesh.point(v2);
	return min_cosAngle_triangle(p0, p1, p2);
}
double IsotropicRemeshing::max_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2) {
	Mesh::Point p01 = p1 - p0;
	Mesh::Point p02 = p2 - p0;
	Mesh::Point p12 = p2 - p1;
	double cos201 = p01.dot(p02) / (norm(p01) * norm(p02));
	double cos012 = -p01.dot(p12) / (norm(p01) * norm(p12));
	double cos120 = p02.dot(p12) / (norm(p02) * norm(p12));
	double max_cos = max(cos201, max(cos012, cos120));
	return max_cos;
}
double IsotropicRemeshing::max_cosAngle_triangle(Mesh::FaceHandle f) {
	Mesh::HalfedgeHandle heh = mesh.halfedge_handle(f);
	Mesh::VertexHandle v0 = mesh.from_vertex_handle(heh);
	Mesh::VertexHandle v1 = mesh.to_vertex_handle(heh);
	Mesh::VertexHandle v2 = mesh.to_vertex_handle(mesh.next_halfedge_handle(heh));
	Mesh::Point p0 = mesh.point(v0);
	Mesh::Point p1 = mesh.point(v1);
	Mesh::Point p2 = mesh.point(v2);
	return max_cosAngle_triangle(p0, p1, p2);
}
