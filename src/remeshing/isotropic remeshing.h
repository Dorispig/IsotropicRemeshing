#pragma once
#include "src/MeshViewer/MeshDefinition.h"
#include "src/AABB_Tree/AABB_Tree.h"

const double PI = std::acos(-1);//pi=3.141592...
using namespace std;

///isotropic remeshing
///Towards robust adaptive isotropic remeshing
class IsotropicRemeshing {
public:
	OpenMesh::VPropHandleT<int> v_feature_edge_num;//Store the number of feature edges connected to point v
	OpenMesh::VPropHandleT<bool> v_plane_flag;
	OpenMesh::VPropHandleT<bool> v_plane_bound_flag;
	//OpenMesh::VPropHandleT<double> v_corner_flag;
	OpenMesh::VPropHandleT<double> v_target_length;
	OpenMesh::VPropHandleT<bool> v_is_boundary;

	OpenMesh::EPropHandleT<double> e_min_target_length;
	OpenMesh::EPropHandleT<double> e_max_target_length;
	OpenMesh::EPropHandleT<bool> is_crease;
	OpenMesh::EPropHandleT<bool> e_is_boundary;

	OpenMesh::FPropHandleT<bool> f_is_boundary;

	typedef struct Params {
		int iter = 3;

		double target_length_init;//Initial feature length, half of the average of all edge lengths of the input mesh
		double min_length = DBL_MAX;
		double max_length = DBL_MIN;
		double min_area = 1e-9;
		double target_length;//=target_length_init * target_length_ratio
		double target_length_ratio = 1.;//target_length = target_length_init * target_length_ratio
		double min_target_length, max_target_length;//The length of all edges should be between these two parameters
		double max_edge_length = 0;
		double pre_split_iter = 0;
		double smooth_ratio = 0.2;


		bool aabb_flag = false;
		bool split_flag = true; //Whether to perform split operation
		bool collapse_flag = true;//Whether to perform collapse operation
		bool flip_flag = true;//Whether to perform flip operation
		bool smooth_flag = true;//Whether to perform smooth operation
		bool project_flag = true;//Whether to perform project operation
		bool selected_only = false;//Whether to operate only on the selected part
		bool clean_flag = true;//improve poor quality grids
		bool feature_flag = true;
		bool adapt_flag = false;//Adaptive or not

		double angle_Threshold = 30.;//dihedral angle threshold,angular system
		double angle_Threshold_Rad;
		double cos_angle_Threshold;
		double min_angle = 30, max_angle = 120;
		double min_angle_Rad, max_angle_Rad;
		double cos_min_angle, cos_max_angle;
		double tolerance = 0.02;//Error threshold with respect to the input mesh


		void set_target_length() {
			min_target_length = 4. / 5. * target_length;
			max_target_length = 4. / 3. * target_length;
			min_area = max(min_area, min_length * min_length / 1000);
			std::cout << "min_area:::" << min_area << "\n";
		}

		void set_angle_Threshold() {
			angle_Threshold_Rad = angle_Threshold / 180. * PI;
			cos_angle_Threshold = cos(angle_Threshold_Rad);
			min_angle_Rad = min_angle / 180. * PI;
			cos_min_angle = cos(min_angle_Rad);
			max_angle_Rad = max_angle / 180. * PI;
			cos_max_angle = cos(max_angle_Rad);

		}
	};
	Params para;
private:
	Mesh mesh;
	AABB_Tree* abtree;//Storing the input mesh
	//std::vector<double> vertices_adaptive_length;
	
public:
	IsotropicRemeshing(Mesh mesh_) {
		mesh = mesh_;
		init_mesh_request();
		//generate_AABB_tree();
		init_para();
		//init_target_length();
		mesh.update_normals();
		
	}
	void init_mesh_request() {
		mesh.request_vertex_status();
		mesh.request_edge_status();
		mesh.request_halfedge_status();
		mesh.request_face_status();
		mesh.request_face_normals();
		mesh.request_halfedge_normals();
		mesh.request_vertex_normals();
	}
	void init_para() {
		para.target_length_init = calTargetLength() / 2;
		para.set_target_length();
		para.set_angle_Threshold();
	}

	void init_target_length() {
		for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
			mesh.property(v_target_length, *v_it) = para.target_length;
		}
		for (Mesh::EdgeIter e_it = mesh.edges_begin(); e_it != mesh.edges_end(); ++e_it) {
			Mesh::EdgeHandle eh = *e_it;
			mesh.property(e_min_target_length, eh) = para.min_target_length;
			mesh.property(e_max_target_length, eh) = para.max_target_length;
		}
	}

	void update_para() {
		para.target_length = para.target_length_init * para.target_length_ratio;
		para.set_target_length();
		para.pre_split_iter = log2(int(para.max_edge_length/para.target_length));
		para.set_angle_Threshold();
	}

	Mesh get_mesh() { return mesh; }
	AABB_Tree* get_AABB_tree() { return abtree; }

	void isotropic_remeshing();

	void bug_test();
	int cal_vertice_feature_edges(Mesh::VertexHandle vh);
	bool check_legal_feature_corner(Mesh::VertexHandle vh);
	bool check_legal_feature_corner(Mesh::VertexHandle vh,int num_feature_edges);
	void generate_AABB_tree();
	double calTargetLength();

	bool if_split_ok(Mesh::EdgeHandle eh, Mesh::VertexHandle vh);
	void split_edge(Mesh::EdgeHandle eh,Mesh::VertexHandle vh);
	void split_long_edges();
	void collapse_short_edges();
	void collapse_cross();
	void equalize_valences();
	void tangential_relaxation();
	void project_to_surface();

	void update_bound();

	void update_edge_feature(Mesh::EdgeHandle eh);
	void update_edges_feature();
	void update_vertice_feature(Mesh::VertexHandle vh);
	void update_vertices_feature();
	void update_feature();

	void update_crease();

	void update_target_length();
	void update_vertice_target_length(Mesh::VertexHandle vh);
	void update_vertices_target_length();
	void update_edge_target_length(Mesh::EdgeHandle eh);
	void update_edges_target_length();

	void clean_mesh(double quality);
	// 
	void removeColinearFaces();
	void repair_BadFaces(double quality);

	// Describe the quality of the triangle face,
	// return value range [0, 1] 
	// positive triangles for 1, linear is 0
	double quality(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);// =  8S^2/(abcp)  ---Taken from meshlab
	double quality_face(Mesh::FaceHandle fh);
	double quality_halfedge(Mesh::HalfedgeHandle heh);
	double quality_edge(Mesh::EdgeHandle heh);

	double calAdaptTargetlength(Mesh::VertexHandle vh);
	double calAdaptTargetlength(Mesh::HalfedgeHandle heh);
	double calAdaptTargetlength(Mesh::EdgeHandle eh);

	//calculate the H and K,return the Area_Voronoi_mixed
	double cal_curvature_And_Area_Voronoi(Mesh::VertexHandle vh, double& H, double& K);

	double cal_Voronoi_Area_mixed(Mesh::VertexHandle vh);
	//void update_vertice_target_length();
	//void update_vertices_target_length();
	Mesh::Point cal_face_Circumcenter(Mesh::FaceHandle fh);
	Mesh::Normal cal_3p_face_normal(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	Mesh::Normal cal_3p_face_normal(Mesh::FaceHandle fh);
	double cal_3p_face_2Area(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	double cal_3p_face_2Area(Mesh::FaceHandle fh);

	//check_angle:如果大于最大角度阈值，返回false
	bool check_angle_max(Mesh::FaceHandle fh);
	bool check_angle_max(Mesh::HalfedgeHandle heh);
	bool check_angle_max(Mesh::EdgeHandle eh);
	//check_angle:如果小于最小角度阈值，返回false
	bool check_angle_min(Mesh::FaceHandle fh);
	bool check_angle_min(Mesh::HalfedgeHandle heh);
	bool check_angle_min(Mesh::EdgeHandle eh);
	//check_angle:如果越过角度阈值，返回false
	bool check_angle(Mesh::FaceHandle fh);
	bool check_angle(Mesh::HalfedgeHandle heh);
	bool check_angle(Mesh::EdgeHandle eh);

	bool check_collapse_legal_feature(Mesh::HalfedgeHandle heh);//Legitimacy of features
	//Collapsing along one feature edge cannot affect another feature edge
	bool check_collapse_CanMove(Mesh::HalfedgeHandle heh);
	bool check_collapse_Quality(Mesh::HalfedgeHandle heh);
	bool check_relocate_Normal(Mesh::VertexHandle vh, Mesh::Point q);
	//Normal vectors are not excessively deflected after collapse.
	bool check_collapse_Normal(Mesh::HalfedgeHandle heh);
	bool check_collapse_Condition(Mesh::HalfedgeHandle heh);
	bool check_flip_Condition(Mesh::EdgeHandle eh);

	Mesh::HalfedgeHandle vertices_min_Out_Halfedge(Mesh::VertexHandle vh);
	Mesh::HalfedgeHandle vertices_max_Out_Halfedge(Mesh::VertexHandle vh);

// Calculate the two points with the shortest distance between two line segments
// return the nearest distance between line segment and line segment
	double distLineLine(Mesh::Point p00, Mesh::Point p01,Mesh::Point p10, Mesh::Point p11, Mesh::Point& min_p0, Mesh::Point& min_p1);
//
	double distPointLine(Mesh::Point p, Mesh::Point p0, Mesh::Point p1);

	double min_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	double min_cosAngle_triangle(Mesh::FaceHandle f);
	double max_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	double max_cosAngle_triangle(Mesh::FaceHandle f);
};
