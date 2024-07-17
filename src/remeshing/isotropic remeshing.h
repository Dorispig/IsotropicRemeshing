#pragma once
#include "src/MeshViewer/MeshDefinition.h"
#include "src/AABB_Tree/AABB_Tree.h"

const double PI = std::acos(-1);//pi=3.141592...
using namespace std;

///isotropic remeshing
///Towards robust adaptive isotropic remeshing
class IsotropicRemeshing {
private:
	struct V_property {
		int feature_edge_num=0;
		bool feature = false;
		bool corner = false;
		bool boundary = false;
		double target_length;
		bool has_normal_ = false;
		Mesh::Normal normal;
		bool has_normal() { return has_normal_; }
	};
	OpenMesh::VPropHandleT<V_property> V_prop;

	struct H_property {
		bool feature = false;
		bool boundary = false;
		double target_length;
		double min_target_length;
		double max_target_length;
		bool has_normal_ = false;
		Mesh::Normal normal;
		bool has_normal() { return has_normal_; }
	};
	OpenMesh::HPropHandleT<H_property> H_prop;

	struct E_property {
		bool feature = false;
		bool boundary = false;
		double target_length;
		double min_target_length;
		double max_target_length;
		bool has_normal_ = false;
		Mesh::Normal normal;
		bool has_normal() { return has_normal_; }
	};
	OpenMesh::EPropHandleT<E_property> E_prop;

	struct F_property {
		bool boundary = false;
		bool has_normal_ = false;
		Mesh::Normal normal;
		bool has_normal() { return has_normal_; }
	};
	OpenMesh::FPropHandleT<F_property> F_prop;


	OpenMesh::VPropHandleT<int> v_feature_edge_num;//Store the number of feature edges connected to point v
	//OpenMesh::VPropHandleT<double> v_corner_flag;
	OpenMesh::VPropHandleT<double> v_target_length;
	OpenMesh::VPropHandleT<bool> v_is_boundary;

	OpenMesh::HPropHandleT<bool> he_is_boundary;

	OpenMesh::EPropHandleT<double> e_min_target_length;
	OpenMesh::EPropHandleT<double> e_max_target_length;
	//OpenMesh::EPropHandleT<bool> e_is_crease;
	OpenMesh::EPropHandleT<bool> e_is_boundary;

	OpenMesh::FPropHandleT<bool> f_is_boundary;
public:
	typedef struct Params {
		OpenMesh::PropertyT<V_property> v_prop;
		OpenMesh::PropertyT<H_property> he_prop;
		OpenMesh::PropertyT<E_property> e_prop;
		OpenMesh::PropertyT<F_property> f_prop;
		int splitNum=0;
		int collapseNum=0;
		int flipNum=0;

		int iter = 3;

		double target_length_init;//Initial feature length, half of the average of all edge lengths of the input mesh
		double min_length = DBL_MAX;
		double max_length = DBL_MIN;
		double min_area = 1e-9;
		double target_length;//=target_length_init * target_length_ratio
		double target_length_ratio = 1.;//target_length = target_length_init * target_length_ratio
		double min_target_length = DBL_MAX, max_target_length = DBL_MIN;//The length of all edges should be between these two parameters
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
		bool check_Surf_Dist = false;

		double angle_Threshold = 30.;//dihedral angle threshold,angular system
		double angle_Threshold_Rad;
		double cos_angle_Threshold;
		double min_angle = 30, max_angle = 120;
		double min_angle_Rad, max_angle_Rad;
		double cos_min_angle, cos_max_angle;
		double tolerance = 0.0001;//Error threshold with respect to the input mesh
		double max_SurfDist = 0.0001;
		double min_quality = 0.005;

		void set_target_length() {
			min_target_length = min(min_target_length, 4. / 5. * target_length);
			max_target_length = min(max_target_length, 4. / 3. * target_length);
			std::cout << "min_target_length参考值：" << 4. / 5. * target_length << "\n";
			std::cout << "max_target_length参考值：" << 4. / 3. * target_length << "\n";
			std::cout << "min_target_length实际值：" << min_target_length << "\n";
			std::cout << "max_target_length实际值：" << max_target_length << "\n";
			min_area = max(min_area, min_length * min_length / 1000);
			std::cout << "min_len:::" << min_length << "\n";
			std::cout << "max_len:::" << max_length << "\n";
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
		init_property();
		init_mesh_request();
		init_para();
		//init_target_length();
		mesh.update_normals();
		generate_AABB_tree();
		
	}
	void init_property() {
		mesh.add_property(V_prop);
		mesh.add_property(H_prop);
		mesh.add_property(E_prop);
		mesh.add_property(F_prop);
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
		para.max_SurfDist = para.tolerance;
		para.v_prop = mesh.property(V_prop);
		para.he_prop = mesh.property(H_prop);
		para.e_prop = mesh.property(E_prop);
		para.f_prop = mesh.property(F_prop);
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
	//TODO
	void update_face_normal(Mesh::FaceHandle fh);
	void update_faces_normal();
	void update_vertice_normal(Mesh::VertexHandle vh);
	void update_vertices_normal();
	void update_normals();

	bool is_feature_edge(Mesh::HalfedgeHandle heh, const double _feature_angle);//和openmesh的不一样（结果不一样）
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
	double quality(const Mesh::Point& p0, const Mesh::Point& p1, const Mesh::Point& p2);// =  8S^2/(abcp)  ---Taken from meshlab
	double quality_face(Mesh::FaceHandle fh);
	double quality_halfedge(Mesh::HalfedgeHandle heh);
	double quality_edge_min(Mesh::EdgeHandle heh);
	double quality_edge_max(Mesh::EdgeHandle heh);

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

	bool check_Hausdorff(vector<Mesh::Point> points,double max_SurfDist);
	bool check_collapse_ok(Mesh::HalfedgeHandle heh);
	bool check_collapse_legal_feature(Mesh::HalfedgeHandle heh);//Legitimacy of features
	//Collapsing along one feature edge cannot affect another feature edge
	bool check_collapse_CanMove(Mesh::HalfedgeHandle heh);
	bool check_collapse_Quality(Mesh::HalfedgeHandle heh);
	bool check_relocate_Normal(Mesh::VertexHandle vh, const Mesh::Point& q);
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
	double distPointLine(const Mesh::Point& p, const Mesh::Point& p0, const Mesh::Point& p1);

	double min_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	double min_cosAngle_triangle(Mesh::FaceHandle f);
	double max_cosAngle_triangle(Mesh::Point p0, Mesh::Point p1, Mesh::Point p2);
	double max_cosAngle_triangle(Mesh::FaceHandle f);

};
