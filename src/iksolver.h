#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

typedef struct Material {
    GLfloat ambient[4];
    GLfloat diffuse[4];
    GLfloat specular[4];
    GLfloat shininess[1];
} material; 

class Segment {
    public:
        Segment *parent;
        Segment *child;
        Vector3f local_pi;
        Vector3f world_pi;
        Vector3f r_xyz;
        float length;

        Segment (void) {
            r_xyz = Vector3f(0.0, 0.0, 0.0);
            length = 1;
            parent = NULL;
            child = NULL;
        } 

        void set_child(Segment *_child) { 
            this->child = _child;
            _child->parent = this; 
        } 
};

float random_float_in_range(float a, float b);

class Arm {
    public:
        Segment *root;
        Vector3f origin;
        int num_segments; 
    void calc_new_pi(void); 
    //Default 4 segments
    Arm(void);
    Arm(int num_segs);
    MatrixXf get_jacobian(void);
    VectorXf get_dr(MatrixXf, float);
    void update_rotations(VectorXf);
    Vector3f get_end_effector_world(void);
    Vector3f get_end_effector_local(void);
    float update_position(float, float);
    float get_total_length(void);
        
};



bool close_enough(Vector3f, Vector3f, float);
void initialize_goal(void);
void update_goal(int);
void render(void);
Matrix3f get_rodriguez(Vector3f);
void print_vec_3(Vector3f);
void print_vec_4(Vector4f);
bool render_sphere(Vector3f, float, float, float, int);
bool render_cylinder(float, Vector3f, Vector3f, float, float, float, float, float, int);
material get_material(void);
void set_material(int mat_idx);
Matrix4f get_xi(Matrix3f Ri, Vector3f li);
Vector3f non_homogenous(Vector4f);
Matrix3f cross_matrix(Vector3f);
void print_seg(Segment *curr_seg);
MatrixXf pseudo_inv(MatrixXf J);
float random_float_in_range(float a, float b);
void get_resolution();


