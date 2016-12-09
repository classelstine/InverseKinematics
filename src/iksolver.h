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
            r_xyz = Vector3f(1.0, 0.5, 0.0);
            length = 1;
            parent = NULL;
            child = NULL;
        } 

        void set_child(Segment *_child) { 
            this->child = _child;
            _child->parent = this; 
        } 
};

class Arm {
    public:
        Segment *root;
        Vector3f origin;
        unsigned int num_segments; 
    void calc_new_pi(void); 
    //Default 4 segments
    Arm() { 
        root = new Segment();  
        origin = Vector3f(0.0,0.0,5.0); 
        Segment *child1 = new Segment();  
        Segment *child2 = new Segment();  
        Segment *child3 = new Segment();  

        root->set_child(child1);
        child1->set_child(child2);
        child2->set_child(child3);
        Segment *curr = root;
        float length = 1;
        while(curr) { 
            curr->world_pi = Vector3f(length,0.0,5.0);
            length += 1;
            curr = curr->child;
        } 

        calc_new_pi();
    }
    Matrix4f get_jacobian(void);
    Matrix4f get_dr(Matrix4f, float);
    void update_rotations(Matrix4f);
    Vector3f get_end_effector_world(void);
    Vector3f get_end_effector_local(void);
    float update_position(float, float);
        
};



bool close_enough(Vector3f, Vector3f, float);
void initialize_goal(void);
void update_goal(int);
void render(void);
Matrix3f get_rodriguez(Vector3f);
void print_vec(Vector3f);
bool render_sphere(Vector3f, float, float, float, int);
bool render_cylinder(float, Vector3f, Vector3f, float, float, float, float, float, int); 
material get_material(void);
void set_material(int mat_idx);


