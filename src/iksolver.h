#include "Eigen/Dense"

using namespace std;
using namespace Eigen;

class Segment {
    public:
        Segment *parent;
        Segment *child;
        Vector3f local_pi;
        Vector3f world_pi;
        Vector3f r_xyz;
        float length;

        Segment (void) {
            r_xyz = Vector3f(0.2,0.9,0.2);
            length = 0.4; 
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
        float length = 0.1;
        cout << "1" << endl;
        while(curr) { 
            curr->world_pi = Vector3f(length,0.0,5.0);
            length += 0.1;
            curr = curr->child;
        } 
        cout << "2" << endl;

        calc_new_pi();
        cout << "past calc pi" << endl;
    }
    Matrix4f get_jacobian(void);
    Matrix4f get_dr(Matrix4f jacobian, float step);
    void update_rotations(Matrix4f dr);
    Vector3f get_end_effector_world(void);
    Vector3f get_end_effector_local(void);
    float update_position(float epsilon, float step_size);
        
};



bool close_enough(Vector3f end_effector, Vector3f goal, float epsilon);
void initialize_goal(void);
void update_goal(int path_mode);
void render(void);
Matrix3f get_rodriguez(Vector3f);
void print_vec(Vector3f vec);



