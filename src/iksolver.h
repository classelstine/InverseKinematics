using namespace std;
using namespace Eigen;

#include <Eigen>

#define NULL nullptr; 

class Segment {
    Segment parent;
    Segment child;
    Vector3f in_joint;
    Vector3f out_joint;
    Vector3f in_theta;
    float length;

    Segment () {
        in_theta = Vector3f(0,0,0);
        length = 1; 
        parent = NULL;
        child = NULL;
    } 

    void set_child(Segment _child) { 
        this.child = _child;
        _child.parent = this; 
    } 
};

class Arm {
    public:
        Segment root;
        unsigned int num_segments; 
        float end_effector; 
    
    //Default 4 segments
    Arm() { 
        root = Segment();  
        child1 = Segment();  
        child2 = Segment();  
        child3 = Segment();  

        root.set_child(child1);
        child1.set_child(child2);
        child2.set_child(child3);

        calc_new_pi();
    }
    void calc_new_pi(void); 
    Matrix4f get_jacobian(void);
    Matrix4f get_dr(Matrix4f jacobian, float step);
    void update_rotations(Matrix4f dr);
        
};

bool close_enough(Vector3f end_effector, Vector3f goal, float epsilon);
void initialize_goal(void);
void update_goal(float path_mode);
void render(void);
float update_position(float epsilon, float step_size);



