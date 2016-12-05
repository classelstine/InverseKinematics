using namespace std;
#include <Eigen/Dense>
glm::vec3 goal_pos;


class Segment {
    Segment parent;
    Segment child;
    glm::vec3 in_joint;
    glm::vec3 out_joint;
    glm::vec3 in_theta;
    float length;

    Segment () {
        in_joint = glm::vec3(0,0,0): 
        out_joint = glm::vec3(0,0,0): 
        length = 1; 
    } 
    
}

class Arm {
    public:
        Segment root;
    
    //Default 4 segments
    Arm() { 
        root = Segment 
    }
        
}




bool close_enough(glm::vec3 p1, glm::vec3] p2, float epsilon);
void initialize_goal(void);
void update_goal(float path_mode);
void initialize_arm(void);
void render(void);
float update_position(float epsilon, float step_size);
glm::mat4 getJacobian(



