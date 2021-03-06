#include <vector>
#include <iostream>
#include <exception>
#include <fstream>
#include <cmath>
#include <GL/glew.h> 
#include <vec3.hpp>
#include <glm.hpp>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream> 
#include <sstream>
#include <string> 
#include "iksolver.h"

#ifdef _WIN32
static DWORD lastTime;
#else
static struct timeval lastTime;
#endif

#define PI 3.14159265 // Should be used from mathlib

using namespace std;

/*
For UC Berkeley's CS184 Fall 2016 course, assignment 3 (Bezier surfaces)
*/

//****************************************************
// Global Variables
//****************************************************
GLfloat translation[3] = {0.0f, 0.0f, 0.0f};
GLfloat rotation[3] = {0.0f, 0.0f, 0.0f}; // Rotation of angle, axis_x, axis_y, axis_z
bool auto_strech = false;
bool goal_in_reach = true;
int Width_global = 1000;
int Height_global = 1000;
int Z_buffer_bit_depth = 128;
float step_size;
bool wireframe_mode = false;
float epsilon;
float zoom = 1;
float box_size;
float x_min;
float x_max;
float y_min;
float y_max;
material *material_list;

Arm *arm;
Vector3f goal_position;

inline float sqr(float x) { return x*x; }


bool close_enough(Vector3f end, Vector3f goal, float epsilon) {
    Vector3f diff = goal - end;
    float dist = diff.norm();
    bool is_close =  (dist < epsilon);
    return is_close;
}

float random_float_in_range(float a, float b) {
        float random = ((float) rand()) / (float) RAND_MAX;
        float diff = b - a;
        float r = random * diff;
        return a + r;
}

//****************************************************
// Simple init function
//****************************************************
void initializeRendering()
{
    glfwInit();
}

void initialize_goal(void) {
    goal_position = Vector3f(1.0, 1.0, 1.0);
}

void update_goal(int mode) {
    switch (mode) {
        case 0: break;
    }
}

void get_resolution() {
    const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());

    Width_global = mode->width;
    Height_global = mode->height;
}
Arm::Arm(void) { 
    new (this) Arm(4);
} 

Arm::Arm(int num_segs) { 
        this->num_segments = num_segs;
        this->origin = Vector3f(0.0,0.0,10.0); 
        this->root = new Segment();  
        Segment *curr = root;
        float length;
        for (int i = 0; i < num_segs-1; i++) { 
            curr->world_pi = Vector3f(length,0.0,5.0);
            length = random_float_in_range(1.0, 5.0);
            curr->length = length;
            Segment *child = new Segment();
            curr->set_child(child);
            curr = curr->child;
        } 
        //last one 
        curr->world_pi = Vector3f(length,0.0,5.0);
        length = random_float_in_range(1.0, 5.0);
        curr->length = length;
        this->calc_new_pi();
}
//****************************************************
// A routine that updates the position of the end effector and 
// returns how far it is away from the goal position. 
//****************************************************
float Arm::update_position(float epsilon, float step_size) {
    Vector3f end_effector = arm->get_end_effector_world();
    float cur_error = (end_effector - goal_position).norm();
    if(close_enough(end_effector, goal_position, epsilon)) { 
        return cur_error; 
    } 
    MatrixXf jacobian(3, 3*this->num_segments);
    jacobian = arm->get_jacobian(); 
    VectorXf dr = arm->get_dr(jacobian, step_size);
    arm->update_rotations(dr);
    arm->calc_new_pi();
    end_effector = arm->get_end_effector_world();
    
    float after_rotation_error = (end_effector - goal_position).norm();
    if (after_rotation_error > cur_error) {
        //arm->update_rotations(-1 * dr);
        //arm->calc_new_pi();
    } else {
        cur_error = after_rotation_error;
    }
    cur_error = after_rotation_error;
    return cur_error;
} 

MatrixXf Arm::get_jacobian(void) {
    Matrix3f *rotation_list = new Matrix3f[num_segments];
    Matrix3f compound_rotation = Matrix3f::Identity();
    int seg_idx = 0;
    //get rotations
    Segment *curr_seg = this->root;
    while(true) { 
        Vector3f rot_vec;
        if (!curr_seg->parent) { 
            rot_vec = Vector3f(0,0,0);
        }
        else {
            rot_vec = curr_seg->parent->r_xyz;
        } 
        Matrix3f ri = get_rodriguez(rot_vec);
        compound_rotation = compound_rotation*ri;
        rotation_list[seg_idx] = compound_rotation;
        if (!curr_seg->child) { 
            break;
        } 
        curr_seg = curr_seg->child;
        seg_idx++;
    }
    //get transformations
    Matrix4f *transformation_list = new Matrix4f[num_segments];
    Matrix4f compound_transformation = Matrix4f::Identity();
    while(true) {
        if (!curr_seg->child) { 
            transformation_list[seg_idx] = compound_transformation;
        } 
        else { 
            Vector3f rot_vec = curr_seg->r_xyz;
            Matrix3f ri = get_rodriguez(rot_vec);
            Vector3f pi = curr_seg->local_pi;
            Matrix4f xi = get_xi(ri, pi);
            compound_transformation = xi*compound_transformation;
            transformation_list[seg_idx] = compound_transformation;
        } 
        if (!curr_seg->parent) { 
            break;
        } 
        curr_seg = curr_seg->parent;
        seg_idx--;
    }
    //calculate Ji's 
    MatrixXf J(3, 3*num_segments);
    Vector4f pn;
    pn <<  this->get_end_effector_local(), 1;
    while(true) { 
        Matrix4f xi = transformation_list[seg_idx];
        Vector4f x_pn_4 = transformation_list[seg_idx]*pn;
        Vector3f x_pn_3 = non_homogenous(x_pn_4);
        Matrix3f x_pn_cross = cross_matrix(x_pn_3);
        Matrix3f ri = rotation_list[seg_idx];
        Matrix3f Ji = -1*rotation_list[seg_idx]*x_pn_cross; 
        J.block<3,3>(0,3*seg_idx) = Ji;
        if (!curr_seg->child) { 
            break;
        } 
        curr_seg = curr_seg->child;
        seg_idx++;
    }
    return J;
}

Vector3f non_homogenous(Vector4f h) { 
    Vector3f non_h = Vector3f(h[0]/h[3], h[1]/h[3], h[2]/h[3]);
    return non_h;
} 

Matrix4f get_xi(Matrix3f Ri, Vector3f li) { 
    MatrixXf top(3, 4);
    top << Ri, li;
    Matrix4f xi;
    Vector4f bottom = {0,0,0,1};
    xi.block<3,4>(0,0) = top;
    xi.block<1,4>(3,0) = bottom;
    return xi;
} 

Matrix3f get_rodriguez(Vector3f rot_axis) { 
    Matrix3f rx;
    Matrix3f rodriguez;
    float theta = rot_axis.norm(); 
    if (theta == 0) { 
        rodriguez = Matrix3f::Identity();
        return rodriguez; 
    } 
    rot_axis.normalize();
    //print_vec_3(rot_axis);

    Matrix3f I = Matrix3f::Identity();
    rx = cross_matrix(rot_axis);
    rodriguez = rx*sin(theta) + I + rx*rx*(1-cos(theta));
    return rodriguez;
} 

Matrix3f cross_matrix(Vector3f rot_axis) { 
    Matrix3f rx;
    rx << 0, -1*rot_axis[2], rot_axis[1],
          rot_axis[2], 0, -1*rot_axis[0], 
          -1*rot_axis[1], rot_axis[0], 0;
    return rx;
}
MatrixXf pseudo_inv(MatrixXf J) {
    MatrixXf J_plus(3*arm->num_segments, 3);
    JacobiSVD<MatrixXf> svd( J, ComputeThinU | ComputeThinV);
    double tolerance = 0.000001 * std::max(J.cols(), J.rows()) *svd.singularValues().array().abs()(0);
    J_plus = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
    return J_plus;
}

VectorXf Arm::get_dr(MatrixXf jacobian, float step) {
    MatrixXf J_plus(3*this->num_segments, 3);
    J_plus = pseudo_inv(jacobian);
    Vector3f diff = goal_position - this->get_end_effector_world();
    Vector3f cur_step = step * diff;
    VectorXf dr(3*this->num_segments);
    dr = J_plus * cur_step;
    return dr;
}

void Arm::calc_new_pi(void) {
    Matrix3f compound_rotation = Matrix3f::Identity();
    Segment *curr_seg = this->root;
    while(curr_seg) { 
        Vector3f rot_vec = curr_seg->r_xyz;
        Matrix3f ri = get_rodriguez(rot_vec);
        compound_rotation = compound_rotation*ri;
        Vector3f li = Vector3f(curr_seg->length, 0, 0);  
        curr_seg->local_pi = ri*li; 
        if (!curr_seg->parent) { 
            curr_seg->world_pi = compound_rotation*li + this->origin; 
        }
        else { 
            curr_seg->world_pi = compound_rotation*li + curr_seg->parent->world_pi;
        } 
        curr_seg = curr_seg->child;
    } 
}

void Arm::update_rotations(VectorXf dr) {
    int seg_idx = 0;
    Segment *cur_seg = this->root;
    while(cur_seg) {
        int id = seg_idx * 3;
        Vector3f new_xyz = Vector3f(dr[id], dr[id+1], dr[id+2]);
        cur_seg->r_xyz = new_xyz + cur_seg->r_xyz;
        //cout << "updated rotation :" << seg_idx << endl;
        //cout << cur_seg->r_xyz << endl;
        seg_idx++;
        cur_seg = cur_seg->child;
    }
    return;
} 

Vector3f Arm::get_end_effector_local(void){
    //cout << "in get_local" << endl;
    Segment *curr = this->root;
    while(curr->child) { 
        curr = curr->child;
    } 
    Vector3f end = curr->local_pi;
    //cout << "finished get_local" << endl;
    return end;
}

Vector3f Arm::get_end_effector_world(void){
    //cout << "in get_world" << endl;
    Segment *curr = this->root;
    while(curr->child) { 
        curr = curr->child;
    } 
    Vector3f end = curr->world_pi;
    //cout << "finished get_world" << endl;
    return end;
}

float Arm::get_total_length(void) {
    float length = 0;
    cout << "1" << endl;
    Segment *curr = this->root;
    cout << "2" << endl;
    while(curr) { 
        cout << "3" << endl;
        curr->world_pi = Vector3f(length,0.0,0.0);
        cout << "4" << endl;
        length = curr->length + length;
        cout << "5" << endl;
        curr = curr->child;
        cout << "6" << endl;
    } 
    return length;
}

//****************************************************
// Keyboard inputs. Add things to match the spec! 
//****************************************************
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    switch (key) {
            
        case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
        case GLFW_KEY_Q: glfwSetWindowShouldClose(window, GLFW_TRUE); break;
        case GLFW_KEY_LEFT :
            if (action) {
                if (mods == GLFW_MOD_SHIFT) {
                    goal_position[0] -= 1.0f;
                    float dist = (arm->origin - goal_position).norm();
                    float arm_length = arm->get_total_length();
                    if (dist > arm_length) { 
                        goal_in_reach = false;
                    } 
                    else { 
                        goal_in_reach = true;
                    } 
                } else {
                    rotation[0] += 5.0f;
                }
            }
            break;
        case GLFW_KEY_RIGHT:
            if (action) {
                if (mods == GLFW_MOD_SHIFT){
                    goal_position[0] += 1.0f;
                    float dist = (arm->origin - goal_position).norm();
                    float arm_length = arm->get_total_length();
                    if (dist > arm_length) { 
                        goal_in_reach = false;
                    } 
                    else { 
                        goal_in_reach = true;
                    } 
                } else {
                    rotation[0] -= 5.0f;
                }
            }
            break;
        case GLFW_KEY_UP   :
            if (action) {
                if (mods == GLFW_MOD_SHIFT) {
                    goal_position[1] += 1.0f;
                    float dist = (arm->origin - goal_position).norm();
                    float arm_length = arm->get_total_length();
                    if (dist > arm_length) { 
                        goal_in_reach = false;
                    } 
                    else { 
                        goal_in_reach = true;
                    } 
                } else {
                    rotation[1] += 5.0f;
                }
            }
            break;
        case GLFW_KEY_DOWN :
            if (action) {
                if (mods == GLFW_MOD_SHIFT) {
                    goal_position[1] -= 1.0f;
                    float dist = (arm->origin - goal_position).norm();
                    float arm_length = arm->get_total_length();
                    if (dist > arm_length) { 
                        goal_in_reach = false;
                    } 
                    else { 
                        goal_in_reach = true;
                    } 
                } else {
                    rotation[2] += 5.0f;
                }
            }
            break;
        case GLFW_KEY_EQUAL: 
            if (action) translation[2] += 0.1f;
            break;
        case GLFW_KEY_MINUS :
            if (action) translation[2] -= 0.1f;
            break;
        case GLFW_KEY_SPACE : break;
        default: break;
    }
    
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods)
{
        if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
            get_resolution();
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            /*
            cout << "MOUSE CLICK" << endl;
            cout << xpos << " " << ypos << endl;
            */
            //xpos and ypos is the position on the screen

            //The following is projecting them onto the screen
        
            GLint viewport[4]; //var to hold the viewport info
            GLdouble modelview[16]; //var to hold the modelview info
            GLdouble projection[16]; //var to hold the projection matrix info
            GLfloat win_x, win_y, win_z; //variables to hold screen x,y,z coordinates
            GLdouble world_x, world_y, world_z; //variables to hold world x,y,z coordinates

            glGetDoublev( GL_MODELVIEW_MATRIX, modelview ); //get the modelview info
            glGetDoublev( GL_PROJECTION_MATRIX, projection ); //get the projection matrix info
            glGetIntegerv( GL_VIEWPORT, viewport ); //get the viewport info

            win_x = (float)xpos;
            win_y = (float)viewport[3] - (float)ypos;
            win_z = 0;

            //get the world coordinates from the screen coordinates
            gluUnProject( win_x, win_y, win_z, modelview, projection, viewport, &world_x, &world_y, &world_z);
            //cout << "VIEWPORT: " << viewport[0] << " "  << viewport[1] << " " <<  viewport[2] << " " <<  viewport[3]  << endl;
            
            //Now update information we care about
            
            float origin_x = Width_global/2.0;
            float origin_y = Height_global/2.0;
            float x_ratio = (2 * x_max)/ Width_global;
            float y_ratio = (2 * y_max)/ Height_global;
            float length = arm->get_total_length();
            float x = (xpos - origin_x) * x_ratio;
            float y = -1 * (ypos - origin_y) * y_ratio;
            /*
            cout << "origin x, y: " << origin_x << " " << origin_y << endl;
            cout << "x and y ratio: " << x_ratio << " " << y_ratio << endl;
            cout << "x y min and max: " << x_min << " " << y_min << " " << x_max << " " << x_max << endl;
            cout << "final x, y: " << x << " " << y << endl;
            */
       
            float z = random_float_in_range((-1*length)/2, length/2) + arm->origin[2]; 
            goal_position[0] = x;
            goal_position[1] = y;
            goal_position[2] = z;
            //cout << "goal" << endl;
            print_vec_3(goal_position);
            //cout << "distance from origin to goal" << endl;
            float dist = (arm->origin - goal_position).norm();
            //cout << dist << endl;
            //cout << "length of arm" << endl;
            float arm_length = arm->get_total_length();
            //cout << arm_length << endl;
            if (dist > arm_length) { 
                goal_in_reach = false;
            } 
            else { 
                goal_in_reach = true;
            } 
            cout << arm_length << endl;
        } 
}
// ***********************
//    RENDER FUNCTION
// Given current global Arm, 
// render it to the display
// ***********************
void render(void) { 
    //now we need to render arm
    //Render Origin
    float sphere_radius = 0.6;
    float slices = 32;
    float stacks = 32;
    float base = 0.3;
    float top = 0.3;
    int mat_idx = 0;
    // render the goal
    render_sphere(goal_position, sphere_radius * 0.5, slices, stacks, 0);
    Vector3f sphere_center = arm->origin;
    render_sphere(sphere_center, sphere_radius, slices, stacks, mat_idx);
    mat_idx++;
    GLUquadric *quad;
    
    //Render Rest of Arm
    //Need to add cylinders
    Segment *curr_segment = arm->root;
    while (curr_segment) { 
        //render joint
        sphere_center = curr_segment->world_pi;
        render_sphere(sphere_center, sphere_radius, slices, stacks, mat_idx);
        mat_idx++;
        //cout << "sphere at: " << x << " " << y << " " << z << endl;
        //render cylinder
        Vector3f cyl_base;
        if(curr_segment->parent) {
            cyl_base = curr_segment->parent->world_pi;
        } else {
            cyl_base = arm->origin;
        }
        Vector3f r = curr_segment->world_pi - cyl_base;
        if (r[2] == 0) {
            r[2] = 0.001;
        }
        float angle = 57.2957795*acos( r[2]/r.norm());
        if (r[2] < 0.0) {
            angle = -1 * angle;
        }
        float rot_x = -r[1]*r[2];
        float rot_y = r[0]*r[2];
        Vector3f rot_cyl_axis = Vector3f(rot_x, rot_y, 0.0);
        render_cylinder(angle, rot_cyl_axis, cyl_base, r.norm(), base, top, slices, stacks, mat_idx);
        mat_idx++;
        curr_segment = curr_segment->child;
    } 
}
// ***********************
//    RENDER SPHERE
// render a sphere, which we
// are using to represent joints
// ***********************
bool render_sphere(Vector3f center, float radius, float slices, float stacks, int mat_idx) { 
    glPushMatrix();
    GLUquadric *quad;
    set_material(mat_idx);
    quad = gluNewQuadric();
    if( !quad) {
            cout << "quad didn't initialize" << endl;
            return false;
    }
    gluQuadricNormals(quad, GLU_SMOOTH);
    gluQuadricTexture(quad, GL_TRUE); 
    glTranslatef(center[0], center[1], center[2]);
    gluSphere(quad, radius, slices, stacks);
    glPopMatrix();
    return true;
} 
        
// ***********************
//    RENDER CYLINDER
// render a sphere, which we
// are using to represent joints
// ***********************
bool render_cylinder(float angle, Vector3f rot_axis, Vector3f cyl_loc, float height, float base, float top, float slices, float stacks, int mat_idx) { 
    glPushMatrix();
    set_material(mat_idx);
    GLUquadric *quad;
    quad = gluNewQuadric();
    if( !quad) {
            cout << "quad didn't initialize" << endl;
            return false;
    }
    gluQuadricNormals(quad, GLU_SMOOTH);
    gluQuadricTexture(quad, GL_TRUE); 
    glTranslatef(cyl_loc[0], cyl_loc[1], cyl_loc[2]); 
    glRotatef(angle, rot_axis[0], rot_axis[1], rot_axis[2]);
    gluCylinder(quad, base, top, height, slices, stacks);
    glPopMatrix();
    return true;
}

material get_material(void) {
    material new_material = {};
    for (int i = 0; i < 4; i++) { 
        new_material.ambient[i] = (float) rand()/(RAND_MAX);
        new_material.diffuse[i] = (float) rand()/(RAND_MAX);
        new_material.specular[i] = (float) rand()/(RAND_MAX);
        if (i == 0) 
            new_material.shininess[i] = (float) rand()/(RAND_MAX)*128;
    } 
    return new_material;
} 

void set_material(int mat_idx) { 
    material cyl_material = material_list[mat_idx];
    glMaterialfv(GL_FRONT, GL_DIFFUSE, cyl_material.diffuse); 
    glMaterialfv(GL_FRONT, GL_AMBIENT, cyl_material.ambient); 
    glMaterialfv(GL_FRONT, GL_SPECULAR, cyl_material.specular); 
    glMaterialfv(GL_FRONT, GL_SHININESS, cyl_material.shininess); 
} 
//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void display( GLFWwindow* window )
{
    //glOrtho(0*zoom, Width_global*zoom, 0*zoom, Height_global*zoom, 1, -1);
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //clear background screen to black
    GLfloat light_pos[] = {1.0, 2.0, -3.0, 1.0}; 
    GLfloat ambient_light[] = {0.4, 0.4, 0.4};
    GLfloat diffuse_light[] = {1.0, 1.0, 1.0};
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glShadeModel(GL_SMOOTH);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                // clear the color buffer (sets everything to black)
    glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
    glLoadIdentity();                            // make sure transformation is "zero'd"
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuse_light);
    glLightfv(GL_LIGHT1, GL_AMBIENT, ambient_light);
    
    //----------------------- code to draw objects --------------------------
    
    //glOrtho(0*zoom, Width_global*zoom, 0*zoom, Height_global*zoom, 1, -1);
    
    render();
    glfwSwapBuffers(window);


}

//****************************************************
// function that is called when window is resized
//***************************************************
void size_callback(GLFWwindow* window, int width, int height)
{
    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
    
    glViewport(0, 0, Width_global, Height_global);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, Width_global, 0, Height_global, 1, -1);
    
    display(window);
}

void print_vec_3(Vector3f vec) { 
    cout << vec[0] << " " << vec[1] << " " << vec[2] << endl;
} 

void print_vec_4(Vector4f vec) { 
    cout << vec[0] << " " << vec[1] << " " << vec[2] << " " << vec[3] << endl;
}

void print_seg(Segment *curr_seg) { 
    cout << "CURR SEG INFO" << endl;
    cout << "world pi" << endl;
    print_vec_3(curr_seg->world_pi);
    cout << "local pi" << endl;
    print_vec_3(curr_seg->local_pi);
    cout << "rot axis" << endl;
    print_vec_3(curr_seg->r_xyz);
    cout << "length" << endl;
    cout << curr_seg->length << endl;
    cout << "DONE WITH SEG INFO" << endl;
} 

//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
    //This initializes glfw
    initializeRendering();
    get_resolution();
    GLFWwindow* window = glfwCreateWindow( Width_global, Height_global, "CS184", NULL, NULL );
    if ( !window )
    {
        cerr << "Error on window creating" << endl;
        glfwTerminate();
        return -1;
    }

    const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if ( !mode )
    {
        cerr << "Error on getting monitor" << endl;
        glfwTerminate();
        return -1;
    }
    if (argc == 1) {
        arm = new Arm();
        epsilon = 0.05f;
        step_size = 0.1f;
    } 
    else if (argc == 2) { 
        arm = new Arm(atoi(argv[1]));
        epsilon = 0.05f;
        step_size = 0.1f;
    } 
    else if (argc == 3) {  
        arm = new Arm(atoi(argv[1]));
        step_size = atof(argv[2]);
        epsilon = 0.05f;
    } 
    float box_size = arm->get_total_length()*1.2;
    
    glfwMakeContextCurrent( window );
    
    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glOrtho(-3.5, 3.5, -3.5, 3.5, 5, -5);
    cout << "width global pre ortho initialize " << Width_global << endl;
    cout << "height global pre ortho initialize " << Height_global << endl;
    float width_ratio = (float) Width_global/((float) Width_global + (float) Height_global);
    float height_ratio = (float) Height_global/((float) Width_global + (float) Height_global);
    x_max = width_ratio*box_size;
    x_min = -1*box_size*width_ratio;
    y_max = height_ratio*box_size;
    y_min = -1*height_ratio*box_size;
    cout << "x_max " << x_max << endl;
    cout << "x_min " << x_min << endl;
    cout << "y_max " << y_max << endl;
    cout << "y_min " << y_min << endl;
    glOrtho(x_min, x_max, y_min, y_max, 2*box_size, -2*box_size);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glEnable(GL_DEPTH_TEST);	// enable z-buffering
    glDepthFunc(GL_LESS);

    glfwSetWindowTitle(window, "CS184");
    glfwSetWindowSizeCallback(window, size_callback);
    glfwSetKeyCallback(window, key_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);

    
    int path_mode = 0;
    initialize_goal();
    /*
    MatrixXf J = arm->get_jacobian();
    cout << "J+" << endl;
    cout << pseudo_inv(J) << endl;
    
    cout << "DR" << endl;
    cout << arm->get_dr(J, 0.001) << endl;
    
    
    arm->get_jacobian();
    */
    int segs_and_joints = arm->num_segments*2 + 1;
    material_list = new material[segs_and_joints];  
    for (int i = 0; i < segs_and_joints; i++) { 
        material_list[i] = get_material();
    } 
     
    cout << "3" << endl;


    while( !glfwWindowShouldClose( window ) ) // infinite loop to draw object again and again
    {   // because once object is draw then window is terminated
        display( window );
        if (goal_in_reach) { 
            arm->update_position(epsilon, step_size);
        } 
        
        if (auto_strech){
            glfwSetWindowSize(window, mode->width, mode->height);
            glfwSetWindowPos(window, 0, 0);
        }
        glfwPollEvents();
        
    }

    return 0;
}
