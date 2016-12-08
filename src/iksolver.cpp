#include <vector>
#include <iostream>
#include <exception>
#include <fstream>
#include <cmath>
#include <GL/glew.h> 
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
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
int Width_global = 400;
int Height_global = 400;
int Z_buffer_bit_depth = 128;
float step_size;
bool wireframe_mode = false;
float epsilon;
float zoom = 1;

Arm *arm;
Vector3f goal_position;

inline float sqr(float x) { return x*x; }


bool close_enough(Vector3f end, Vector3f goal, float epsilon) {
    Vector3f diff = goal - end;
    float dist = diff.norm();
    bool is_close =  (dist < epsilon);
    return is_close;
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

//****************************************************
// A routine that updates the position of the end effector and 
// returns how far it is away from the goal position. 
//****************************************************
float Arm::update_position(float epsilon, float step_size) {
    Vector3f end_effector = arm->get_end_effector_world();
    float error;
    if(close_enough(end_effector, goal_position, epsilon)) { 
        error = (end_effector - goal_position).norm();
        return error; 
    } 
    
    Matrix4f jacobian = arm->get_jacobian(); 
    Matrix4f dr = arm->get_dr(jacobian, step_size);
    arm->update_rotations(dr);
    arm->calc_new_pi();
    end_effector = arm->get_end_effector_world();
    error = (end_effector - goal_position).norm();
    return error;
} 

Matrix4f Arm::get_jacobian(void) {
    Matrix4f J;
    J << 1, 0, 0, 0,
      0, 1, 0, 0, 
      0, 0, 1, 0,
      0, 0, 0, 1;
    return J;
}


Matrix3f get_rodriguez(Vector3f rot_axis) { 
    Matrix3f rx;
    Matrix3f rodriguez;
    float theta = rot_axis.norm(); 
    if (theta == 0) { 
        rodriguez = Matrix3f::Identity();
        return rodriguez; 
    } 
    cout << "theta: " << theta << endl;
    rot_axis.normalize();
    cout << "post normalize" << endl;
    print_vec(rot_axis);

    Matrix3f I = Matrix3f::Identity();
    rx << 0, -1*rot_axis[2], rot_axis[1],
          rot_axis[2], 0, -1*rot_axis[0], 
          -1*rot_axis[1], rot_axis[0], 0;
   rodriguez = rx*sin(theta) + I + rx*rx*(1-cos(theta));
   return rodriguez;
} 

Matrix4f Arm::get_dr(Matrix4f jacobian, float step) {
    Matrix4f dr;
    dr << 1, 0, 0, 0,
      0, 1, 0, 0, 
      0, 0, 1, 0,
      0, 0, 0, 1;
     
    return dr;
}

void Arm::calc_new_pi(void) {
    cout << "in calc pi" << endl;
    Matrix3f compound_rotation = Matrix3f::Identity();
    cout << "1" << endl;
    Segment *curr_seg = this->root;
    cout << "2" << endl;
    while(curr_seg) { 
        cout << "1" << endl;
        Vector3f rot_vec = curr_seg->r_xyz;
        cout << "2" << endl;
        Matrix3f ri = get_rodriguez(rot_vec);
        cout << "3" << endl;
        compound_rotation = compound_rotation*ri;
        cout << "4" << endl;
        Vector3f li = Vector3f(curr_seg->length, 0, 0);  
        cout << "5" << endl;
        curr_seg->local_pi = ri*li; 
        cout << "6" << endl;
        if (!curr_seg->parent) { 
            cout << "7" << endl;
            curr_seg->world_pi = compound_rotation*li + this->origin; 
            cout << "8" << endl;
        }
        else { 
            cout << "9" << endl;
            curr_seg->world_pi = compound_rotation*li + curr_seg->parent->world_pi;
            cout << "10" << endl;
        } 
        cout << "12" << endl;
        curr_seg = curr_seg->child;
        cout << "11" << endl;
    } 
    cout << "13" << endl;
}

void Arm::update_rotations(Matrix4f dr) { 
    return;
} 

Vector3f Arm::get_end_effector_local(void){
    Vector3f end = root->child->child->child->local_pi;
    return end;
}

Vector3f Arm::get_end_effector_world(void){
    Vector3f end = root->child->child->child->local_pi;
    return end;
}


//****************************************************
// A routine to set a pixel by drawing a GL point.  This is not a
// general purpose routine as it assumes a lot of stuff specific to
// this example.
//****************************************************

void setPixel(float x, float y, GLfloat r, GLfloat g, GLfloat b) {
    glColor3f(r, g, b);
    glVertex2f(x+0.5, y+0.5);  // The 0.5 is to target pixel centers
    // Note: Need to check for gap bug on inst machines.
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
                    translation[0] -= 0.001f * Width_global;
                } else {
                    rotation[0] += 5.0f;
                }
            }
            break;
        case GLFW_KEY_RIGHT:
            if (action) {
                if (mods == GLFW_MOD_SHIFT){
                    translation[0] += 0.001f * Width_global;
                } else {
                    rotation[0] -= 5.0f;
                }
            }
            break;
        case GLFW_KEY_UP   :
            if (action) {
                if (mods == GLFW_MOD_SHIFT) {
                    translation[1] += 0.001f * Height_global;
                } else {
                    rotation[1] += 5.0f;
                }
            }
            break;
        case GLFW_KEY_DOWN :
            if (action) {
                if (mods == GLFW_MOD_SHIFT) {
                    translation[1] -= 0.001f * Height_global;
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
// ***********************
//    RENDER FUNCTION
// Given current global Arm, 
// render it to the display
// ***********************
void render(void) {
    /*
    //how to render circle 
    glColor3f(0,0,1);
    GLUquadric *quad;
    quad = gluNewQuadric();
    if( !quad) {
            cout << "quad didn't initialize" << endl;
    }
    gluQuadricNormals(quad, GLU_SMOOTH);
    gluQuadricTexture(quad, GL_TRUE); 
    glTranslatef(0,0,4);
    gluSphere(quad,1.0,18,10);
    */
    /*
    //how to render cylinder 
    glColor3f(0,1,1);
    GLUquadric *quad;
    quad = gluNewQuadric();
    if( !quad) {
            cout << "didn't work" << endl;
    }
    gluQuadricNormals(quad, GLU_SMOOTH);
    gluQuadricTexture(quad, GL_TRUE); 
    glTranslatef(0,2,4);
    glRotatef(90.0,0.0,0.0,0.0);
    gluCylinder(quad, 1.0, 1.0, 3, 32, 32);
    */
    //now we need to render arm
    //Render Origin
    cout << "1" << endl;
    glPushMatrix();
    GLUquadric *quad;
    quad = gluNewQuadric();
    if( !quad) {
            cout << "quad didn't initialize" << endl;
    }
    gluQuadricNormals(quad, GLU_FLAT);
    gluQuadricTexture(quad, GL_TRUE); 
    float x = arm->origin[0];
    float y = arm->origin[1];
    float z = arm->origin[2];
    cout << "sphere at: " << x << " " << y << " " << z << endl;
    glTranslatef(x, y, z);
    gluSphere(quad,0.1,18,10);
    glPopMatrix();
    
    //Render Rest of Arm
    //Need to add cylinders
    Segment *curr_segment = arm->root;
    while (curr_segment) { 
        glPushMatrix();
        quad = gluNewQuadric();
        if( !quad) {
                cout << "quad didn't initialize" << endl;
        }
        gluQuadricNormals(quad, GLU_SMOOTH);
        gluQuadricTexture(quad, GL_TRUE); 
        x = curr_segment->world_pi[0];
        y = curr_segment->world_pi[1];
        z = curr_segment->world_pi[2];
        cout << "sphere at: " << x << " " << y << " " << z << endl;
        glTranslatef(x, y, z);
        gluSphere(quad,0.1,18,10);
        glPopMatrix();

        Vector3f prev_point;
        if(curr_segment->parent) {
            prev_point = curr_segment->parent->world_pi;
        } else {
            prev_point = arm->origin;
        }
        float rx = curr_segment->world_pi[0] - prev_point[0];
        float ry = curr_segment->world_pi[1] - prev_point[1];
        float rz = curr_segment->world_pi[2] - prev_point[2];
        if (rz == 0) {
            rz = 0.001;
        }
        Vector3f r = Vector3f(rx, ry, rz);
        float angle = 57.2957795*acos( rz/r.norm());
        if (rz < 0.0) {
            angle = -1 * angle;
        }
        float rot_x = -ry*rz;
        float rot_y = rx*rz;
        
        glPushMatrix();
        quad = gluNewQuadric();
        gluQuadricNormals(quad, GLU_SMOOTH);
        gluQuadricTexture(quad, GL_TRUE); 
        glTranslatef(prev_point[0], prev_point[1], prev_point[2]); 
        glRotatef(angle, rot_x, rot_y, 0.0);
        gluCylinder(quad, 0.05, 0.05, r.norm(), 32, 32);
        glPopMatrix();

        curr_segment = curr_segment->child;
    } 
}

/*
void drawShapes(){
    if (is_adaptive) {
        glBegin(GL_TRIANGLES) ;
    } else {
        glBegin(GL_QUADS);
    }
    for(shape s: shapes) {
        glColor3f(0.0f, 0.0f, 1.0f);
        int i = 0;
        for(glm::vec3 v : s.vertices) {
            glNormal3f(s.normals[i][0], s.normals[i][1], s.normals[i][2]);
            glVertex3f(v[0], v[1], v[2]);
            i++;
        }
    }
    glEnd();
}
*/


//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void display( GLFWwindow* window )
{
    //glOrtho(0*zoom, Width_global*zoom, 0*zoom, Height_global*zoom, 1, -1);
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //clear background screen to black
    GLfloat light_pos[] = {1.0, 2.0, -3.0, 1.0}; 
    GLfloat ambient_light[] = {0.9, 0.9, 0};
    GLfloat diffuse_light[] = {0, 0.9, 0.9};
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
    
    //drawCube(); // REPLACE ME!
    //drawShapes();
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

void print_vec(Vector3f vec) { 
    cout << vec[0] << " " << vec[1] << " " << vec[2] << endl;
} 


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
    //This initializes glfw
    initializeRendering();
    Vector3f rot_axis = Vector3f(0.01,0.01,0.01);
    Vector3f x = Vector3f(5,4,1);
    cout << "pre rotation: " << endl;
    print_vec(x);
    cout << "post rotation: " << endl;
    
    cout << "1" << endl;
    GLFWwindow* window = glfwCreateWindow( Width_global, Height_global, "CS184", NULL, NULL );
    if ( !window )
    {
        cerr << "Error on window creating" << endl;
        glfwTerminate();
        return -1;
    }

    cout << "2" << endl;
    const GLFWvidmode * mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    if ( !mode )
    {
        cerr << "Error on getting monitor" << endl;
        glfwTerminate();
        return -1;
    }
    
    cout << "3" << endl;
    glfwMakeContextCurrent( window );
    
    // Get the pixel coordinate of the window
    // it returns the size, in pixels, of the framebuffer of the specified window
    glfwGetFramebufferSize(window, &Width_global, &Height_global);
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(-3.5, 3.5, -3.5, 3.5, 5, -5);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

    glEnable(GL_DEPTH_TEST);	// enable z-buffering
    glDepthFunc(GL_LESS);

    glfwSetWindowTitle(window, "CS184");
    glfwSetWindowSizeCallback(window, size_callback);
    glfwSetKeyCallback(window, key_callback);
    cout << "4" << endl;

    
    int path_mode = 0;
    initialize_goal();
    arm = new Arm();
    epsilon = 0.05f;
    step_size = 0.5f;


    while( !glfwWindowShouldClose( window ) ) // infinite loop to draw object again and again
    {   // because once object is draw then window is terminated
        display( window );
        update_goal(path_mode);
        arm->update_position(epsilon, step_size);
        
        if (auto_strech){
            glfwSetWindowSize(window, mode->width, mode->height);
            glfwSetWindowPos(window, 0, 0);
        }
        
        glfwPollEvents();
        
    }

    return 0;
}
