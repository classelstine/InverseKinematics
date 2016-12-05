#include <vector>
#include <iostream>
#include <fstream>
#include <cmath>
#include <glm/vec3.hpp>
#include <glm/glm.hpp>
//include header file for glfw library so that we can use OpenGL
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <fstream> 
#include <sstream>
#include <string> 
#include "example_03.h"
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
vector< vector < vector < glm::vec3>>> patches; // Patches data structure [# patches][4][4][xyz point]
vector<shape> shapes; // Shapes (either triangle or quad)
bool is_adaptive = false; // adaptive or uniform subdivision
bool is_smooth_shade = true; // If not smooth, then flat shading
float step_size;
int num_steps;
bool wireframe_mode = false;
float epsilon;
int total_patches;
float zoom = 1;

inline float sqr(float x) { return x*x; }


bool close_enough(glm::vec3 p1, glm::vec3 p2, float epsilon) {
    float dist = glm::distance(p1, p2); 
    bool is_close = (dist < epsilon);
    return is_close;
}

// ****************
//  PARSE 
//  Parses a .bez file, and populates the global patches vector
//  ***************
bool parse_file(char* filename){
    string line; 
    ifstream bezfile(filename);
    int linecount = 0; 
    if(!bezfile.is_open()){
        printf("Can't read the BEZ file !\n");
        return false;
    }
    vector<vector<glm::vec3>> single_patch; 
    while(getline(bezfile, line)) {
        linecount++;
        if (linecount == 1) { 
            stringstream curr_line(line);
            curr_line >> total_patches;
            continue;
        } 
        if (line.length() == 1) { 
            if (single_patch.size() > 0) { 
                patches.push_back(single_patch);
                single_patch.clear();
            } 
            continue;
        } 
        vector < glm::vec3 > control_points; //First 4 control points 
        vector < float > coords; 
        stringstream curr_line(line); 
        float coord;
        while (curr_line >> coord) { 
            coords.push_back(coord);
        } 
        for (int i = 0; i < 12; i+=3) { 
            glm::vec3 control_point(coords[i], coords[i+1], coords[i+2]); 
            control_points.push_back(control_point); 
        } 
        single_patch.push_back(control_points); 
    }
    if (single_patch.size() > 0) {
        patches.push_back(single_patch);
        single_patch.clear();
    } 
    return true;
}

// ****************
// CREATE SHAPES
// Given the current patches, populate the global shapes vector
// Depending on is_adaptive, shapes will either be a vector of triangles 
// or a vector of squares
// ****************
void create_shapes(void){
    if (is_adaptive) { 
        for(int i = 0; i < patches.size(); i++) { 
            vector<vector<glm::vec3>> curr_patch = patches.at(i); 
            float p1[2] =  {0,0};
            float p2[2] = {1,0};
            float p3[2] = {0,1};
            float p4[2]  = {1,1};
            triangulate(curr_patch, p1, p2, p3);  
            triangulate(curr_patch, p2, p3, p4);  
        } 
    } else { 
        uniform_subdivision(); 
    } 
}

// ****************
// ADAPTIVE SUBDIVISION
// Given a single patch and an epsilon, creates the list of triangles which subdivide the patch
// such that no point is further than epsilon away from the surface of the bezier curve
// This entire list is pushed back onto the global list of patches 
// ****************
void adaptive_subdivision(vector<vector<glm::vec3>> patch){
}

// ****************
// UNIFORM SUBDIVISION
// Given a single patch, step size, and num_steps, creates list of squares that are
// sufficiently small, and pushes them back on to the global list of shapes
// ****************
void uniform_subdivision(void) {
    for(int i = 0; i < patches.size(); i++) { 
        vector<vector<glm::vec3>> curr_patch = patches.at(i); 
        for(int j = 0; j < num_steps; j++) { 
            for(int k = 0; k < num_steps; k++) { 
                float u = j*step_size;  
                float v = k*step_size;  
                glm::vec3 n1, n2, n3, n4;
                glm::vec3 p1, p2, p3, p4;
                p1 = patch_interp(curr_patch, u, v, &n1); 
                p2 = patch_interp(curr_patch, u+step_size, v, &n2); 
                p3 = patch_interp(curr_patch, u+step_size, v+step_size, &n3); 
                p4 = patch_interp(curr_patch, u, v+step_size, &n4); 
                shape quad = shape();
                quad.vertices = {p1, p2, p3, p4}; 
                quad.normals = {n1, n2, n3, n4}; 
                shapes.push_back(quad);
            } 
        } 
    }
}


// *****************
// TRIANGULATE 
// Given a patch and three (u,v) pairs, and global epsilon, push back onto global shape list
// *****************
void triangulate(vector<vector<glm::vec3>> curr_patch, float p1[2], float p2[2], float p3[2]){
    glm::vec3 v1, v2, v3, n1, n2, n3;
    v1 = patch_interp(curr_patch, p1[0], p1[1], &n1); 
    v2 = patch_interp(curr_patch, p2[0], p2[1], &n2); 
    v3 = patch_interp(curr_patch, p3[0], p3[1], &n3); 
    
    glm::vec3 mid12, mid13, mid23; 
    mid12 = (v1+v2)/2.0f;
    mid13 = (v1+v3)/2.0f;
    mid23 = (v2+v3)/2.0f;
    
    glm::vec3 bez12, bez13, bez23;
    glm::vec3 n12, n13, n23;
    float uv12[2] = { (p1[0] + p2[0])/2, (p1[1] + p2[1])/2};
    float uv13[2] = { (p1[0] + p3[0])/2, (p1[1] + p3[1])/2 };
    float uv23[2] = { (p2[0] + p3[0])/2, (p2[1] + p3[1])/2 };
    bez12 = patch_interp(curr_patch, uv12[0], uv12[1], &n12);
    bez13 = patch_interp(curr_patch, uv13[0], uv13[1], &n13);
    bez23 = patch_interp(curr_patch, uv23[0], uv23[1], &n23);

    bool touching12 = close_enough(mid12, bez12, epsilon);
    bool touching13 = close_enough(mid13, bez13, epsilon);
    bool touching23 = close_enough(mid23, bez23, epsilon);

    if (touching12 && touching13 && touching23) {
        shape tri = shape();
        tri.vertices.push_back(v1);
        tri.vertices.push_back(v2);
        tri.vertices.push_back(v3);

        tri.normals.push_back(n1);
        tri.normals.push_back(n2);
        tri.normals.push_back(n3);
        shapes.push_back(tri);
    } 
    else if (touching12 && touching13 && !touching23) {
        triangulate(curr_patch, p1, p2, uv23);
        triangulate(curr_patch, p1, p3, uv23);
    } 
    else if (touching12 && !touching13 && touching23) {
        triangulate(curr_patch, p1, p2, uv13);
        triangulate(curr_patch, p2, p3, uv13);
    } 
    else if (!touching12 && touching13 && touching23) {
        triangulate(curr_patch, p1, p3, uv12);
        triangulate(curr_patch, p2, p3, uv12);
    } 
    else if (touching12 && !touching13 && !touching23) {
        triangulate(curr_patch, p3, uv23, uv13);
        triangulate(curr_patch, p2, uv13, uv23);
        triangulate(curr_patch, p1, p2, uv13);
    }
    else if (!touching12 && !touching13 && touching23) {
        triangulate(curr_patch, p1, uv12, uv13);
        triangulate(curr_patch, p2, uv12, uv13);
        triangulate(curr_patch, p2, p3, uv13);
    } 
    else if (!touching12 && touching13 && !touching23) {
        triangulate(curr_patch, p2, uv12, uv23);
        triangulate(curr_patch, p1, uv12, uv23);
        triangulate(curr_patch, p1, p3, uv23);
    } 
    else if (!touching12 && !touching13 && !touching23) {
        triangulate(curr_patch, p1, uv12, uv13);
        triangulate(curr_patch, p2, uv12, uv23);
        triangulate(curr_patch, p3, uv13, uv23);
        triangulate(curr_patch, uv12, uv13, uv23);
    } 
}

// ****************
// BEZ PATCH INTERPOLATION
// Given a patch and a point defined by (u,v)
// finds the value of the bezier surface at this point
// also finds the normal at this point
// ****************
glm::vec3 patch_interp(vector<vector<glm::vec3>> patch, float u, float v, glm::vec3 * norm) {
    glm::vec3 u1, u2, u3, u4, v1, v2, v3, v4, p;
    glm::vec3 du, dv;
    u1 = curve_interp(patch[0], v, &dv);
    u2 = curve_interp(patch[1], v, &dv);
    u3 = curve_interp(patch[2], v, &dv);
    u4 = curve_interp(patch[3], v, &dv);

    vector<glm::vec3> curve_v1, curve_v2, curve_v3, curve_v4;
    curve_v1 = {patch[0][0], patch[1][0], patch[2][0], patch[3][0]};
    v1 = curve_interp(curve_v1, u, &dv);
    curve_v2 = {patch[0][1], patch[1][1], patch[2][1], patch[3][1]};
    v2 = curve_interp(curve_v2, u, &dv);
    curve_v3 = {patch[0][2], patch[1][2], patch[2][2], patch[3][2]}; 
    v3 = curve_interp(curve_v3, u, &dv);
    curve_v4 = {patch[0][3], patch[1][3], patch[2][3], patch[3][3]}; 
    v4 = curve_interp(curve_v4, u, &dv);
    
    vector<glm::vec3> curve_u = {u1, u2, u3, u4}; 
    vector<glm::vec3> curve_v = {v1, v2, v3, v4};
    p = curve_interp(curve_u, u, &du);
    p = curve_interp(curve_v, v, &dv);
    *norm = glm::normalize(glm::cross(du, dv));
    if (isnan(glm::length(*norm) )) {
        float nu = u + 0.1;
        float nv = v + 0.1;
        patch_interp(patch, nu, nv, norm);
    }

    return p;
}

// *****************
// BEZ CURVE INTERPOLATION
// Given a curve of four control points, finds the bezier interpolation on this curve and
// evaluates at a given u value, and finds the derivative
// *****************
glm::vec3 curve_interp(vector<glm::vec3> curve, float param, glm::vec3 *dPdU) {
    glm::vec3 A, B, C, D, E, p; 

    A = curve[0]*(1.0f - param) + curve[1]*param;
    B = curve[1]*(1.0f - param) + curve[2]*param;
    C = curve[2]*(1.0f - param) + curve[3]*param;

    D = A*(1.0f - param) + B*param;
    E = B*(1.0f - param) + C*param;

    p = D*(1.0f - param) + E*param;

    *dPdU = 3.0f*(E-D);

    return p;
}


//****************************************************
// Simple init function
//****************************************************
void initializeRendering()
{
    glfwInit();
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
        case GLFW_KEY_F :
            if (action && mods == GLFW_MOD_SHIFT) auto_strech = !auto_strech; break;
        case GLFW_KEY_S : 
            if (action) is_smooth_shade = !is_smooth_shade;
            break;
        case GLFW_KEY_W : 
            if (action) wireframe_mode = !wireframe_mode;
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
//****************************************************
// Draw a cube. You don't need this for your final assignment, but it's
// here so you don't look at a blank screen.
// Taken from https://www.ntu.edu.sg/home/ehchua/programming/opengl/CG_Examples.html
//****************************************************
/*
void drawCube() {
    
    shapes = vector<shape>();
    shape shape1 = shape();
    
    
             // Begin drawing the color cube with 6 quads
      // Top face (y = 1.0f)
      // Define vertices in counter-clockwise (CCW) order with normal pointing out

    shape1.vertices.push_back(glm::vec3( 1.0f, 1.0f, -1.0f));
    shape1.vertices.push_back(glm::vec3(-1.0f, 1.0f, -1.0f));
    shape1.vertices.push_back(glm::vec3(-1.0f, 1.0f,  1.0f));
    shape1.vertices.push_back(glm::vec3( 1.0f, 1.0f,  1.0f));
    shapes.push_back(shape1);
      // Bottom face (y = -1.0f)
 
    shape shape2 = shape();
    shape2.vertices.push_back(glm::vec3( 1.0f, -1.0f,  1.0f));
    shape2.vertices.push_back(glm::vec3(-1.0f, -1.0f,  1.0f));
    shape2.vertices.push_back(glm::vec3(-1.0f, -1.0f, -1.0f));
    shape2.vertices.push_back(glm::vec3( 1.0f, -1.0f, -1.0f));
    shapes.push_back(shape2);

    shape shape3 = shape();    
      // Front face  (z = 1.0f)
    shape3.vertices.push_back(glm::vec3( 1.0f,  1.0f, 1.0f));
    shape3.vertices.push_back(glm::vec3(-1.0f,  1.0f, 1.0f));
    shape3.vertices.push_back(glm::vec3(-1.0f, -1.0f, 1.0f));
    shape3.vertices.push_back(glm::vec3(1.0f, -1.0f, 1.0f));
    shapes.push_back(shape3);

    shape shape4 = shape(); 
      // Back face (z = -1.0f)
    shape4.vertices.push_back(glm::vec3 (1.0f, -1.0f, -1.0f));
     shape4.vertices.push_back(glm::vec3 (-1.0f, -1.0f, -1.0f));
     shape4.vertices.push_back(glm::vec3 (-1.0f,  1.0f, -1.0f));
     shape4.vertices.push_back(glm::vec3 ( 1.0f,  1.0f, -1.0f));
 
    shapes.push_back(shape4);
    shape shape5 = shape();
    
      // Left face (x = -1.0f)
      shape5.vertices.push_back(glm::vec3(-1.0f,  1.0f,  1.0f));
       shape5.vertices.push_back(glm::vec3(-1.0f,  1.0f, -1.0f));
       shape5.vertices.push_back(glm::vec3(-1.0f, -1.0f, -1.0f));
       shape5.vertices.push_back(glm::vec3(-1.0f, -1.0f,  1.0f));
 
    shapes.push_back(shape5);
    shape shape6 = shape();
    
      // Right face (x = 1.0f)
      shape6.vertices.push_back(glm::vec3(1.0f,  1.0f, -1.0f));
      shape6.vertices.push_back(glm::vec3(1.0f,  1.0f,  1.0f));
      shape6.vertices.push_back(glm::vec3(1.0f, -1.0f,  1.0f));
      shape6.vertices.push_back(glm::vec3(1.0f, -1.0f, -1.0f));
    
    shapes.push_back(shape6);
    drawShapes();
}
*/

//****************************************************
// function that does the actual drawing of stuff
//***************************************************
void display( GLFWwindow* window )
{
    
    //glOrtho(0*zoom, Width_global*zoom, 0*zoom, Height_global*zoom, 1, -1);
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f ); //clear background screen to black
    GLfloat light_pos[] = {1.0, 2.0, 3.0, 1.0}; 
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                // clear the color buffer (sets everything to black)
    glMatrixMode(GL_MODELVIEW);                  // indicate we are specifying camera transformations
    glLoadIdentity();                            // make sure transformation is "zero'd"
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    
    //----------------------- code to draw objects --------------------------
    
    //glOrtho(0*zoom, Width_global*zoom, 0*zoom, Height_global*zoom, 1, -1);
    glPushMatrix();
    glTranslatef (translation[0], translation[1], translation[2]);
    glRotatef(rotation[0], 1.0f, 0.0f, 0.0f);
    glRotatef(rotation[1], 0.0f, 1.0f, 0.0f);
    glRotatef(rotation[2], 0.0f, 0.0f, 1.0f);
    
    //drawCube(); // REPLACE ME!
    drawShapes();

    glPopMatrix();

    if (wireframe_mode) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    } else {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    if (is_smooth_shade) {
        glShadeModel(GL_SMOOTH);
    } else {
        glShadeModel(GL_FLAT);
    }
    
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


//****************************************************
// the usual stuff, nothing exciting here
//****************************************************
int main(int argc, char *argv[]) {
    //This initializes glfw
    initializeRendering();
    
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

    if (argc < 3) { 
        cout << "ERROR: INVALID PROGRAM PARAMETERS" << endl; 
    } else if (argc == 3) { 
        parse_file(argv[1]); 
        is_adaptive = false;
        step_size = atof(argv[2]);  
        num_steps = ceil(1.0f/step_size); 
    } else if (argc == 4) { 
        parse_file(argv[1]); 
        epsilon = atof(argv[2]);  
        is_adaptive = true;
    } 
    for (int i = 0; i < total_patches; i++) { 
        for (int j = 0; j < 4; j++) { 
            for (int k = 0; k < 4; k++) { 
            } 
        } 
    } 
    create_shapes(); 
    /*
    for (int i = 0; i < total_patches; i++) { 
        cout << "this is patch " << i << endl;
        for (int j = 0; j < 4; j++) { 
            cout << "this is v" << j << endl;
            for (int k = 0; k < 4; k++) { 
                cout << "array index patches[" << i << "]" << "[" << j << "]" << "[" << k << "]" << endl;
                cout << patches[i][j][k][0] << " " << patches[i][j][k][1] << " " << patches[i][j][k][2] << endl;
            } 
        } 
    } 
    */
    while( !glfwWindowShouldClose( window ) ) // infinite loop to draw object again and again
    {   // because once object is draw then window is terminated
        display( window );
        
        if (auto_strech){
            glfwSetWindowSize(window, mode->width, mode->height);
            glfwSetWindowPos(window, 0, 0);
        }
        
        glfwPollEvents();
        
    }

    return 0;
}
