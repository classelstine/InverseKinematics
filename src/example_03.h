using namespace std;

struct shape {
    vector<glm::vec3> vertices;
    vector<glm::vec3> normals;
    bool has_normals;
};

bool close_enough(glm::vec3 p1, glm::vec3 p2, float epsilon);
// ****************
//  PARSE 
//  Parses a .bez file, and populates the global patches vector
//  ***************
bool parse_file(char* filename);

// ****************
// CREATE SHAPES
// Given the current patches, populate the global shapes vector
// Depending on is_adaptive, shapes will either be a vector of triangles 
// or a vector of squares
// ****************
void create_shapes(void);

// ****************
// ADAPTIVE SUBDIVISION
// Given a single patch and an epsilon, creates the list of triangles which subdivide the patch
// such that no point is further than epsilon away from the surface of the bezier curve
// This entire list is pushed back onto the global list of patches 
// ****************
void adaptive_subdivision(vector<vector<glm::vec3>> patch);

// ****************
// UNIFORM SUBDIVISION
// Given a single patch, step size, and num_steps, creates list of squares that are
// sufficiently small, and pushes them back on to the global list of shapes
// ****************
void uniform_subdivision(void);


// *****************
// TRIANGULATE 
// Given a patch and three (u,v) pairs, and global epsilon, push back onto global shape list
// *****************
void triangulate(vector<vector<glm::vec3>> patch, float p1[2], float p2[2], float p3[2]);

// ****************
// BEZ PATCH INTERPOLATION
// Given a patch and a point defined by (u,v)
// finds the value of the bezier surface at this point
// also finds the normal at this point
// ****************
glm::vec3 patch_interp(vector<vector<glm::vec3>> patch, float u, float v, glm::vec3 * norm);

// *****************
// BEZ CURVE INTERPOLATION
// Given a curve of four control points, finds the bezier interpolation on this curve and
// evaluates at a given u value, and finds the derivative
// *****************
glm::vec3 curve_interp(vector<glm::vec3> curve, float param, glm::vec3 *derivative);
