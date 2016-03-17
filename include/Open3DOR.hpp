#ifndef OPEN3DOR_HPP
#define OPEN3DOR_HPP

#include <vector>
#include <iostream>

/** Vertex class declaration
 *
 *
 */
class Vertex {
public:
    Vertex(void);
    Vertex(float, float, float);
    Vertex(Vertex&&);
    Vertex(const Vertex&);

    float L2Norm(void) const;
    float L2Norm(const Vertex&) const;
    float Dot(const Vertex&);
    Vertex Cross(const Vertex&);
    Vertex Normalize();
    float Angle(const Vertex&);

    bool operator==(const Vertex&) const;
    Vertex& operator=(const Vertex&);
    Vertex& operator=(Vertex&&);
    Vertex operator-(const Vertex&);
    Vertex operator+(const Vertex&);
    Vertex operator*(float);
    Vertex operator/(float);

    virtual ~Vertex();

    float x;
    float y;
    float z;
};

class Edge {
public:
    Edge(void);
    Edge(int, int);
    Edge(Edge&&);
    Edge(const Edge&);

    Edge& operator=(Edge&&);
    Edge& operator=(const Edge&);
    bool operator==(const Edge&) const;

    virtual ~Edge();

    int v1;
    int v2;
};

class Triangle {
public:
    Triangle(void);
    Triangle(int, int, int);
    Triangle(Triangle&&);
    Triangle(const Triangle&);

    bool areNeighbors(const Triangle&);
    float Area(std::vector<Vertex>&);

    Triangle& operator=(Triangle&&);
    Triangle& operator=(const Triangle&);

    virtual ~Triangle();

    int v1;
    int v2;
    int v3;
};

class Mesh {
public:
    Mesh(void);

    void computeDualVertices (void);
    void computeDualEdges (void);
    void computeAdjacency (void);
    void computeDualAdjacency (void);
    void findNeighbors (void);
    std::vector<int> findNearestNeighbors (int, float);
    void computeNormals (void);
    void computeFPFH (void);
    void fittoUnitSphere (void);
    void movetoCenter (void);
    bool empty (void);
    void clear (void);
    Mesh gridFilter (void);
    Mesh statoutFilter (void);

    ~Mesh();

    Vertex centroid;
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    std::vector<Vertex> normals;
    std::vector<std::vector<float>> fpfhist;
    std::vector<int> voxel_grid;

    float grid_size;

    std::vector<Vertex> dvertices;
    std::vector<Vertex> trinormals;
    std::vector<Edge> edges;
    std::vector<Edge> dedges;
    std::vector<std::vector<int>> neighbors;

private :
    void computeNormals_PCA();
};

void read_mesh(const std::string, Mesh&);
void preprocess_mesh(Mesh&);
float local_distance(const Mesh&, const Mesh&);
float dist_L1(const std::vector<float>& , const std::vector<float>&);
int find_type(const std::string);
void save_descriptors(const std::string , const Mesh&);
std::vector<std::vector<float> > load_descriptors(const std::string );
void preprocess_database(const std::string);
void load_database(const std::string, std::vector<Mesh>&, std::vector<std::string>&);

#endif // OPEN3DOR_HPP
