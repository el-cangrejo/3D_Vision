#ifndef MESH_HPP
#define MESH_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

#include "MeshComponents.hpp"

class Mesh {
public:
  Mesh(void);

  void computeDualVertices(void);
  void computeDualEdges(void);
  void computeAdjacency(void);
  void computeDualAdjacency(void);
  void findNeighbors(void);
  std::vector<int> findNearestNeighbors(int, float);
  void computeNormals(void);
  void computeFPFH(void);
  void fittoUnitSphere(void);
  void movetoCenter(void);
  bool empty(void);
  void clear(void);
  Mesh gridFilter(void);
  Mesh statoutFilter(void);
  void printInfo(void);

  ~Mesh();

  Vertex centroid;
  Vertex max, min;
  std::vector<Vertex> vertices;
  std::vector<cv::Vec3b> colors;
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
private:
  void computeNormals_PCA();
};

#endif // MESH_HPP

