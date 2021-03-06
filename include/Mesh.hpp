#ifndef MESH_HPP
#define MESH_HPP

#include <memory>
#include <opencv2/opencv.hpp>
#include <pcl/features/fpfh.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

#include "MeshComponents.hpp"

extern "C" {
#include <vl/generic.h>
#include <vl/gmm.h>
#include <vl/fisher.h>
}

// Struct to hold the dFPFH signature of the point as a histogram
struct dFPFHSignature66 {
	float histogram[66];
	float outer_histogram[33];
	float inner_histogram[33];
	static int descriptorSize () { return 66; }
	void populate (pcl::FPFHSignature33 p, pcl::FPFHSignature33 outer, pcl::FPFHSignature33 inner) {
		for (int i = 0; i < 33; ++i) {
			histogram[i] = std::round(p.histogram[i]);
			histogram[i + 33] = std::round(fabs(outer.histogram[i] - inner.histogram[i]));
			outer_histogram[i] = std::round(outer.histogram[i]);
			inner_histogram[i] = std::round(inner.histogram[i]);
			}
	}
};

template <typename T> int sgn(T val);

// Computes the L1 distance between two dFPFH signatures
float l1(const dFPFHSignature66, const dFPFHSignature66);

class Mesh {
public:
	// Helper functions
  Mesh();
	int load (const std::string file_path);
	void exportOff(std::string);
  bool empty();
  void clear();
  void printInfo();
	void passToPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &, pcl::PointCloud<pcl::Normal>::Ptr &) const;
	void setGridSize(float);
  ~Mesh();

	// Processing functions
	void preprocess();
	void computeNormals();
	void computeNormals(int);
	void process();
	void queryProcess();
	float distanceTo(Mesh &other);

  Mesh gridFilter();
  Mesh statoutFilter();
	void triangulate();
	Mesh bilateralFilter(float, float);
	Mesh multilateralFilter(float);
	Mesh partialView(float);

	// Data members
  Vertex centroid = Vertex(0, 0, 0);
  Vertex max, min;
  std::vector<Vertex> vertices;
  std::vector<Triangle> triangles;
  std::vector<Vertex> normals;
  std::vector<Vertex> trinormals;
  std::vector<std::vector<float>> fpfhist;
  std::vector<int> voxel_grid;
  std::vector<cv::Vec3b> colors;
  float grid_size = 0.05;
	float overall_distance;

	VlGMM *_gmm = nullptr;

	float trianglesAreaStd();
	std::vector<std::pair<float, int>> findMaxAreaTriangles();
	void resample();

	void setClass(int );
	int getClass() const;
	int getID() const;
	void setID(int);

	void printSignatureHistogram (int idx);
private:
	// Loading functions
	int loadObj(const std::string file_path);
	int loadOff(const std::string file_path);
	int findType(const std::string line);

	// Preprocessing functions
  void fitToUnitSphere();
  void moveToCenter();
	void calculateCentroid();

	// Computing functions
	void calculateMeanPointDistance();
  void computeNormals_Tri();
  void computeNormals_PCA();
  void computeFPFH();
	void calculatedFPFHSignatures(float radius, float inner_radius, float outer_radius);
	void calculateFisherVectors();
	void calculateFisherVectors(VlGMM *);
	void postProcessFisherVectors();

	// Distance functions
	float localDistanceTo(const Mesh &other);
	float localDistanceToImproved(const Mesh &other);
	float globalDistanceTo(const Mesh &other);

	// Point cloud of dFPFH signature for every point in mesh	
	pcl::PointCloud<dFPFHSignature66>::Ptr dFPFHSignatures;

	// Array of Fisher vectors
	float *fisherVectors = nullptr;

	int obj_class;
	int _id = 0;
};

#endif // MESH_HPP

