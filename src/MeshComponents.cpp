#include "MeshComponents.hpp"

Vertex::Vertex(float x, float y, float z) : x(x), y(y), z(z) {}
Vertex::Vertex(void) : x(0.0), y(0.0), z(0.0) {}
Vertex::Vertex(Vertex &&other) = default;
Vertex::Vertex(const Vertex &other) = default;
float Vertex::L2Norm(void) const {
  return sqrt((pow(x, 2) + pow(y, 2) + pow(z, 2)));
}
float Vertex::L2Norm(const Vertex &other) const {
  return sqrt(
      (pow(x - other.x, 2) + pow(y - other.y, 2) + pow(z - other.z, 2)));
}
float Vertex::Dot(const Vertex &other) {
  return (x * other.x + y * other.y + z * other.z);
}
Vertex Vertex::Cross(const Vertex &other) {
  Vertex newvertex;
  newvertex.x = (y * other.z - z * other.y);
  newvertex.y = (z * other.x - x * other.z);
  newvertex.z = (x * other.y - y * other.x);
  return newvertex;
}
Vertex Vertex::Normalize() {
  Vertex newvertex;
  newvertex.x = x / this->L2Norm();
  newvertex.y = y / this->L2Norm();
  newvertex.z = z / this->L2Norm();
  return newvertex;
}
float Vertex::Angle(const Vertex &other) {
  float costheta = (this->Dot(other) / (this->L2Norm() * other.L2Norm()));
  float theta = acos(costheta);
  return theta;
}
Vertex &Vertex::operator=(Vertex &&other) = default;
Vertex &Vertex::operator=(const Vertex &other) = default;
bool Vertex::operator==(const Vertex &other) const {
  return (x == other.x && y == other.y && z == other.z);
}
bool Vertex::operator==(Vertex &&other) const {
  return (x == other.x && y == other.y && z == other.z);
}
Vertex Vertex::operator-(const Vertex &other) {
  return Vertex(x - other.x, y - other.y, z - other.z);
}
Vertex Vertex::operator+(const Vertex &other) {
  return Vertex(x + other.x, y + other.y, z + other.z);
}

//Vertex &Vertex::operator+=(const Vertex &other) {
//  return Vertex(x + other.x, y + other.y, z + other.z);
//}
Vertex Vertex::operator*(const float num) {
  return Vertex(x * num, y * num, z * num);
}
Vertex Vertex::operator/(const float num) {
  if (num != 0)
    return Vertex(x / num, y / num, z / num);
  return Vertex(0, 0, 0);
}
Vertex::~Vertex() {}

Edge::Edge(int v1, int v2) : v1(v1), v2(v2) {}
Edge::Edge(void) : v1(0), v2(0) {}
Edge::Edge(Edge &&e) = default;
Edge::Edge(const Edge &e) = default;
Edge &Edge::operator=(Edge &&e) = default;
Edge &Edge::operator=(const Edge &e) = default;
bool Edge::operator==(const Edge &other) const {
  return ((v1 == other.v1 && v2 == other.v2) ||
          (v2 == other.v1 && v1 == other.v2));
}
Edge::~Edge() {}

Triangle::Triangle(int v1, int v2, int v3) : v1(v1), v2(v2), v3(v3) {}
Triangle::Triangle(void) : v1(0), v2(0), v3(0) {}
Triangle::Triangle(Triangle &&t) = default;
Triangle::Triangle(const Triangle &t) = default;
bool Triangle::areNeighbors(const Triangle &other) {
  Edge e1(v1, v2);
  Edge e2(v2, v3);
  Edge e3(v3, v1);

  Edge eo1(other.v1, other.v2);
  Edge eo2(other.v2, other.v3);
  Edge eo3(other.v3, other.v1);

  if (e1 == eo1 || e1 == eo2 || e1 == eo3)
    return true;
  if (e2 == eo1 || e2 == eo2 || e2 == eo3)
    return true;
  if (e3 == eo1 || e3 == eo2 || e3 == eo3)
    return true;

  return false;
}
float Triangle::Area(std::vector<Vertex> &v) {
  Vertex diff1 = v[v2] - v[v1];
  Vertex diff2 = v[v3] - v[v1];
  Vertex n = diff1.Cross(diff2);
  float area = n.L2Norm() / 2.;
  return area;
}
Triangle &Triangle::operator=(Triangle &&t) = default;
Triangle &Triangle::operator=(const Triangle &t) = default;
Triangle::~Triangle() {}

