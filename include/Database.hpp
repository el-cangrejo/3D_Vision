#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include "Mesh.hpp"

void loadDB(std::string db_path, std::vector<Mesh> &meshes);

void preprocessDB(std::vector<Mesh> &meshes, std::vector<Mesh> &processed_meshes);

void writeDBDescriptors(std::vector<Mesh> &meshes);

void readDBDescriptors(std::vector<Mesh> &meshes);

void readObjectClasses(std::string path, std::vector<Mesh> &meshes);

void printClassesHistogram(std::vector<Mesh> &);

void takePartialView(std::vector<Mesh> &, float);
#endif //DATABASE_HPP
