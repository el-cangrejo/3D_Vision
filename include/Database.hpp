#ifndef DATABASE_HPP
#define DATABASE_HPP

#include <string>
#include <vector>
#include <boost/filesystem.hpp>

#include "Mesh.hpp"

void loadDB(std::string db_path, std::vector<Mesh> &meshes);

#endif //DATABASE_HPP
