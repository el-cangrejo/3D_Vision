#include "Database.hpp"


void loadDB(const std::string db_path, std::vector<Mesh> &meshes) {
  boost::filesystem::path p(db_path);

  for (boost::filesystem::directory_iterator dir(p);
       dir != boost::filesystem::directory_iterator(); dir++) {
    if (!boost::filesystem::is_regular_file(*dir))
      continue;

    std::string db_entry = (*dir).path().string();

    if (db_entry.substr(db_entry.size() - 3, 3) != "obj" || 
				db_entry.substr(db_entry.size() - 3, 3) != "off")
      continue;

    Mesh m;
		m.load(db_entry);
		m.preprocess();

		meshes.push_back(m);
  }
  std::cout << "Meshes load to database for display : " << meshes.size() << "\n";
}
