#include "Database.hpp"

void loadDB(std::string db_path, std::vector<Mesh> &meshes)
{
  std::cout << "\e[1;34mStart loading Database: " << db_path << "\n";

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

  boost::filesystem::path p(db_path);

  for (boost::filesystem::directory_iterator dir(p);
       dir != boost::filesystem::directory_iterator(); dir++) {
    std::string db_entry = (*dir).path().string();
    if (db_entry.substr(db_entry.size() - 3, 3) != "obj" &&
	db_entry.substr(db_entry.size() - 3, 3) != "off")
      continue;

    Mesh m;
    m.load(db_entry);
    m.preprocess();
    m.computeNormals();

    meshes.push_back(std::move(m));
  }

  std::sort(meshes.begin(), meshes.end(),
	    [](const Mesh &a, const Mesh &b) -> bool {
	      return a.getID() < b.getID();
	    });

  //	for (auto &m : meshes)
  //		std::cout << m.getID() << "\n";

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "\e[1;34mMeshes loaded to database: " << meshes.size() << "\n";
  std::cout << "End loading Database\nElapsed time: " << elapsed_secs
	    << "\e[0m\n";
}

void preprocessDB(std::vector<Mesh> &meshes,
		  std::vector<Mesh> &processed_meshes)
{
  if (!processed_meshes.empty()) {
    processed_meshes.clear();
    processed_meshes.shrink_to_fit();
  }

  std::cout << "\e[1;34mStart preprocessing Database\n";

  clock_t begin, end;
  double elapsed_secs;
  begin = clock();

  for (int i = 0; i < meshes.size(); ++i) {
    // meshes[i].setGridSize(0.08);
    processed_meshes.push_back(std::move(meshes[i].gridFilter()));
    // processed_meshes.push_back(meshes[i]);
    processed_meshes[i].process();
    processed_meshes[i].setClass(meshes[i].getClass());
  }

  end = clock();
  elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "End preprocessing Database\nElapsed time: " << elapsed_secs
	    << "\e[0m\n";
}

void readObjectClasses(std::string path, std::vector<Mesh> &meshes)
{
  std::ifstream class_file;
  std::string line;

  int count = 0;

  class_file.open(path);
  if (class_file.is_open()) {
    while (getline(class_file, line)) {
      std::istringstream in(line);
      int obj_class;
      int obj_idx;

      in >> obj_class;
      in >> obj_idx;

      //std::cout << "Object : " << obj_idx << " belongs to class : " <<
      //obj_class << "\n";
      //std::cout << "ID : " << meshes[count].getID() << " belongs to class : " << obj_class << "\n";

      meshes[count].setClass(obj_class);
      ++count;
    }
  }

  if (meshes.size() != count) {
    std::cout << "\e[1;32mWARNING: Classes are not same size as Meshes!\e[0m\n";
    std::cout << "\e[1;32mMeshes : " << meshes.size() <<  "Classes : " << count << "\e[0m\n";
	}
  class_file.close();
  printClassesHistogram(meshes);
}

std::vector<int> printClassesHistogram(std::vector<Mesh> &meshes)
{
  std::vector<int> histogram(6, 0);

  for (auto &m : meshes)
    histogram[m.getClass() - 1]++;

  for (int i = 0; i < histogram.size(); ++i)
    std::cout << "Class " << i + 1 << " has " << histogram[i] << " objects\n";

  for (auto i : histogram) {
    std::cout << "\e[1;31;43m";
    for (auto j : std::vector<int>(i))
      std::cout << " ";
    std::cout << "\e[0m\n";
  }
  return histogram;
}

void takePartialView(std::vector<Mesh> &meshes, float percentage)
{
  Mesh partial_mesh;

  for (int i = 0; i < meshes.size(); ++i) {
    partial_mesh = meshes[i].partialView(percentage);
    partial_mesh.exportOff(std::to_string(meshes[i].getID()));
  }
}

std::vector<float> precisionRecall(Mesh &query,
				   std::vector<Mesh> &target_meshes,
				   int num_objects_in_class_q)
{
  std::vector<float> precision(10, 0);

  retrieve(query, target_meshes);

  for (int i = 1; i < precision.size(); ++i) {
    int recall = std::ceil((float)(i * num_objects_in_class_q) / 10.);
    std::cout << "Recall : " << recall << "\n";
    int Tp = 0;
    int Fp = 0;

    for (int j = 0; j < recall; ++j) {
      if (target_meshes[j].getClass() == query.getClass())
	Tp++;
      else
	Fp++;
    }
    std::cout << "Tp : " << Tp << " Fp : " << Fp << "\n";
    precision[i] = (float)Tp / (Tp + Fp);
  }

  return precision;
}

void precisionRecall(std::vector<Mesh> &target_meshes,
		     std::vector<Mesh> &query_meshes)
{
  auto classes = printClassesHistogram(target_meshes);
  auto query_classes = printClassesHistogram(query_meshes);

  //  std::sort(target_meshes.begin(), target_meshes.end(),
  //	    [](const Mesh &a, const Mesh &b) -> bool {
  //	      return a.getClass() < b.getClass();
  //	    });
  //
  //  std::sort(query_meshes.begin(), query_meshes.end(),
  //	    [](const Mesh &a, const Mesh &b) -> bool {
  //	      return a.getClass() < b.getClass();
  //	    });

  std::vector<std::vector<float>> precisions(classes.size(),
					     std::vector<float>(10, 0));

  for (int i = 0; i < query_meshes.size(); ++i) {
    std::vector<float> precision =
	precisionRecall(query_meshes[i], target_meshes,
			classes[query_meshes[i].getClass() - 1]);
    std::cout << "Precision for query : " << i << " which belongs to class "
	      << query_meshes[i].getClass()
	      << " and ID : " << query_meshes[i].getID() << "\n";
    for (auto p : precision)
      std::cout << p << " ";
    std::cout << "\n";

    std::transform(
	precisions[query_meshes[i].getClass() - 1].begin(),
	precisions[query_meshes[i].getClass() - 1].end(), precision.begin(),
	precisions[query_meshes[i].getClass() - 1].begin(), std::plus<float>());
  }

	for (int i = 0; i < precisions.size(); ++i) {
		for (auto &p : precisions[i]) {
			if (query_classes[i] == 0) continue;
			p /= query_classes[i];
		}
	}

  std::cout << "Classes : " << precisions.size()
	    << " precisions : " << precisions[1].size() << "\n";
  for (auto v : precisions) {
    for (auto p : v)
      std::cout << p << " ";
    std::cout << "\n";
  }
}

void retrieve(Mesh &query, std::vector<Mesh> &target_meshes)
{
  clock_t begin, end;
  double elapsed_secs;

  begin = clock();
  for (auto &t : target_meshes)
    float dist = query.distanceTo(t);
  end = clock();
  double elapsed_secs_s = double(end - begin) / CLOCKS_PER_SEC;

  std::sort(target_meshes.begin(), target_meshes.end(),
	    [](const Mesh &a, const Mesh &b) -> bool {
	      return a.overall_distance < b.overall_distance;
	    });
}
