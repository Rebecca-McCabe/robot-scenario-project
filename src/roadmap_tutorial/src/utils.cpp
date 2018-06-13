#include <roadmap_tutorial/utils.h>

using namespace std;



bool SaveConfigs(const vector<vector<double> >& config_list,
                 const vector<string>& joint_names,
                 const string& out_fn) {
  ofstream out_file(out_fn);
  if (!out_file.is_open()) {
    cerr << "Error: could not open file " << out_fn << " for saving" << endl;
    return false;
  }
  /* First print all the joint names *********************************/
  out_file << joint_names << endl;
  /* Next print each configuration on a single line ******************/
  for (const auto& config: config_list) {
    out_file << config << endl;
  }
  out_file.close();
  return true;
}


bool SaveEdges(const vector<pair<size_t, size_t> >& edge_list,
               const string& out_fn) {
  ofstream out_file(out_fn);
  if (!out_file.is_open()) {
    cerr << "Error: could not open file " << out_fn << " for saving" << endl;
    return false;
  }
  for (const auto& edge: edge_list) {
    out_file << edge.first << " " << edge.second << endl;
  }
  out_file.close();
  return true;
}


/* Calculate the distance between two points ************************/
float SqDist(const KDL::Vector& pt1,
             const KDL::Vector& pt2) {
  float ans = (pow(pt1(0) - pt2(0), 2) +
               pow(pt1(1) - pt2(1), 2) +
               pow(pt1(2) - pt2(2), 2));
  return ans;
}
