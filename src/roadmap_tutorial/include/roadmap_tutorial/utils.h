#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <kdl/chainiksolverpos_nr_jl.hpp>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& arr) {
  for (size_t i = 0; i < arr.size(); ++i) {
    os << arr[i];
    if (i != arr.size() - 1) {
      os << " ";
    }
  }
  return os;
}

bool SaveConfigs(const std::vector<std::vector<double> >& config_list,
                 const std::vector<std::string>& joint_names,
                 const std::string& out_fn);

bool SaveEdges(const std::vector<std::pair<size_t, size_t> >& edge_list,
               const std::string& out_fn);

float SqDist(const KDL::Vector& pt1,
             const KDL::Vector& pt2);


#endif //UTILS_H
