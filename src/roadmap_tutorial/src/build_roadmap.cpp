#include <boost/date_time.hpp>
#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <roadmap_tutorial/IKManager.h>
#include <roadmap_tutorial/EdgeChecker.h>
#include <roadmap_tutorial/utils.h>

using namespace std;

struct Range {
  float min;
  float max;
  int num_samples;
};

/* 3D grid of pairs of (list index, XYZ location) */
typedef vector<vector<vector<pair<int, KDL::Vector> > > > ConfigMap;

/* Find the closest neighbor in each of the sparse config layers to the
   given location.  This would be more efficient in a kd-tree, but for
   the sake of keeping this example simple, will simply traverse the
   structure  ********************************************************/
void FindNNs(const ConfigMap& config_map,
             const KDL::Vector& loc,
             vector<size_t>& ngbr_ids) {
  //Initialize dists to Inf
  ngbr_ids.resize(config_map[0][0].size());
  vector<float> dists(ngbr_ids.size(),
                      numeric_limits<float>::max());
  for (size_t i = 0; i < config_map.size(); i++) {
    for (size_t j = 0; j < config_map[0].size(); j++) {
      for (size_t k = 0; k < config_map[0][0].size(); k++) {
        if (config_map[i][j][k].first >= 0) {
          double curr_dist = SqDist(loc, config_map[i][j][k].second);
          if (curr_dist < dists[k]) {
            dists[k] = curr_dist;
            ngbr_ids[k] = config_map[i][j][k].first;
          }
        }
      }
    }
  }
}


/* Connect each node in the sparse layer directly to the home configurations */
void ConnectHomes(const size_t node_id,
                  const ConfigMap& home_config_map,
                  vector<pair<size_t, size_t> >& candidate_edges) {
  for (size_t i = 0; i < home_config_map.size(); i++) {
    for (size_t j = 0; j < home_config_map[0].size(); j++) {
      for (size_t k = 0; k < home_config_map[0][0].size(); k++) {
        if (home_config_map[i][j][k].first >= 0) {
          candidate_edges.push_back(make_pair(node_id,
                                              home_config_map[i][j][k].first));
        }
      }
    }
  }
}

/* For each node in the dense grid, connect it to the closest node in each
   layer of the sparse grid  ********************************************/
void ConnectLayers(const pair<size_t, KDL::Vector>& curr,
                   const ConfigMap& sparse_config_map,
                   vector<pair<size_t, size_t> >& candidate_edges) {
  vector<size_t> ngbr_ids;
  FindNNs(sparse_config_map, curr.second, ngbr_ids);
  for (size_t layer = 0; layer < sparse_config_map[0][0].size(); layer++) {
    candidate_edges.push_back(make_pair(curr.first, ngbr_ids[layer]));
  }
}

/* Connecting up the grid of nodes to their neighbors *****************/
void ConnectGrid(const ConfigMap& config_map,
                 size_t i, size_t j, size_t k,
                 vector<pair<size_t, size_t> >& candidate_edges) {
  if (i != 0 && config_map[i-1][j][k].first >= 0) {
    candidate_edges.push_back(make_pair(config_map[i][j][k].first,
                                        config_map[i-1][j][k].first));
  }
  if (j != 0 && config_map[i][j-1][k].first >= 0) {
    candidate_edges.push_back(make_pair(config_map[i][j][k].first,
                                        config_map[i][j-1][k].first));
  }
  if (k != 0 && config_map[i][j][k-1].first >= 0) {
    candidate_edges.push_back(make_pair(config_map[i][j][k].first,
                                        config_map[i][j][k-1].first));
  }
}

void AttachEdges(const ConfigMap& dense_config_map,
                 const ConfigMap& sparse_config_map,
                 const ConfigMap& home_config_map,
                 vector<pair<size_t, size_t> >& candidate_edges) {
  //First iterate over all the nodes in the dense grid
  for (size_t i = 0; i < dense_config_map.size(); i++) {
    for (size_t j = 0; j < dense_config_map[0].size(); j++) {
      for (size_t k = 0; k < dense_config_map[0][0].size(); k++) {
        //Only do something if an IK solution was found at this point
        if (dense_config_map[i][j][k].first >= 0) {
          //Connect up the layer in a grid-like fashion
          ConnectGrid(dense_config_map,
                      i, j, k,
                      candidate_edges);
          //Connect to each layer of the sparse grid
          ConnectLayers(dense_config_map[i][j][k],
                        sparse_config_map,
                        candidate_edges);
        }
      }
    }
  }

  //Next iterate over all the nodes in the sparse grid
  for (size_t i = 0; i < sparse_config_map.size(); i++) {
    for (size_t j = 0; j < sparse_config_map[0].size(); j++) {
      for (size_t k = 0; k < sparse_config_map[0][0].size(); k++) {
        //Only do something if an IK solution was found at this point
        if (sparse_config_map[i][j][k].first >= 0) {
          //Connect up the layer in a grid-like fashion
          ConnectGrid(sparse_config_map,
                      i, j, k,
                      candidate_edges);
          //Connect each node in the sparse grid directly to the home configurations
          ConnectHomes(sparse_config_map[i][j][k].first,
                       home_config_map,
                       candidate_edges);
        }
      }
    }
  }
}

/* Iterate over a cartesian area generating IK solutions *******************/
bool IKGrid(IKManager& ik_solver,
            const Range& x_range,
            const Range& y_range,
            const Range& z_range,
            const KDL::JntArray& seed,
            const KDL::Rotation& orientation,
            vector<vector<double> >& config_list,
            ConfigMap& config_map) {
  int total_count = 0;
  int successes = 0;
  // Make sure not to divide by zero if num_samples == 1
  float x_spacing = ((x_range.max - x_range.min) /
                     (x_range.num_samples - 1 ? x_range.num_samples - 1 : 1));
  float y_spacing = ((y_range.max - y_range.min) /
                     (y_range.num_samples - 1 ? y_range.num_samples - 1 : 1));
  float z_spacing = ((z_range.max - z_range.min) /
                     (z_range.num_samples - 1 ? z_range.num_samples - 1 : 1));
  // Telling the solver that we don't care about the yaw of the end effector
  // KDL::Twist defaults to 0 at each index
  KDL::Twist tolerance;
  tolerance(5) = 2*M_PI;

  config_map.resize(x_range.num_samples);
  for (size_t i = 0; i < x_range.num_samples; i++) {
    config_map[i].resize(y_range.num_samples);
    float curr_x = x_range.min + x_spacing * i;
    for (size_t j = 0; j < y_range.num_samples; j++) {
      config_map[i][j].resize(z_range.num_samples);
      float curr_y = y_range.min + y_spacing * j;
      for (size_t k = 0; k < z_range.num_samples; k++) {
        float curr_z = z_range.min + z_spacing * k;
        KDL::Frame pose;
        // Set the goal pose of the IK to the current xyz
        pose.M = orientation;
        pose.p = KDL::Vector(curr_x, curr_y, curr_z);
        vector<double> q_out;
        total_count++;
        // If a solution is found, add it to config_list and the corresponding
        // index into the map. Otherwise add a -1 into the map
        if (ik_solver.Solve(seed, pose, q_out, tolerance)) {
          config_list.push_back(q_out);
          config_map[i][j][k] = make_pair(config_list.size() - 1,
                                          KDL::Vector(curr_x, curr_y, curr_z));
          successes++;
        }
        else {
          config_map[i][j][k] = make_pair(-1, KDL::Vector(curr_x, curr_y, curr_z));
        }
      }
    }
  }
  cout << "Total IKs attempted: " << total_count << endl;
  cout << "Solutions Found: " << successes << endl;
  return true;
}


int main(int argc, char** argv) {
  /* Grab input data ****************************************************/
  ros::init(argc, argv, "build_roadmap");
  ros::NodeHandle nh("~");
  string chain_start;
  string chain_end;
  string urdf_fn;
  string config_fn;
  string edge_fn;
  double timeout;
  vector<double> seed_config;
  nh.param("chain_start", chain_start, string(""));
  nh.param("chain_end", chain_end, string(""));
  if (chain_start.length() == 0 || chain_end.length() == 0) {
    cout << "Chain start: " << chain_start << endl;
    cout << "Chain end: " << chain_end << endl;
    ROS_ERROR("Cannot execute without chain information");
    return EXIT_FAILURE;
  }
  nh.param("timeout", timeout, 0.005);
  nh.param("urdf_fn", urdf_fn, string("/robot_description"));
  nh.param("config_fn", config_fn, string("configs.txt"));
  nh.param("edge_fn", edge_fn, string("edges.txt"));
  nh.getParam("seed_config" , seed_config);
  /***********************************************************************/

  /* Initialize Structures **********************************************/
  vector<vector<double> > config_list;
  ConfigMap dense_config_map;
  ConfigMap sparse_config_map;
  ConfigMap home_config_map;
  KDL::JntArray seed(seed_config.size());
  for (size_t i = 0; i < seed.data.size(); i++) {
    seed(i) = seed_config[i];
  }
  IKManager ik_solver(chain_start,
                      chain_end,
                      timeout,
                      1e-5,
                      urdf_fn);
  if (!ik_solver.Init()) {
    ROS_ERROR("IKSolver Failed to Init");
    return EXIT_FAILURE;
  }

  KDL::Rotation orientation = KDL::Rotation::EulerZYX(0, 0, M_PI);
  Range x_range, y_range, z_range;
  nh.getParam("x_min" , x_range.min);
  nh.getParam("x_max" , x_range.max);
  nh.getParam("dense_x_num_samples" , x_range.num_samples);

  nh.getParam("y_min" , y_range.min);
  nh.getParam("y_max" , y_range.max);
  nh.getParam("dense_y_num_samples" , y_range.num_samples);


  nh.getParam("dense_z_min" , z_range.min);
  nh.getParam("dense_z_max" , z_range.max);
  nh.getParam("dense_z_num_samples" , z_range.num_samples);
  /*********************************************************************/

  /* Sample a dense area ************************************************/
  IKGrid(ik_solver,
         x_range,
         y_range,
         z_range,
         seed,
         orientation,
         config_list,
         dense_config_map);

  /* Adjust ranges and then sample a sparse area ************************/
  nh.getParam("sparse_x_num_samples" , x_range.num_samples);
  nh.getParam("sparse_y_num_samples" , y_range.num_samples);
  nh.getParam("sparse_z_min" , z_range.min);
  nh.getParam("sparse_z_max" , z_range.max);
  nh.getParam("sparse_z_num_samples" , z_range.num_samples);
  IKGrid(ik_solver,
         x_range,
         y_range,
         z_range,
         seed,
         orientation,
         config_list,
         sparse_config_map);

  /* Adjust ranges and sample a couple home configurations ********************/
  nh.getParam("home_x_min" , x_range.min);
  nh.getParam("home_x_max" , x_range.max);
  nh.getParam("home_x_num_samples" , x_range.num_samples);

  nh.getParam("home_y_min" , y_range.min);
  nh.getParam("home_y_max" , y_range.max);
  nh.getParam("home_y_num_samples" , y_range.num_samples);

  nh.getParam("home_z_min" , z_range.min);
  nh.getParam("home_z_max" , z_range.max);
  nh.getParam("home_z_num_samples" , z_range.num_samples);
  
  IKGrid(ik_solver,
         x_range,
         y_range,
         z_range,
         seed,
         orientation,
         config_list,
         home_config_map);
  /*********************************************************************/

  /* Create edges by attaching configurations together *******/
  vector<pair<size_t, size_t> > candidate_edges;
  AttachEdges(dense_config_map,
              sparse_config_map,
              home_config_map,
              candidate_edges);

  cout << candidate_edges.size() << " Candidate edges" << endl;

  /* Collision check the edges to make sure they are safe  ************/
  // list of (node index, node index)
  vector<pair<size_t, size_t> > safe_edges;
  EdgeChecker edge_checker(urdf_fn,
                           ik_solver.chain_names,
                           0.01);
  for (size_t i = 0; i < candidate_edges.size(); i++) {
    const vector<double>& q0 = config_list[candidate_edges[i].first];
    const vector<double>& q1 = config_list[candidate_edges[i].second];
    bool safe;
    if (!edge_checker.EdgeSafe(q0, q1, safe)) {
      return EXIT_FAILURE;
    }
    if (safe) {
      safe_edges.push_back(candidate_edges[i]);
    }
  }
  cout << safe_edges.size() << " edges are safe" << endl;
  /********************************************************************/

  /* Save Results ***************************************************/
  if (!SaveConfigs(config_list, ik_solver.chain_names, config_fn)) {
    cerr << "Couldn't save config results" << endl;
    return EXIT_FAILURE;
  }
  if (!SaveEdges(safe_edges, edge_fn)) {
    cerr << "Couldn't save edge results" << endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
