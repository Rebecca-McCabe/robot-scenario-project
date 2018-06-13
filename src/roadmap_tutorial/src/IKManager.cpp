#include <roadmap_tutorial/IKManager.h>
using namespace std;

IKManager::IKManager(string chain_start,
                     string chain_end,
                     float timeout,
                     float eps,
                     string urdf):
  successful_init(false),
  ik_solver(chain_start, chain_end, urdf, timeout, eps),
  model_loader(urdf),
  kinematic_model(model_loader.getModel()),
  planning_scene(kinematic_model),
  current_state(planning_scene.getCurrentStateNonConst())
{
}


bool IKManager::Init() {
  KDL::Chain chain;
  if (!ik_solver.getKDLChain(chain)) {
    ROS_ERROR("Specified KDL chain not valid");
    return false;
  }
  KDL::JntArray lower_limits;
  KDL::JntArray upper_limits;
  if (!ik_solver.getKDLLimits(lower_limits, upper_limits)) {
    ROS_ERROR("Joint limits encoded in URDF not valid");
    return false;
  }
  if (chain.getNrOfJoints() != lower_limits.data.size() ||
     chain.getNrOfJoints() != upper_limits.data.size()) {
    ROS_ERROR("Number of joint limits doesn't match joints in chain");
    return false;
  }

  joint_moveable_flag.resize(chain.getNrOfJoints());
  for (size_t i = 0; i < joint_moveable_flag.size(); i++) {
    if (chain.segments[i].getJoint().getTypeName() != "None") {
      chain_names.push_back(chain.segments[i].getJoint().getName());
      joint_moveable_flag[i] = true;
    }
    else {
      joint_moveable_flag[i] = false;
    }
  }
  successful_init = true;
  return true;
}


bool IKManager::CollisionFree(const vector<double>& q) {
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  current_state.setVariablePositions(chain_names, q);
  planning_scene.checkSelfCollision(collision_request, collision_result);
  return !collision_result.collision;
}


bool IKManager::Solve(const KDL::JntArray& seed,
                      const KDL::Frame& pose,
                      vector<double>& q_out,
                      const KDL::Twist& tolerance) {
  if (!successful_init) {
    cerr << "Cannot call Solve() before a successful Init!!" << endl;
    return false;
  }
  KDL::JntArray q;
  if (ik_solver.CartToJnt(seed, pose, q, tolerance) < 0) {
    return false;
  }
  for (unsigned int idx = 0; idx < q.rows(); idx++) {
    if (joint_moveable_flag[idx]) {
      q_out.push_back(q(idx));
    }
  }
  return CollisionFree(q_out);
}
