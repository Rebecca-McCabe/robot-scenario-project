#include <roadmap_tutorial/EdgeChecker.h>

using namespace std;

EdgeChecker::EdgeChecker(const string& urdf,
                         const vector<string>& joint_names,
                         float edge_eps):
  joint_names_(joint_names),
  edge_eps_(edge_eps),
  model_loader_(urdf),
  kinematic_model_(model_loader_.getModel()),
  planning_scene_(kinematic_model_),
  current_state_(planning_scene_.getCurrentStateNonConst())
{}


bool EdgeChecker::NodeSafe(const vector<double>& q) {
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  current_state_.setVariablePositions(joint_names_, q);
  planning_scene_.checkSelfCollision(collision_request, collision_result);
  if (collision_result.collision) {
    return false;
  }
  else {
    return true;
  }
}

bool EdgeChecker::EdgeSafe(const vector<double>& q0,
                           const vector<double>& q1,
                           bool& safe) {
  if (q0.size() != q1.size() ||
     q0.size() != joint_names_.size()) {
    cerr << "Configurations provided not same size,"
         << " or not same size as the chain provided."
         << "q0 size: " << q0.size() << endl
         << "q1 size: " << q1.size() << endl
         << "joint_names_ size: " << joint_names_.size() << endl
         << endl;
    return false;
  }
  size_t steps = 1;
  float step_joint_dist = JointDistance(q0, q1);

  /***********************************************************************
   * This EdgeChecker successively splits the edge into smaller halves
   * until the step-distance is smaller than the edge_eps.
   * To avoid re-checking the same node multiple times, the loop
   * only checks the odd-numbered samples in the current iteration,
   * since previous iterations checked the even-numbered samples.
  **********************************************************************/
  while (step_joint_dist > edge_eps_) {
    steps *= 2;
    step_joint_dist *= 0.5;
    float step_size = 1.0/float(steps);
    for (size_t i = 0; i < steps/2; i++) {
      float u = step_size + i * 2 * step_size;
      vector<double> q;
      InterpolateLinear(q0, q1, u, q);
      if (!NodeSafe(q)) {
        safe = false;
        return true;
      }
    }
  }
  safe = true;
  return true;
}


float EdgeChecker::JointDistance(const vector<double>& q0,
                                 const vector<double>& q1) const {
  float dist_sqr = 0.0;
  for (size_t i = 0; i < q0.size(); i++) {
    dist_sqr += pow(q0[i] - q1[i], 2);
  }
  return pow(dist_sqr, 0.5);
}

void EdgeChecker::InterpolateLinear(const vector<double>& q0,
                                    const vector<double>& q1,
                                    const float u,
                                    vector<double>& q_out) const {
  q_out.resize(q0.size());
  for (size_t i = 0; i < q1.size(); i++) {
    q_out[i] = q0[i] + u*(q1[i] - q0[i]);
  }
}
