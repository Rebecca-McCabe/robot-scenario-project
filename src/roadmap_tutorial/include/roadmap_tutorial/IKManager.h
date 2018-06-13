#ifndef IKMANAGER_H
#define IKMANAGER_H

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

/******************************************************************************
 * This class abstracts away the details of using Trac_IK to generate inverse
 * kinematic solutions, and then using MoveIt to verify that the solutios are
 * collision free.
 ******************************************************************************/

class IKManager{
 public:
  /************************************************************************
   * Constructor needs the start and end link names of the chain involved,
   * a timeout parameter, an acceptable error parameter, and the urdf filename
   *************************************************************************/
  IKManager(std::string chain_start,
            std::string chain_end,
            float timeout,
            float eps,
            std::string urdf);

  /************************************************************************
   * Must call Init() before calling Solve()
   ************************************************************************/
  bool Init();

  /************************************************************************
   * Attempt to bring the last link of the chain to the desired pose.
   * The boolean result indicates success or failure, and the resulting
   * configuration is returned by reference.
   ***********************************************************************/
  bool Solve(const KDL::JntArray& seed,
             const KDL::Frame& pose,
             std::vector<double>& q_out,
             const KDL::Twist& tolerance = KDL::Twist::Zero());

  std::vector<std::string> chain_names;

private:
  /************************************************************************
   * Use MoveIt to test whether the generated solution is collision-free
   ***********************************************************************/
  bool CollisionFree(const std::vector<double>& q);
  bool successful_init;
  TRAC_IK::TRAC_IK ik_solver;
  std::vector<bool> joint_moveable_flag;
  robot_model_loader::RobotModelLoader model_loader;
  robot_model::RobotModelPtr kinematic_model;
  planning_scene::PlanningScene planning_scene;
  robot_state::RobotState& current_state;
};


#endif //IKMANAGER_H
