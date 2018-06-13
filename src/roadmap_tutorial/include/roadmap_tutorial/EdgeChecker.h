#ifndef EDGE_CHECKER_H
#define EDGE_CHECKER_H

#include <trac_ik/trac_ik.hpp>
#include <ros/ros.h>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>

/******************************************************************************
 * This class abstracts away the details of using MoveIt to collision check
 * the motion between two joint positions. There is the underlying assumption
 * that the robot is going to follow a linear path through Configuration Space
 * between the two positions. 
 ******************************************************************************/

class EdgeChecker{
 public:

  /************************************************************************ 
   * Constructor needs the urdf filename, and a list of joint names involved
   * in the edges that will be checked 
   *************************************************************************/
  EdgeChecker(const std::string& urdf,
              const std::vector<std::string>& joint_names,
              float edge_eps = 0.01);

  
  /**************************************************************************
   * This function puts the result in the boolean reference. The return value
   * indicates whether the problem was well-founded or not. It returns false
   * if the two configurations are of different size, or if either doesn't
   * match the length of the joint_names provided upon construction 
   *************************************************************************/
  bool EdgeSafe(const std::vector<double>& q0,
                const std::vector<double>& q1,
                bool& safe);
  
 private:

  /************************************************************************** 
   * This function checks whether a single configuration of the robot is safe.
   ***************************************************************************/
  bool NodeSafe(const std::vector<double>& q);

  /*********************************************************************** 
   * This function calculates the L2 norm of the difference between the two
   * configurations 
  *************************************************************************/
  float JointDistance(const std::vector<double>& q0,
                      const std::vector<double>& q1) const;

  /*********************************************************************** 
   * Linearly interpolate between two configurations by a factor u = [0, 1]
   * u = 0 will yield q0, u = 1 will yield q1, u = 0.5 the midpoint, etc.
  *************************************************************************/
  void InterpolateLinear(const std::vector<double>& q0,
                         const std::vector<double>& q1,
                         const float u,
                         std::vector<double>& q_out) const;
  
  std::vector<std::string> joint_names_;
  float edge_eps_;
  robot_model_loader::RobotModelLoader model_loader_;
  robot_model::RobotModelPtr kinematic_model_;
  planning_scene::PlanningScene planning_scene_;
  robot_state::RobotState& current_state_;
};

#endif  //EDGE_CHECKER_H
