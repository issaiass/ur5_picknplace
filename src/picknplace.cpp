#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class PickNPlace
{
public:
  explicit PickNPlace(ros::NodeHandle nh);
  ~PickNPlace();
  bool movePose(_Float64 x, _Float64 y, _Float64 z, _Float64 roll, _Float64 pitch, _Float64 yaw);
  bool movePose(geometry_msgs::Pose pose);
  bool movePose(geometry_msgs::Pose pose, tf2::Quaternion rpy_orientation);
  
  const char *getPlanningFrame(void);
  const char *getEELink(void);

  const std::string ARM_GROUP = "arm";
  const std::string GRIPPER_GROUP = "gripper";

  moveit::planning_interface::MoveGroupInterface arm;
  moveit::planning_interface::MoveGroupInterface gripper;

  enum GRIPPER_OPERATION {OPEN, CLOSE};

  void CloseGripper(void);
  void OpenGripper(void);
  void attachObject(void);
  void detachObject(void);
  void supportSurface(std::string surface);
private:
  ros::NodeHandle _nh;
  const robot_state::JointModelGroup *arm_joint_model_group;
  const robot_state::JointModelGroup *gripper_joint_model_group;  
  bool OperateGripper(const bool &close_gripper);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	moveit_msgs::AttachedCollisionObject attached_object;  
	moveit_msgs::CollisionObject grasping_object;
protected:

};

PickNPlace::PickNPlace(ros::NodeHandle nh)
  : _nh(nh),
    arm(ARM_GROUP),
    gripper(GRIPPER_GROUP)
{

}

PickNPlace::~PickNPlace() {}

bool PickNPlace::movePose(_Float64 x, _Float64 y, _Float64 z, _Float64 roll, _Float64 pitch, _Float64 yaw)
{
  tf2::Quaternion rpy_orientation; 
  geometry_msgs::Pose pose;
  moveit::core::RobotStatePtr arm_current_state;
  std::vector<double> arm_joint_positions;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;


  arm_joint_model_group = arm.getCurrentState()->getJointModelGroup(ARM_GROUP);
  arm_current_state = arm.getCurrentState();
  arm_current_state->copyJointGroupPositions(arm_joint_model_group, arm_joint_positions);
  ROS_INFO("No. of joints in arm_group: %zd", arm_joint_positions.size());
  success = (arm.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);  
  if (success) {
    ROS_INFO("Visualizing plan 1 (pose goal) %s", success ? "":"FAILED");    
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    rpy_orientation.setRPY(roll, pitch, yaw);
    pose.orientation = tf2::toMsg(rpy_orientation);
    arm.setPoseTarget(pose);
    ros::Duration(1.5).sleep();
    arm.move();
  }
  return success;
}

bool PickNPlace::OperateGripper(const bool &close_gripper)
{
  moveit::core::RobotStatePtr gripper_current_state;
  std::vector<double> gripper_joint_positions;
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success;

  gripper_joint_model_group = gripper.getCurrentState()->getJointModelGroup(GRIPPER_GROUP);
  gripper_current_state = gripper.getCurrentState();
  gripper_current_state->copyJointGroupPositions(gripper_joint_model_group, gripper_joint_positions);
  ROS_INFO("No. of joints in eef_group: %zd", gripper_joint_positions.size());
  gripper_joint_positions[0] = 0;     // radians
  if (close_gripper)
    gripper_joint_positions[0] = 0.3625;  // radians
  gripper.setJointValueTarget(gripper_joint_positions);
  success = (gripper.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    ROS_INFO("Visualizing gripper_plan (pose goal) %s", success ? "":"FAILED");
    ros::Duration(1.5).sleep();
    gripper.move();
  }
  return success;
}

void PickNPlace::attachObject(void) 
{
	ROS_INFO("Attaching object grasping_object to robot's body");
  grasping_object.id = "cube1";
	attached_object.link_name = "gripper_base_link";
	attached_object.object = grasping_object;
	planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

void PickNPlace::detachObject(void) 
{
	ROS_INFO("Detaching object grasping_object to robot's body");  
	grasping_object.operation = grasping_object.REMOVE;
	attached_object.link_name = "gripper_base_link";
	attached_object.object = grasping_object;
	planning_scene_interface.applyAttachedCollisionObject(attached_object);
}

const char* PickNPlace::getPlanningFrame(void)
{
  return arm.getPlanningFrame().c_str();
}

const char* PickNPlace::getEELink(void)
{
  return gripper.getEndEffectorLink().c_str();
}

void PickNPlace::OpenGripper(void)
{
  OperateGripper(false);
  detachObject();
}

void PickNPlace::CloseGripper(void)
{
  OperateGripper(true);
  attachObject();
}

void PickNPlace::supportSurface(std::string surface)
{
    gripper.setSupportSurfaceName(surface);  // neccesary for picknplace application
    arm.setSupportSurfaceName(surface);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "picknplace");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Duration(2.0).sleep();
  PickNPlace pnp(node_handle);
  ROS_INFO("Planning frame: %s", pnp.getPlanningFrame());
  ROS_INFO("End Effector frame: %s", pnp.getEELink());
  pnp.supportSurface("cafe_table");
  pnp.OpenGripper(); // open tool
  pnp.movePose(0.17, -0.20, 0.90, 0.00, 0.00, -1.5707); // Move over cube1
  pnp.movePose(0.17, -0.20, 0.82, 0.00, 0.00, -1.5707); // approach to cube1 
  pnp.CloseGripper(); // close tool
  pnp.movePose(0.17, -0.20, 0.90, 0.00, 0.00, -1.5707); // move far
  pnp.movePose(0.00, -0.80, 0.70, -1.5707, 0.00, 0.00); // move to dumpster
  pnp.OpenGripper(); // open tool
  ros::shutdown();  
  return 0;
}