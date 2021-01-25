#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometric_shapes/shape_operations.h>

class CollisionTasks
{
public:
  explicit CollisionTasks();
  ~CollisionTasks();
  void SetPrimitiveCollisionObject(const std::string &object_id,
                    const int8_t type,
                    const geometry_msgs::Pose &object_pose,
                    const std::vector<_Float64> & object_dimensions);


  void SetMeshCollisionObject(const std::string &object_id,
                            const std::string &mesh_path,
                            const geometry_msgs::Pose &object_pose,
                            const _Float64 &scale_factor);

  std::string FIXED_FRAME;
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  enum SolidPrimitive {BOX, CYLINDER, SPHERE, CONE};
 

private:
  // Define PlanningSceneInterface object to add and remove collision objects
  void clearVector(std::vector<_Float64> obj);

};

CollisionTasks::CollisionTasks():FIXED_FRAME("world") {}

CollisionTasks::~CollisionTasks() {}


void CollisionTasks::clearVector(std::vector<_Float64> obj) {
  for (int i = 0; i < obj.size(); i++) {
    obj.erase(obj.begin() + i);
  }
}

void CollisionTasks::SetPrimitiveCollisionObject(const std::string &object_id,
  const int8_t type, const geometry_msgs::Pose &object_pose,
  const std::vector<_Float64> & object_dimensions)
{
  moveit_msgs::CollisionObject collision_obj;
  shape_msgs::SolidPrimitive primitive;  


  collision_obj.id = object_id;
  collision_obj.header.frame_id = FIXED_FRAME;
  switch (type) {
    case CollisionTasks::BOX:
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = object_dimensions.at(0);
      primitive.dimensions[primitive.BOX_Y] = object_dimensions.at(1);
      primitive.dimensions[primitive.BOX_Z] = object_dimensions.at(2); 
      break;
    case CollisionTasks::CYLINDER: 
      primitive.type = primitive.CYLINDER;
      primitive.dimensions.resize(2);
      primitive.dimensions[primitive.CYLINDER_HEIGHT] = object_dimensions.at(0);
      primitive.dimensions[primitive.CYLINDER_RADIUS] = object_dimensions.at(1);
      break;
    case CollisionTasks::SPHERE:
      primitive.type = primitive.SPHERE;
      primitive.dimensions.resize(1); 
      primitive.dimensions[primitive.SPHERE_RADIUS] = object_dimensions.at(0);
      break;
    case CollisionTasks::CONE:
      primitive.type = primitive.CONE;
      primitive.dimensions.resize(2); 
      primitive.dimensions[primitive.CONE_HEIGHT] = object_dimensions.at(0);
      primitive.dimensions[primitive.CONE_RADIUS] = object_dimensions.at(1);
    default:
      ROS_INFO("There value is out of range, no box, cylinder, sphere or cone");
      break;
  }
  clearVector(object_dimensions);
  collision_obj.primitives.push_back(primitive);
  collision_obj.primitive_poses.push_back(object_pose);
  collision_objects.push_back(collision_obj);
}

void CollisionTasks::SetMeshCollisionObject(const std::string &object_id,
                                          const std::string &mesh_path,
                                          const geometry_msgs::Pose &object_pose,
                                          const _Float64 &scale_factor)
{
  moveit_msgs::CollisionObject collision_object;

  collision_object.header.frame_id = FIXED_FRAME;
  collision_object.id = object_id;

  shapes::Mesh* m = shapes::createMeshFromResource(mesh_path);
  m->scale(scale_factor);

  shape_msgs::Mesh object_mesh;
  shapes::ShapeMsg object_mesh_msg;
  shapes::constructMsgFromShape(m, object_mesh_msg);
  object_mesh = boost::get<shape_msgs::Mesh>(object_mesh_msg);
  collision_object.meshes.resize(1); 
  collision_object.mesh_poses.resize(1);
  collision_object.meshes[0] = object_mesh;

  collision_object.mesh_poses[0].position = object_pose.position;
  collision_object.mesh_poses[0].orientation = object_pose.orientation;

  collision_object.meshes.push_back(object_mesh);
  collision_object.mesh_poses.push_back(collision_object.mesh_poses[0]);
  collision_object.operation = collision_object.ADD;

  collision_objects.push_back(collision_object);
}

int main(int argc, char **argv)
{
    // Initialize ROS, create the node handle and an async spinner
    ros::init(argc, argv, "insert_collision_objects__planning_scene");
    ros::NodeHandle nh;
    ros::AsyncSpinner spin(1);

    // variables for planning scene addition of objects
    CollisionTasks ct;
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    const std::string BIN_MESH_PATH   = "package://ur5_picknplace/models/small_dumpster/meshes/small_dumpster.dae";
    const std::string TABLE_MESH_PATH = "package://ur5_picknplace/models/cafe_table/meshes/cafe_table.dae";



    spin.start();
    ros::Duration(1.0).sleep();


    // ur5_base
    geometry_msgs::Pose top_plate_pose;
    std::vector<_Float64> top_plate_dims;
    top_plate_pose.position.x = 0.0; 
    top_plate_pose.position.y = 0.0; 
    top_plate_pose.position.z = 0.58;
    top_plate_pose.orientation.w = 1;
    top_plate_pose.orientation.x = 0; 
    top_plate_pose.orientation.y = 0; 
    top_plate_pose.orientation.z = 0;
    top_plate_dims.push_back(0.5); 
    top_plate_dims.push_back(0.5); 
    top_plate_dims.push_back(0.02);     
    ct.SetPrimitiveCollisionObject("ur5_base_top_plate", ct.BOX, top_plate_pose, top_plate_dims);
    geometry_msgs::Pose leg_pose;
    std::vector<_Float64> leg_dims;
    std::string leg;
    for (int i = 0; i < 4; i++) {
      switch(i) {
        case 0:
          leg_pose.position.x = 0.22;
          leg_pose.position.y = 0.22;
          leg = "leg1";
          break;
        case 1:
          leg_pose.position.x = -0.22;
          leg_pose.position.y = 0.22;
          leg = "leg2";          
          break;        
        case 2:
          leg_pose.position.x = 0.22;
          leg_pose.position.y = -0.22;
          leg = "leg3";
          break;
        case 3:
          leg_pose.position.x = -0.22;
          leg_pose.position.y = -0.22;
          leg = "leg4";
          break;
      }
      leg_pose.position.z = 0.285;
      leg_pose.orientation.w = 1;
      leg_pose.orientation.x = 0; 
      leg_pose.orientation.y = 0; 
      leg_pose.orientation.z = 0;
      leg_dims.push_back(0.04);
      leg_dims.push_back(0.04);
      leg_dims.push_back(0.57);
      ct.SetPrimitiveCollisionObject(leg, ct.BOX, leg_pose, leg_dims);
    }


    // kinect pilar 
    geometry_msgs::Pose kinect_pilar_pose;
    std::vector<_Float64> kinect_pilar_dims;
    kinect_pilar_pose.position.x = 1.39; 
    kinect_pilar_pose.position.y = 0; 
    kinect_pilar_pose.position.z = 0.5;
    kinect_pilar_pose.orientation.w = 1;
    kinect_pilar_pose.orientation.x = 0; 
    kinect_pilar_pose.orientation.y = 0; 
    kinect_pilar_pose.orientation.z = 0;
    kinect_pilar_dims.push_back(1); // height
    kinect_pilar_dims.push_back(0.04); // radius
    ct.SetPrimitiveCollisionObject("kinect_pilar", ct.CYLINDER, kinect_pilar_pose, kinect_pilar_dims);

    // dumpster 
    geometry_msgs::Pose bin_mesh_pose;
    tf2::Quaternion bin_mesh_quaternion; 
    bin_mesh_pose.position.x = -21.85;
    bin_mesh_pose.position.y = 58.30;
    bin_mesh_pose.position.z = -27.745;
    bin_mesh_quaternion.setRPY(1.5707, 0, -1.5707);
    bin_mesh_pose.orientation = tf2::toMsg(bin_mesh_quaternion);
    ct.SetMeshCollisionObject("dumpster", BIN_MESH_PATH, bin_mesh_pose, 0.0055);


    // table
    geometry_msgs::Pose table_pose;
    table_pose.position.x = 0.7;
    table_pose.position.y = 0.0;
    table_pose.position.z = -22.065;
    table_pose.orientation.w = 1.0;
    table_pose.orientation.x = 0.0;
    table_pose.orientation.y = 0.0;
    table_pose.orientation.z = 0.0;    
    ct.SetMeshCollisionObject("cafe_table", TABLE_MESH_PATH, table_pose, 0.025);

    // cubes 
    geometry_msgs::Pose cube1_pose;
    std::vector<_Float64> cube1_dims;
    cube1_pose.position.x = 0.4; 
    cube1_pose.position.y = -0.2; 
    cube1_pose.position.z = 0.775+0.05/2;
    cube1_pose.orientation.w = 1;
    cube1_pose.orientation.x = 0; 
    cube1_pose.orientation.y = 0; 
    cube1_pose.orientation.z = 0;
    cube1_dims.push_back(0.05); 
    cube1_dims.push_back(0.05); 
    cube1_dims.push_back(0.05);     
    ct.SetPrimitiveCollisionObject("cube1", ct.BOX, cube1_pose, cube1_dims);


    geometry_msgs::Pose cube2_pose;
    std::vector<_Float64> cube2_dims;
    cube2_pose.position.x = 0.4; 
    cube2_pose.position.y = 0; 
    cube2_pose.position.z = 0.775+0.05/2;
    cube2_pose.orientation.w = 1;
    cube2_pose.orientation.x = 0; 
    cube2_pose.orientation.y = 0; 
    cube2_pose.orientation.z = 0;
    cube2_dims.push_back(0.05); 
    cube2_dims.push_back(0.05); 
    cube2_dims.push_back(0.05);     
    ct.SetPrimitiveCollisionObject("cube2", ct.BOX, cube2_pose, cube2_dims);

    geometry_msgs::Pose cube3_pose;
    std::vector<_Float64> cube3_dims;
    cube3_pose.position.x = 0.4; 
    cube3_pose.position.y = 0.2; 
    cube3_pose.position.z = 0.775+0.05/2;
    cube3_pose.orientation.w = 1;
    cube3_pose.orientation.x = 0; 
    cube3_pose.orientation.y = 0; 
    cube3_pose.orientation.z = 0;
    cube3_dims.push_back(0.05); 
    cube3_dims.push_back(0.05); 
    cube3_dims.push_back(0.05);     
    ct.SetPrimitiveCollisionObject("cube3", ct.BOX, cube3_pose, cube3_dims);

    planning_scene.addCollisionObjects(ct.collision_objects);

    ros::Duration(1.0).sleep();
    ros::shutdown();
    return 0;
}
