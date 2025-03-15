#include <filesystem>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/geometry.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/collision/broadphase.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/parsers/srdf.hpp"
 
int main(int /*argc*/, char* /*argv*/ [])
{
  // Get the URDF and SRDF file paths.
  const auto package_share_path = ament_index_cpp::get_package_share_directory("pinocchio_ros_example");
  const auto urdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.urdf";
  const auto srdf_path = std::filesystem::path(package_share_path) / "ur_robot_model" / "ur5_gripper.srdf";

  // Create a set of Pinocchio models and data.
  pinocchio::Model model;
  pinocchio::urdf::buildModel(urdf_path, model);

  pinocchio::GeometryModel visual_model;
  pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::VISUAL, visual_model);

  pinocchio::GeometryModel collision_model;
  pinocchio::urdf::buildGeom(model, urdf_path, pinocchio::COLLISION, collision_model);
  collision_model.addAllCollisionPairs();
  pinocchio::srdf::removeCollisionPairs(model, collision_model, srdf_path);

  pinocchio::Data data(model);
  pinocchio::GeometryData collision_data(collision_model);

  // Get a joint configuration.
  Eigen::VectorXd q(model.nq);
  q << 0.0, -0.785, -3.14, -1.57, 1.57, 0.0;
  std::cout << "Joint configuration: " << std::endl << q << std::endl << std::endl;

  // Get the frame ID of the end effector for later lookups.
  const auto ee_frame_id = model.getFrameId("ee_link");

  // Perform forward kinematics and get a transform.
  pinocchio::framesForwardKinematics(model, data, q);
  std::cout << "Frame transform: " << std::endl << data.oMf[ee_frame_id] << std::endl;

  // Get a Jacobian at a specific frame.
  Eigen::MatrixXd ee_jacobian(6, model.nv);
  pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, ee_jacobian);
  std::cout << "Frame Jacobian: " << std::endl << ee_jacobian << std::endl << std::endl;

  // Check collisions.
  pinocchio::computeCollisions(model, data, collision_model, collision_data, q);
  for (size_t k = 0; k < collision_model.collisionPairs.size(); ++k)
  {
    const auto& cp = collision_model.collisionPairs[k];
    const auto& cr = collision_data.collisionResults[k];
    if (cr.isCollision())
    {
      const auto& body1 = collision_model.geometryObjects[cp.first].name;
      const auto& body2 = collision_model.geometryObjects[cp.second].name;
      std::cout << "Collision detected between " << body1 << " and " << body2 << std::endl;
    }
  }

  return 0;
}