// Pinocchio example

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/geometry.hpp"

int main(int argc, char * argv[])
{
  // Create a set of Pinocchio models and data
  pinocchio::Model model;
  pinocchio::Data data(model);
  pinocchio::GeometryModel collision_model;
  pinocchio::GeometryData collision_data(collision_model);

  // Uncomment and try to compile collision code
  // Eigen::VectorXd q(2);
  // auto result = pinocchio::computeCollisions(model, data, collision_model, collision_data, q);

  return 0;
}