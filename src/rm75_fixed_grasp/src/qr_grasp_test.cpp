#include <memory>

#include <rclcpp/rclcpp.hpp>

// Kept only to replace older installed test binaries safely. The former version
// executed a trajectory immediately after receiving a tag TF.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("qr_grasp_test_disabled");
  RCLCPP_ERROR(
    node->get_logger(),
    "qr_grasp_test is disabled because it bypassed the hardware motion interlock. "
    "Use: ros2 launch rm75_fixed_grasp fixed_grasp.launch.py");
  rclcpp::shutdown();
  return 2;
}
