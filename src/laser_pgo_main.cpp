#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "aloam_velodyne/laser_pgo_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto node = std::make_shared<aloam_velodyne::LaserPGONode>(options);
  rclcpp::spin(node);

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
