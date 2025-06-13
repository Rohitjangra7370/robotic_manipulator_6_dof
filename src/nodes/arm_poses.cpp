#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

class MoveToPoseNode : public rclcpp::Node
{
public:
  MoveToPoseNode()
  : Node("move_to_pose_node")
  {}

  void do_motion()
  {
    // Create MoveGroupInterface using shared_from_this()
    auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "manipulator");

    // Define the target pose
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 0.4;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.4;
    target_pose.orientation.x = 0.0;
    target_pose.orientation.y = 0.0;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 1.0;

    move_group.setPoseTarget(target_pose);

    // Plan and execute the motion
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Plan found, executing...");
      move_group.execute(plan);
    }
    else
    {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Create the node as a shared_ptr so shared_from_this() works
  auto node = std::make_shared<MoveToPoseNode>();

  // Call the motion function after construction
  node->do_motion();

  // Keep node alive (optional, in case you want to add subscriptions later)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
