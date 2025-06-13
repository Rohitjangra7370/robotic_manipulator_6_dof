#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

class PoseGoalCommander : public rclcpp::Node
{
public:
  PoseGoalCommander()
    : Node("pose_goal_commander")
  {
    // Nothing here that calls shared_from_this()
  }

  void initialize()
  {
    // Now it's safe to use shared_from_this()
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "manipulator"); // Change "manipulator" to your planning group name

    subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
      "/pose_goal", 10,
      std::bind(&PoseGoalCommander::pose_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "PoseGoalCommander initialized and listening on /pose_goal");
  }

private:
  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received pose goal: [%.3f, %.3f, %.3f]", msg->position.x, msg->position.y, msg->position.z);

    move_group_->setPoseTarget(*msg);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
      RCLCPP_INFO(this->get_logger(), "Plan found, executing...");
      move_group_->move();
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Planning failed!");
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PoseGoalCommander>();
  node->initialize(); // Safe to call shared_from_this() now
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
