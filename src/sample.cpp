#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/time.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class Nav2Client : public rclcpp::Node
{
public:

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  Nav2Client()
  : Node("nav2_send_goal")
  {
  }

  void sendGoal(void) {
    //アクション Client の作成
    //rclcpp_action::create_client (rclcpp::Node::SharedPtr node, const std::string &name, rclcpp::callback_group::CallbackGroup::SharedPtr group=nullptr)
    navigate_to_pose_client_ptr_ = rclcpp_action::create_client<NavigateToPose>(shared_from_this(), "navigate_to_pose");

    // アクションが提供されているまでに待つ
    while (!this->navigate_to_pose_client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
      // シャットダウンされたかどうか確認する
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(get_logger(), "Interrupted while waiting for action server.");
        rclcpp::shutdown();
      }
      RCLCPP_INFO(get_logger(), "Waiting for action server...");
    }

    //アクション　Goalの作成
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.stamp = now();
    goal_msg.pose.header.frame_id = "map";

    goal_msg.pose.pose.position.x = -2;
    goal_msg.pose.pose.position.y = 0;
    goal_msg.pose.pose.orientation.w = 1.0;

    //進捗状況のFeedbackコールバックを設定
    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.feedback_callback =
      std::bind(&Nav2Client::feedback_callback, this, _1, _2);
    //Goal をサーバーに送信
    auto goal_handle_future = navigate_to_pose_client_ptr_->async_send_goal(goal_msg, send_goal_options);

    // Goalがサーバーでアクセプトされるまでに待つ
    if (rclcpp::spin_until_future_complete(shared_from_this(), goal_handle_future) !=
        rclcpp::executor::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Send goal call failed");
      rclcpp::shutdown();
    }
  }

  void feedback_callback(
    GoalHandleNavigateToPose::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
  {
    RCLCPP_INFO(get_logger(), "Distance remaininf = %f",
      feedback->distance_remaining);
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr navigate_to_pose_client_ptr_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Nav2Client>();
  node->sendGoal();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

