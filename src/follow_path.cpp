#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "turtlesim/msg/pose.hpp"


using namespace std::chrono_literals;

class TurtleGoToGoal : public rclcpp::Node
{

turtlesim::msg::Pose goal_pose;

public:
  TurtleGoToGoal(double goal_x, double goal_y, float speed)
    : Node("turtle_go_to_goal")
  {
    cmdvel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    pose_subs_ = this->create_subscription<turtlesim::msg::Pose>(
      "/turtle1/pose", 10, std::bind(&TurtleGoToGoal::pose_callback, this, std::placeholders::_1));
    
    goal_pose.x = goal_x;
    goal_pose.y = goal_y;

    
    speed_ = speed;
    
    timer_ = this->create_wall_timer(300ms, std::bind(&TurtleGoToGoal::move2goal, this));

    flag_ = false; 

  }

private:
  void pose_callback(const turtlesim::msg::Pose::SharedPtr data)
  {
    pose_.x = data->x;
    pose_.y = data->y;
    pose_.theta = data->theta;
    RCLCPP_INFO(this->get_logger(), "X: %.3f, Y: %.3f", data->x, data->y);
  }


  double euclidean_distance(const turtlesim::msg::Pose& goal_pose)
  {
    return std::sqrt(std::pow((goal_pose.x - pose_.x), 2) + std::pow((goal_pose.y - pose_.y), 2));
  }

  double linear_vel(const turtlesim::msg::Pose& goal_pose, double constant = 2.0)
  {
    return constant * euclidean_distance(goal_pose);
  }

  double steering_angle(const turtlesim::msg::Pose& goal_pose)
  {
    return std::atan2(goal_pose.y - pose_.y, goal_pose.x - pose_.x);
  }

  double angular_vel(const turtlesim::msg::Pose& goal_pose, double constant = 2.0)
  {
    return constant * (steering_angle(goal_pose) - pose_.theta);
  }

  void move2goal()
  {

    double distance_tolerance = 0.1;
    double angular_tolerance = 0.01;

    auto vel_msg = std::make_shared<geometry_msgs::msg::Twist>();

    if (std::abs(steering_angle(goal_pose) - pose_.theta) > angular_tolerance) {
      vel_msg->linear.x = 0.0;
      vel_msg->linear.y = 0.0;
      vel_msg->linear.z = 0.0;
      vel_msg->angular.z = angular_vel(goal_pose);
    } else {
      vel_msg->angular.z = 0.0;
      if (std::abs(euclidean_distance(goal_pose)) >= distance_tolerance && abs(euclidean_distance(goal_pose)) >= 0.5) {
        vel_msg->linear.x = speed_;    
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
      } 
      else if (std::abs(euclidean_distance(goal_pose)) >= distance_tolerance && abs(euclidean_distance(goal_pose)) <= 0.5){
        vel_msg->linear.x = 0.5;    
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
      }
      
      else {
        vel_msg->linear.x = 0.0;
        vel_msg->linear.y = 0.0;
        vel_msg->linear.z = 0.0;
        flag_ = true;
	rclcpp::shutdown();
     }
    }

  cmdvel_pub_->publish(*vel_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmdvel_pub_;
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subs_;
  rclcpp::TimerBase::SharedPtr timer_;

  turtlesim::msg::Pose pose_;
  bool flag_;

  turtlesim::msg::Pose goal_pose_;
  double speed_;
};


int main(int argc, char** argv)
{


  int num_points;
  std::cout << "How many points do you want to go to? ";
  std::cin >> num_points;

  double speed;
  std::cout << "Set your speed: ";
  std::cin >> speed;

  std::vector<double> goal_x(num_points);
  std::vector<double> goal_y(num_points);

  for (int i = 0; i < num_points; i++) {
    std::cout << "Set x coordinate for point " << i + 1 << ": ";
    std::cin >> goal_x[i];
    std::cout << "Set y coordinate for point " << i + 1 << ": ";
    std::cin >> goal_y[i];
  }
  for (int i = 0; i < num_points; i++){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<TurtleGoToGoal>(goal_x[i], goal_y[i], speed);
    rclcpp::spin(node);
    rclcpp::shutdown();

  }
    rclcpp::shutdown();

  return 0;
}

