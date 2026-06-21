#include "rclcpp/rclcpp.hpp"

class MultiRobotCoordinator : public rclcpp::Node{
  public:
	  MultiRobotCoordinator() : Node ("coordinator"){
	  }

  private:
};

int main (int argc, char ** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiRobotCoordinator>());
  rclcpp::shutdown();
  return 0;
}

