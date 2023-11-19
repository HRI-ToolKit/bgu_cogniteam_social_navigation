#include <cstdio>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

using namespace std;


class SocialNavigationManager : public rclcpp::Node
{


public:
  SocialNavigationManager(): Node("social_navigation_manager")
  { 

    costmap_ros_->configure();
    
  }

  ~SocialNavigationManager(){}

private:


private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

};


int main(int argc, char * argv[])
{
  cerr<<"1111 "<<endl;
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SocialNavigationManager>());
  rclcpp::shutdown();
  return 0;
}