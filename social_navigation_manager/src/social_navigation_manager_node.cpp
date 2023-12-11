#include "rclcpp/rclcpp.hpp"
// Include PointCloud2 message
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "std_msgs/msg/string.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

#include <memory>
#include <chrono>
// this brings in a boost system dependency,
// will get undefined reference to `boost::system::generic_category()
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
using namespace std;
using std::placeholders::_1;

struct Person {
    float x;
    float y;
    float a;
    float b;
    float deg;
};

class PersonsPointCloudGenerator : public rclcpp::Node
{
public:
  PersonsPointCloudGenerator() : Node("generate_point_cloud2")
  {    
    persons_subscriber_ = create_subscription<std_msgs::msg::String>(
      "/persons", 10, std::bind(&PersonsPointCloudGenerator::persons_callback, this, 
      std::placeholders::_1));

    perosns_pc_pub_ = 
      create_publisher<sensor_msgs::msg::PointCloud2>("/perosn_cloud",10);

  }

  ~PersonsPointCloudGenerator() {}

private:

  void persons_callback(const std_msgs::msg::String::SharedPtr msg){

    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg;   
    pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pc2_msg->header.frame_id = frame_id_;


    // Extracting the substring between '[' and ']'
    size_t startPos = msg->data.find("[") + 1;
    size_t endPos = msg->data.find("]");
    std::string pointsSubstring = msg->data.substr(startPos, endPos - startPos);

    // Tokenizing the substring based on commas and parentheses
    std::istringstream iss(pointsSubstring);
    std::string token;

    while (std::getline(iss, token, ',')) {
        // Extracting x and y values from the token
        double x, y;
        token.erase(std::remove(token.begin(), token.end(), '\''), token.end());

        std::stringstream ss(token);

        char underscore;

        // Parsing the numbers
        ss >> x >> underscore >> y;
      
        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;
        //cerr<<token<<endl;

        const uint8_t& pixel_r = 255;
        const uint8_t& pixel_g = 0;
        const uint8_t& pixel_b = 255;
        // Define point color
        uint32_t rgb = (static_cast<uint32_t>(pixel_r) << 16
            | static_cast<uint32_t>(pixel_g) << 8
            | static_cast<uint32_t>(pixel_b));
        pt.rgb = *reinterpret_cast<float*>(&rgb);


        
        cloud.points.push_back(pt);
    }

    pcl::toROSMsg(cloud, *pc2_msg);   

    pc2_msg->header.stamp = now();
    pc2_msg->header.frame_id = frame_id_;
    perosns_pc_pub_->publish(*pc2_msg);   
   
  }
  // void persons_callback(const std_msgs::msg::String::SharedPtr msg){
    
   
  //   pcl::PointCloud<pcl::PointXYZRGB> cloud;
  //   sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg;   
  //   pc2_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
  //   pc2_msg->header.frame_id = frame_id_;

  //   // Vector to store the structs
  //   std::vector<Person> persons;

  //   // Tokenize the string using ';' as a delimiter
  //   std::istringstream tokenStream(msg->data);
  //   std::string token;

  //   while (std::getline(tokenStream, token, ';')) {
  //     // Tokenize each element using ',' as a delimiter
  //     std::istringstream elementStream(token);
  //     std::string element;

  //     Person person;

  //     for (int i = 0; i < 5; ++i)
  //     {
  //       if (std::getline(elementStream, element, ',')) {
  //         // Convert the string to float and store in the struct
  //         switch (i)
  //         {
  //             case 0:
  //                 person.x = std::stof(element);
  //                 break;
  //             case 1:
  //                 person.y = std::stof(element);
  //                 break;
  //             case 2:
  //                 person.a = std::stof(element);
  //                 break;
  //             case 3:
  //                 person.b = std::stof(element);
  //                 break;
  //             case 4:
  //                 person.deg = std::stof(element);
  //                 break;
  //             default:
  //                 break;
  //         }
  //       }
  //     }

  //     persons.push_back(person);
  //   }


  //   for (const auto& person : persons) {
  //     createEllipse(cloud, person.x, person.y, person.deg, person.a,
  //       person.b, 0.05);
  //   }         

  //   pcl::toROSMsg(cloud, *pc2_msg);   

  //   pc2_msg->header.stamp = now();
  //   pc2_msg->header.frame_id = frame_id_;
  //   perosns_pc_pub_->publish(*pc2_msg);

  //   cerr<<"publish cloud "<<endl;
  
  // }


  void createEllipse(pcl::PointCloud<pcl::PointXYZRGB> &cloud,
    float x_center, float y_center, float angle_degrees,
    float a, float b, float resolution) {    

    // Orientation angle in degrees
    double angle_radians = angle_degrees * M_PI / 180.0;

    for (double theta = 0; theta <= 2 * M_PI; theta += resolution) {
        double x = x_center + a * cos(theta) * cos(angle_radians) - b * sin(theta) * sin(angle_radians);
        double y = y_center + a * cos(theta) * sin(angle_radians) + b * sin(theta) * cos(angle_radians);

        pcl::PointXYZRGB pt;
        pt.x = x;
        pt.y = y;
        pt.z = 0.0;

        const uint8_t& pixel_r = 255;
        const uint8_t& pixel_g = 0;
        const uint8_t& pixel_b = 255;
        // Define point color
        uint32_t rgb = (static_cast<uint32_t>(pixel_r) << 16
            | static_cast<uint32_t>(pixel_g) << 8
            | static_cast<uint32_t>(pixel_b));
        pt.rgb = *reinterpret_cast<float*>(&rgb);
        cloud.points.push_back(pt);
    }   

  }
 

private:  

  std::string frame_id_ = "map";

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr persons_subscriber_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr perosns_pc_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

 
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::spin(std::make_shared<PersonsPointCloudGenerator>());
  rclcpp::shutdown();
  return 0;
}
