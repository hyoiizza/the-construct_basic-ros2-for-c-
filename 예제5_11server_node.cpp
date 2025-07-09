#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <chrono>

using Spin = services_quiz_srv::srv::Spin;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class ServerNode: public rclcpp::Node
{
public:
    ServerNode() : Node("ServerNode")
    {
        srv_ = create_service<services_quiz_srv::srv::Spin>("rotate",std::bind(&ServerNode::rotate_callback,this,_1,_2));
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    
    }
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Service<services_quiz_srv::srv::Spin>::SharedPtr srv_;

    void rotate_callback(
        const std::shared_ptr<Spin::Request> request,
        const std::shared_ptr<Spin::Response> response)
    {
        auto message = geometry_msgs::msg::Twist();
        if (request->direction == "right"){
            message.angular.z = std::abs(request->angular_velocity);
            publisher_->publish(message);
            
        } else if (request->direction == "left"){
            message.angular.z = -std::abs(request->angular_velocity);
            publisher_->publish(message);
            
        } else {
            message.angular.z = 0 ;
            publisher_ -> publish(message);
        }

        std::this_thread::sleep_for(std::chrono::duration<int>(request->time));

        message.angular.z = 0.0;
        publisher_->publish(message); // 멈추는 명령

        response->success= true;
    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServerNode>());
    rclcpp::shutdown();

    return 0;
}