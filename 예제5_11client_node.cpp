#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "services_quiz_srv/srv/spin.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <chrono>

using namespace std::placeholders;

class ClientNode : public rclcpp::Node
{
public:
    ClientNode(std::string direction, float angular_velocity, int time)
    : Node("ClientNode"), direction_(direction), angular_velocity_(angular_velocity), time_(time)
    {   
        //client 생성자 정의
        client_ = this->create_client<services_quiz_srv::srv::Spin>("rotate");

        //서버 준비 됐는지 확인
        while(!client_->wait_for_service(1s)){
            if (!rclcpp::ok()){
                RCLCPP_ERROR(this->get_logger(),"Client interrupted while waiting for service. Terminating...");
			    return;
            } else{
                RCLCPP_INFO(this->get_logger(),"Service unavailable. Waiting for Service...");
            }
        }
    }

    //비동기 방식으로 요청 보내기 
    void async_send_request(){
        auto request = std::make_shared<services_quiz_srv::srv::Spin::Request>();
        request->direction = direction_;
        request->angular_velocity = angular_velocity_;
        request->time = time_;

        auto result = client_->async_send_request(request, std::bind(&ClientNode::handle_response, this, std::placeholders::_1));
    }

private:
    rclcpp::Client<services_quiz_srv::srv::Spin>::SharedPtr client_;
    
    std::string direction_;
    float angular_velocity_;
    int time_;

    bool service_done_ = false;
    bool service_called_ = false;


    //응답 처리 함수
    void handle_response(rclcpp::Client<services_quiz_srv::srv::Spin>::SharedFuture future)
    {
        auto response = future.get();
        if (response->success){
            RCLCPP_INFO(this->get_logger(), "Request succeeded.");
        } else {
            RCLCPP_WARN(this->get_logger(), "Request failed.");
        }
        service_done_ = true;
    }
};

int main(int argc, char*argv[])
{   
    rclcpp::init(argc,argv);

    std::string direction = argv[1];
    float angular_velocity = std::stof(argv[2]);
    int time = std::stoi(argv[3]);

    auto node = std::make_shared<ClientNode>(direction, angular_velocity, time);
    node->async_send_request();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}